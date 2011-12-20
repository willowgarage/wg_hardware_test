#! /usr/bin/env python

from __future__ import with_statement

PKG = 'mtrace_tools'
import roslib; roslib.load_manifest(PKG)
import rospy
from ethercat_hardware.msg import MotorTrace
import ethercat_hardware.msg
import getopt
import rosbag
from mtrace_recorder import MtraceRecorder

WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)

import re

import sys, os

import wx
from wx import richtext
import threading

import math, time
import copy

import matplotlib
matplotlib.use('WX')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigCanvas, \
    NavigationToolbar2WxAgg as NavigationToolbar

import pylab
import numpy

class MotorTracePlotException(Exception): pass    

##\brief Container class holds plot data, color, legend for one line on plot
class MtracePlotData:
    __slots__ = [ 'xdata', 'ydata', 'color', 'legend' ]
    def __init__(self, xdata = [], ydata = [], color = 'b-', legend = '_nolegend_'):
        self.xdata = xdata
        self.ydata = ydata
        self.color = color
        self.legend = legend
    
##\brief Converts trace struct into arrays
class struct_of_arrays(object):
    def __init__(self, list_of_structs):
        if type(list_of_structs).__name__ != 'list':
            print "object is not a list"
            return
        if len(list_of_structs) == 0:
            print "list is empty"
            return
        # pull in non __XXXX__ names, define them in this class as empty lists
        names = dir(list_of_structs[0])
        struct_names=[]
        for name in names:
            if name[0] != '_':
                struct_names.append(name)
                self.__setattr__(name,[])
        # Append values to lists
        for struct in list_of_structs:
            for name in struct_names:
                tmp = self.__getattribute__(name)
                tmp.append(struct.__getattribute__(name))
        # Convert lists into arrays        
        for name in struct_names:
            tmp = self.__getattribute__(name)
            array_tmp = numpy.array(tmp)
            self.__setattr__(name,array_tmp)

# Analyzed data from a trace
class MotorTraceAnalyzed:
    def __init__(self, data):
        size = len(data.samples)
        if size <= 1 :    
            raise MotorTracePlotException("No data in message")

        a = struct_of_arrays(data.samples)

        self.stamp = data.header.stamp.to_sec()
        self.reason = data.reason

        self.motor_voltage_error_limit = a.motor_voltage_error_limit
        
        # Start time from 0
        self.time                   = a.timestamp - a.timestamp[0]
        self.measured_motor_voltage = a.measured_motor_voltage
        self.supply_voltage         = a.supply_voltage
        self.measured_current       = a.measured_current
        self.executed_current       = a.executed_current
        self.programmed_pwm         = a.programmed_pwm
        self.velocity               = a.velocity
        self.encoder_position       = a.encoder_position
        self.encoder_errors         = a.encoder_error_count
        
        self.filtered_voltage_error     = a.filtered_motor_voltage_error
        self.filtered_abs_voltage_error = a.filtered_abs_motor_voltage_error
        self.filtered_current_error     = a.filtered_current_error
        self.filtered_abs_current_error = a.filtered_abs_current_error
        
        self.actuator_info = data.actuator_info
        self.board_info = data.board_info
                
        self.backemf_constant = 1.0 / (self.actuator_info.speed_constant * 2.0 * math.pi * 1.0/60.0);
        self.resistance = self.actuator_info.motor_resistance
        self.board_resistance = self.board_info.board_resistance

        # Estimate motor resistance and backemf constant from data + allow voltage offset
        A = numpy.array([self.measured_current, self.velocity, numpy.ones(len(self.velocity))])
        x = numpy.linalg.lstsq(A.transpose(), self.measured_motor_voltage)

        self.resistance_est = x[0][0]
        self.backemf_const_est = -x[0][1]
        self.voltage_offset_est = x[0][2]
 
        # Estimate motor resistance from data
        A = numpy.array([self.measured_current])
        x = numpy.linalg.lstsq(A.transpose(), self.measured_motor_voltage - self.velocity * self.backemf_constant)

        self.resistance_est_no_offset = x[0][0]

        # Estimate board resistance (resistance of MOSFETs and inductors)
        A = numpy.array([self.measured_current, numpy.ones(len(self.measured_current))])
        B = self.supply_voltage * self.programmed_pwm - self.measured_motor_voltage;  # 
        x = numpy.linalg.lstsq(A.transpose(), B)

        self.board_resistance_est = x[0][0]
        
        self.board_output_voltage  = self.supply_voltage * self.programmed_pwm - self.board_resistance * self.measured_current
        self.backemf_voltage = self.velocity * self.actuator_info.encoder_reduction * self.backemf_constant 
        self.resistance_voltage = self.resistance * self.measured_current
        self.motor_model_voltage = self.backemf_voltage + self.resistance_voltage    
        
        self.angular_position = numpy.mod(self.encoder_position, 2.0 * math.pi)

        # Make cycle-by-cycle estimate of motor resistance    
        self.est_resistance = numpy.zeros(len(self.measured_current))
        self.est_resistance_accuracy = numpy.zeros(len(self.measured_current))
        # Don't calculate reistance if there is not enough motor current
        min_current_for_est = 0.02 * self.board_info.hw_max_current + 0.005
        for i in range(0,len(self.measured_current)-1):
            if (abs(self.measured_current[i]) > min_current_for_est):
                self.est_resistance[i] = (self.board_output_voltage[i] - self.backemf_voltage[i]) / self.measured_current[i]
                self.est_resistance_accuracy[i] = 1.0 / (1.0 + abs(self.backemf_voltage[i] / self.resistance_voltage[i]))
                


                


##\brief Plots motor voltage, and error
def plot_motor_voltage(data):
    voltage_data = MtracePlotData()
    voltage_data.xdata = data.time
    voltage_data.ydata = data.measured_motor_voltage
    voltage_data.color = 'r-'
    voltage_data.legend = 'Measured Motor (by ADC on board)'

    board_voltage = MtracePlotData()
    board_voltage.xdata = data.time
    board_voltage.ydata = data.board_output_voltage
    board_voltage.color = 'g-'
    board_voltage.legend = 'Board Output (PWM * Vsupply - R_brd*I)'

    model_data = MtracePlotData()
    model_data.xdata = data.time
    model_data.ydata = data.motor_model_voltage
    model_data.color = 'b-'
    model_data.legend = 'Motor Model (backEMF + R*I)'

    motor_board_output = MtracePlotData()
    motor_board_output.xdata = data.time
    motor_board_output.ydata = data.measured_motor_voltage - data.board_output_voltage
    motor_board_output.color = 'r-'
    motor_board_output.legend = 'Measured Motor - Board Output'

    model_board_output = MtracePlotData()
    model_board_output.xdata = data.time
    model_board_output.ydata = data.motor_model_voltage - data.board_output_voltage
    model_board_output.color = 'b-'
    model_board_output.legend = 'Motor Model - Board Output'

    motor_error_limit = MtracePlotData()
    motor_error_limit.xdata = data.time
    motor_error_limit.ydata = data.motor_voltage_error_limit
    motor_error_limit.color = 'k-'
    
    motor_error_limit_low = copy.deepcopy(motor_error_limit)
    motor_error_limit_low.ydata = -1 * motor_error_limit.ydata 
    motor_error_limit_low.legend = 'Motor Voltage Error Limit'

    name = "Motor Voltage: %s" % data.actuator_info.name
    
    return name, [ [voltage_data, board_voltage, model_data], 
                   [motor_board_output, model_board_output, motor_error_limit, motor_error_limit_low] ]


##\brief Plots motor current, and error
def plotMotorCurrent(data):
    executed_current_data = MtracePlotData()
    executed_current_data.xdata = data.time
    executed_current_data.ydata = data.executed_current
    executed_current_data.color = 'r-'
    executed_current_data.legend = 'Executed Motor Current'

    measured_current_data = MtracePlotData()
    measured_current_data.xdata = data.time
    measured_current_data.ydata = data.measured_current
    measured_current_data.color = 'b-'
    measured_current_data.legend = 'Measured Motor Current'

    
    #title("Motor Current Error")
    #plot(t,measured_current-executed_current,'b-', label='Measured-Executed')
    #plot(t,filtered_current_error,'r-',            label='Trace : Error (Filter)')
    #plot(t,filtered_abs_current_error,'g-',        label='Trace : Error (Abs Filter)')
    #legend(loc='best')

    measured_current_error_data = MtracePlotData()
    measured_current_error_data.xdata = data.time
    measured_current_error_data.ydata = data.measured_current - data.executed_current
    measured_current_error_data.color = 'r-'
    measured_current_error_data.legend = 'Measured-Executed Motor Current'
    
    
    name = "Motor Current: %s" % data.actuator_info.name
    
    return name, [ [executed_current_data, measured_current_data ], 
                   [measured_current_error_data] ]


##\brief Plots supply voltage
def plotSupplyVoltage(data):
    supply_voltage_data = MtracePlotData()
    supply_voltage_data.xdata = data.time
    supply_voltage_data.ydata = data.supply_voltage
    supply_voltage_data.color = 'r-'
    supply_voltage_data.legend = 'Supply Voltage'  
    
    name = "Supply Voltage: %s" % data.actuator_info.name
    
    return name, [ [supply_voltage_data ] ]



##\brief Plots encoder position, errors and velocity
def plotEncoderData(data):
    encoder_position_data = MtracePlotData()
    encoder_position_data.xdata = data.time
    encoder_position_data.ydata = data.encoder_position
    encoder_position_data.color = 'r-'
    encoder_position_data.legend = 'Encoder position'

    encoder_velocity_data = MtracePlotData()
    encoder_velocity_data.xdata = data.time
    encoder_velocity_data.ydata = data.velocity
    encoder_velocity_data.color = 'b-'
    encoder_velocity_data.legend = 'Velocity'

    encoder_error_data = MtracePlotData()
    encoder_error_data.xdata = data.time
    encoder_error_data.ydata = data.encoder_errors
    encoder_error_data.color = 'g-'
    encoder_error_data.legend = 'Encoder error count'
    
    name = "Encoder Position and Velocity : %s" % data.actuator_info.name
    
    return name, [ [encoder_position_data ],
                   [encoder_error_data ],
                   [encoder_velocity_data ] ]



##\brief Plots voltage, current error vs position
def plot_error_v_position(data):
    voltage_error = MtracePlotData()
    voltage_error.xdata = data.angular_position
    voltage_error.ydata = data.motor_model_voltage - data.board_output_voltage
    voltage_error.color = 'r.'
    voltage_error.legend = 'Voltage Error'

    current_error = MtracePlotData()
    current_error.xdata = data.angular_position
    current_error.ydata = data.measured_current - data.executed_current
    current_error.color = 'b.'
    current_error.legend = 'Current Error'

    name = "Error v. Position: %s" % data.actuator_info.name

    return name, [ [voltage_error], [current_error] ]

##\brief Plots motor resistance estimate
def plot_motor_resistance(data):
    name = "Motor Resistance: %s" % data.actuator_info.name

    resistance = MtracePlotData()
    resistance.xdata = data.time
    resistance.ydata = data.est_resistance
    resistance.color = 'r.'
    resistance.legend = 'Estimated Resistance'
    
    resistance_accuracy = MtracePlotData()
    resistance_accuracy.xdata = data.time
    resistance_accuracy.ydata = data.est_resistance_accuracy
    resistance_accuracy.color = 'g-'
    resistance_accuracy.legend = 'Resistance Est. Accuracy'

    return name, [ [resistance], [resistance_accuracy] ]

##\brief Plots voltage relative error
def plot_motor_voltage_rel_error(data):
    name = "Motor Voltage Relative Error: %s" % data.actuator_info.name
    
    measured = MtracePlotData()
    measured.xdata = data.time
    measured.ydata = data.measured_current
    measured.color = 'b-'
    measured.legend = 'Measured'

    executed = MtracePlotData()
    executed.xdata = data.time
    executed.ydata = data.executed_current
    executed.color = 'r-'
    executed.legend = 'Executed'
    
    return name, [ [ measured, executed ] ]
    
class MtraceStatsPanel(wx.Frame):
    def __init__(self, manager, mtrace_data):
        wx.Frame.__init__(self, manager, -1, 'Trace Stats : ' + mtrace_data.actuator_info.name)
        
        self._manager = manager

        self._data = mtrace_data

        self._sizer = wx.BoxSizer(wx.VERTICAL)
        
        self._text_ctrl = richtext.RichTextCtrl(self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_READONLY)
        self._sizer.Add(self._text_ctrl, 1, wx.EXPAND)
        
        self.SetSizer(self._sizer)

        self._write_stats()

    def _write_line(self, key, value):
        self._text_ctrl.BeginBold()
        self._text_ctrl.WriteText("%s: "%(key))
        self._text_ctrl.EndBold()
        
        self._text_ctrl.WriteText(value)
        
        self._text_ctrl.Newline()

    def _write_stats(self):
        self._write_line('Trace Info', ' ')

        stamp = self._data.stamp
        time_str = time.strftime("%m/%d/%Y %H:%M:%S", time.localtime(stamp))

        self._write_line('Timestamp', str(stamp))
        self._write_line('Time', time_str)
        self._write_line('Reason', self._data.reason)
        self._text_ctrl.Newline()

        self._write_line('MCB Info', ' ')
        self._write_line('Description', self._data.board_info.description)
        self._write_line('Product Code', str(self._data.board_info.product_code))
        self._write_line('PCA', str(self._data.board_info.pca))
        self._write_line('PCB', str(self._data.board_info.pcb))
        self._write_line('Serial', str(self._data.board_info.serial))
        self._write_line('Firmware Major', str(self._data.board_info.firmware_major))
        self._write_line('Firmware Minor', str(self._data.board_info.firmware_minor))
        self._write_line('Board Resistance', str(self._data.board_info.board_resistance))
        self._write_line('Max PWM Ratio', str(self._data.board_info.max_pwm_ratio))
        self._write_line('HW Max Current', str(self._data.board_info.hw_max_current))
        self._write_line('Poor Measured Motor Voltage', str(self._data.board_info.poor_measured_motor_voltage))
        self._text_ctrl.Newline()

        self._write_line('Motor Data', ' ')
        self._write_line('Acutuator Name', self._data.actuator_info.name)
        self._write_line('Robot Name', self._data.actuator_info.robot_name)
        self._write_line('Motor Make', self._data.actuator_info.motor_make)
        self._write_line('Motor Model', self._data.actuator_info.motor_model)
        self._write_line('Max Current', str(self._data.actuator_info.max_current))
        self._write_line('Speed Constant', str(self._data.actuator_info.speed_constant))
        self._write_line('Motor Resistance', str(self._data.actuator_info.motor_resistance))
        self._write_line('Motor Torque Constant', str(self._data.actuator_info.motor_torque_constant))
        self._write_line('Encoder Reduction', str(self._data.actuator_info.encoder_reduction))
        self._write_line('Pulses per Revolution', str(self._data.actuator_info.pulses_per_revolution))
                
        

##\brief Plot window of trace data
class MtracePlotPanel(wx.Frame):
    def __init__(self, manager, name, mtrace_data):
        wx.Frame.__init__(self, manager, -1, name)

        self._manager = manager

        self._data = mtrace_data

        self.Bind(wx.EVT_CLOSE, self.on_close)

        self.create_main_canvas()
        
    def init_plot(self):
        self.dpi = 100
        rc = matplotlib.figure.SubplotParams(left=0.05, bottom=0.05, right=0.99, top=0.95, wspace=0.001, hspace=0.1)
        self.fig = Figure((3.0, 3.0), dpi=self.dpi, subplotpars=rc)

        num_sub_plots = len(self._data)

        fp = matplotlib.font_manager.FontProperties(size=10)
        
        for i, subplot in enumerate(self._data):
            subplot_val = 100 * num_sub_plots + 10 + i + 1
            
            axes = self.fig.add_subplot(subplot_val)
            axes.set_axis_bgcolor('white')
            pylab.setp(axes.get_xticklabels(), fontsize=6)
            pylab.setp(axes.get_yticklabels(), fontsize=8)
            
            for data in subplot:
                axes.plot(data.xdata,
                          data.ydata,
                          data.color,
                          linewidth = 1,
                          picker = 5,
                          label = data.legend
                          )[0]
                axes.legend(loc='best', prop=fp)
                
            axes.grid(True, color='gray')
        pylab.setp(axes.get_xticklabels(), visible=True)
        
    def onpick(self, event = None):
        pass
         
    def create_main_canvas(self):
        self.init_plot()
        self.canvas = FigCanvas(self, -1, self.fig)
        self.canvas.mpl_connect('pick_event', self.onpick)

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.SetSizer(self.sizer)

        #self.add_toolbar()        

    def on_close(self, event):
        self.Destroy()
        pass



##\brief Manager of MTrace plot windows
class MtracePlotFrame(wx.Frame):
    def __init__(self, app, msg_recorder):
        wx.Frame.__init__(self, app, wx.ID_ANY, "Motor Trace Plotter")

        self._msg_recorder = msg_recorder

        self._plot_functions = { 'Motor Voltage and Error': plot_motor_voltage,
                                 'Motor Current and Error' : plotMotorCurrent,
                                 'Motor Resistance Estimate' : plot_motor_resistance,
                                 'Motor Voltage Relative Error': plot_motor_voltage_rel_error,
                                 'Motor Error v. Position': plot_error_v_position,
                                 'Encoder Position and Velocity' : plotEncoderData,
                                 'Supply Voltage' : plotSupplyVoltage
                                 }
                             
        plot_names = ['Statistics']
        plot_names.extend(self._plot_functions.keys())

        self._filter_string = ""
        self._filter_msg_list = []

        # Setting up the menu.
        file_menu= wx.Menu()

        open_menu_item = wx.MenuItem(file_menu, -1, text="&Open Bag File",help="Load MotorTrace messages bag file")
        self.Bind(wx.EVT_MENU, self.onOpen, open_menu_item)
        file_menu.AppendItem(open_menu_item)

        save_menu_item = wx.MenuItem(file_menu, -1, text="&Save Bag File",help="Save currently selected message to bag file")
        self.Bind(wx.EVT_MENU, self.onSave, save_menu_item)
        file_menu.AppendItem(save_menu_item)

        clear_menu_item = wx.MenuItem(file_menu, -1, text="&Clear Messages",help="Clear message list")
        self.Bind(wx.EVT_MENU, self.onClear, clear_menu_item)
        file_menu.AppendItem(clear_menu_item)


        #file_menu.AppendSeparator()
        #file_menu.Append(ID_EXIT,"E&xit"," Terminate the program")
        # Creating the menubar.
        menu_bar = wx.MenuBar()
        menu_bar.Append(file_menu,"&File") 
        self.SetMenuBar(menu_bar)  
                
        #wx.EVT_MENU(self, ID_TOPIC, self.OnChangeTopic)
        #wx.EVT_MENU(self, ID_EXIT, self.OnQuit)

        self._filter_textctrl_label = wx.StaticText(self, label="Message Filter:")
        self._filter_textctrl = wx.TextCtrl(self, style = wx.TE_PROCESS_ENTER)
        self._filter_textctrl.SetToolTip(wx.ToolTip("Regex filter for messages"))
        self._filter_textctrl.Bind(wx.EVT_TEXT_ENTER, self.onFilterEnter)

        self._msg_listbox_label = wx.StaticText(self, label="Message List:")
        self._msg_listbox = wx.ListBox(self, style=wx.LB_MULTIPLE)
        self._msg_listbox.SetToolTip(wx.ToolTip("Motor Msg to Analyze"))

        self._plot_listbox_label = wx.StaticText(self, label="Plot List:")
        self._plot_listbox = wx.ListBox(self, style=wx.LB_MULTIPLE)
        self._plot_listbox.SetToolTip(wx.ToolTip("Graphs to plot"))        
        self._plot_listbox.SetItems(plot_names)

        self._plot_button = wx.Button(self, label="Plot")
        self._plot_button.Bind(wx.EVT_BUTTON, self.onMakePlot)
        self._plot_button.SetToolTip(wx.ToolTip("Make motor trace plot"))


        self.updateMsgListBox()

        hsizer = wx.BoxSizer(wx.HORIZONTAL)
        hsizer.Add(self._filter_textctrl_label, 0, wx.EXPAND | wx.ALIGN_CENTER_VERTICAL | wx.TOP, 5)
        hsizer.Add(self._filter_textctrl, 1, wx.EXPAND)

        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(hsizer, 0, wx.EXPAND)
        vsizer.Add(self._msg_listbox_label, 0, wx.EXPAND)
        vsizer.Add(self._msg_listbox, 1, wx.EXPAND)
        vsizer.Add(self._plot_listbox_label, 0, wx.EXPAND)
        vsizer.Add(self._plot_listbox, 1, wx.EXPAND)
        vsizer.Add(self._plot_button, 0, wx.EXPAND)

        self.SetSizer(vsizer)
        self.SetAutoLayout(1)
        vsizer.SetMinSize((600,500))
        vsizer.Fit(self)

        # setup timer to check recorder for updates and update screen periodically 
        self.timer = wx.Timer(self,-1)
        self.Bind(wx.EVT_TIMER, self.onTimer, self.timer)
        self.timer.Start(1000)

        self.Show(True)


    def onOpen(self, event):
        dlg = wx.FileDialog(self, "Select bag file to open", style=(wx.FD_OPEN  | wx.FD_MULTIPLE) )
        if dlg.ShowModal() == wx.ID_OK:
            bag_filenames = os.path.join(dlg.GetPaths())
            try:
                for bag_filename in bag_filenames:
                    self._msg_recorder.loadBagFileAsync(bag_filename)                    
            except Exception, e:
                self.displayError("Error opening bag", str(e))
                return
            self.updateMsgListBox()
        dlg.Destroy()


    def onSave(self, event):
        msgs = self.getMsgSelections()
        if len(msgs) == 0:
            self.displayError("Error saving message", "No messages selected to save")
            return
        dlg = wx.FileDialog(self, "Select bag file to open", style=wx.FD_OPEN)
        if dlg.ShowModal() == wx.ID_OK:        
            bag_filename = os.path.join(dlg.GetPaths()[0])
            try:
                bag = rosbag.Bag(bag_filename, 'w')
                for msg in msgs:
                    bag.write(msg.topic, msg.msg)
            except Exception, e:
                self.displayError("Error saving messages to bag", str(e))        
            finally:
                bag.close();
            print "Saved %d msgs to '%s'" % (len(msgs), bag_filename) 
        dlg.Destroy()

    def onClear(self, event):
        dlg = wx.MessageDialog(self, message="Are you sure you want to clear all messages?", caption="Are you sure?", style=wx.OK | wx.CANCEL | wx.CENTRE | wx.NO_DEFAULT | wx.ICON_QUESTION)        
        if dlg.ShowModal() == wx.ID_OK:
            self._msg_recorder.clearMsgList()
            self.updateMsgListBox()
        dlg.Destroy()

    def onTimer(self, event):
        self.updateMsgListBox()

    def onFilterEnter(self, event):
        self._filter_string = self._filter_textctrl.GetValue()
        self.updateMsgListBox()
        
    def updateMsgListBox(self):
        """ Update _msg_listbox with _msg_list values and filter """
        try:
            filter_re = re.compile(self._filter_string, re.IGNORECASE)
        except Exception, e:
            self.displayError("Regular Expression Error", "Regular Expression Error : " + str(e))
            return
        msgs = self._msg_recorder.getMsgList()
        filter_msgs = []
        for msg in msgs:
            if filter_re.search(msg.description):
                filter_msgs.append(msg)
        if filter_msgs != self._filter_msg_list:
            #print "Filter list changed"
            # save last selection, so it can be restored when list changes
            selected_msgs = self.getMsgSelections()
            self._filter_msg_list = filter_msgs
            descriptions = [msg.description for msg in filter_msgs]
            self._msg_listbox.SetItems(descriptions)
            for index,msg in enumerate(filter_msgs):
                if msg in selected_msgs:
                    self._msg_listbox.Select(index)
            self._msg_listbox_label.SetLabel("Filtered Message List : Showing %d of %d messages" % (len(filter_msgs), len(msgs) ))

        
    def displayError(self, title, msg):
        wx.MessageBox(msg, title, wx.OK|wx.ICON_ERROR, self)

    def getMsgSelections(self):
        """ returns list of msgs that were selected in msg_listbox """
        msg_index_list = self._msg_listbox.GetSelections()
        msgs = [ self._filter_msg_list[index] for index in msg_index_list ]
        return msgs


    def onMakePlot(self, event):
        msgs = self.getMsgSelections()
        if len(msgs) == 0:
            self.displayError("Error", "No message selected")
            return

        for msg in msgs:                  
            data = MotorTraceAnalyzed(msg.msg)

            plot_names = [self._plot_listbox.GetString(index) for index in self._plot_listbox.GetSelections()]

            if len(plot_names) == 0:
                self.displayError("Error", "No plot types selected")
                return

            for plot_name in plot_names:
                if plot_name == 'Statistics':
                    self.make_stats_viewer(data)            
                else:
                    plot_fn = self._plot_functions[plot_name]
                    self.make_plot(data, [plot_fn])

 
    def make_stats_viewer(self, data):
        viewer = MtraceStatsPanel(self, data)
        viewer.SetSize(wx.Size(500, 700))
        viewer.Layout()
        viewer.Show(True)
        viewer.Raise()

    def make_plot(self, data, fns):
        for fn in fns:
            # Should have more plots here or something
            name, plot_data = fn(data)
            
            plotter = MtracePlotPanel(self, name, plot_data)
            plotter.SetSize(wx.Size(600, 600))
            plotter.Layout()
            plotter.Show(True)
            plotter.Raise()


        

class MtracePlotApp(wx.App):
    def __init__(self, msg_recorder):
        self._msg_recorder = msg_recorder
        wx.App.__init__(self, clearSigInt = False)

    def OnInit(self):
        self._frame = MtracePlotFrame(None, self._msg_recorder)
        self._frame.SetMinSize(self._frame.GetEffectiveMinSize())
        self._frame.Layout()
        self._frame.Show()

        return True

    
def main():
    try:
        msg_recorder = MtraceRecorder()
        
        useROS = False
        optlist,argv = getopt.gnu_getopt(sys.argv[1:], "hrf:");
        for opt,arg in optlist:
            if (opt == "-f"):
                msg_recorder.loadBagFileAsync(arg)
            elif (opt == "-r"):
                useROS = True
            elif (opt == "-h"):
                usage()
                return 0
            else :
                print "Internal error : opt = ", opt
                return 1

        if useROS:
            rospy.init_node('mtrace_plotter', anonymous = True, disable_signals=True)
            msg_recorder.startRosRecording()

        app = MtracePlotApp(msg_recorder)
        app.MainLoop()
    except wx.PyDeadObjectError:
        pass
    except KeyboardInterrupt:
        pass
    except:
        print 'Printing exc'
        import traceback
        traceback.print_exc()

    if useROS:
        print "Signalling normal shutdown"
        rospy.signal_shutdown("Shutdown");


if __name__ == '__main__':
    main()

