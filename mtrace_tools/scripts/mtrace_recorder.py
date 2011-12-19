from __future__ import with_statement

PKG = 'mtrace_tools'
import roslib; roslib.load_manifest(PKG)
import rospy
from ethercat_hardware.msg import MotorTrace
import rosbag
import threading
import time
import copy

class MotorTraceTopicException(Exception): pass

def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise MotorTraceTopicException("remote call failed: %s"%msg)
    return val


##\brief Gets list of published motor trace topics
def get_mtrace_topics(type = 'ethercat_hardware/MotorTrace'):
    master = roslib.scriptutil.get_master()
    # Need try/catch here
    try:
        pub_topics = _succeed(master.getPublishedTopics('/mtrace_plotter', '/'))
    except MotorTraceTopicException:
        rospy.logerr('Unable to get list of motor trace topics')
        return []

    valid_topics = []
    for topic,topic_type in pub_topics:
        if topic_type == type:
            valid_topics.append(topic)

    return valid_topics



class MotorTraceDesc:
    """ Simple wrapper class around motor trace messages that generates descriptions for each message"""
    def __init__(self, topic, msg):
        self.topic = topic
        self.msg = msg
        t = time.strftime("%a, %b %d, %I:%M %p", time.localtime(msg.header.stamp.to_sec()))
        self.description = "%s : %s : %s" % (t, msg.actuator_info.name, msg.reason)    


class MtraceCallback:
    """ Wrapper class around callback so topic can be provided"""
    def __init__(self, callback, topic):
        self.topic = topic
        self.callback = callback
        
    def mtraceCallback(self, msg):
        self.callback(msg,self.topic)
    

class MtraceRecorder:
    """ Class to record and keep trace of MotorTrace messages"""
    def __init__(self):
        self._mutex = threading.Lock()
        self._msg_list = []
        self._topic_subs = {}
        self._ros_topic_thread = None

    def getMsgList(self):
        with self._mutex:
            msg_list = copy.copy(self._msg_list)
        return msg_list

    def clearMsgList(self):
        with self._mutex:
            self._msg_list = []

    def loadBagFileInternal(self, bag):
        """ Internal, loads all MotorTrace messages from a given bag file """
        for topic, msg, t in bag.read_messages():
            # use some simple heristic to determine if message is true MotorTrace message
            if hasattr(msg, 'reason') and hasattr(msg, 'board_info') and hasattr(msg, 'samples'):
                with self._mutex:
                    self._msg_list.append(MotorTraceDesc(topic,msg))
                #rospy.sleep(0.5)

    def loadBagFile(self, bag_filename):
        """ Loads all MotorTrace messages from a given bag file """
        bag = rosbag.Bag(bag_filename)
        self.loadBagFileInternal(self, bag)

    def loadBagFileAsync(self, bag_filename):
        """ Starts background thread that loads all MotorTrace messages from a given bag file """
        # open bag file before starting thread so this easily pass onerrors that occurred while opening file
        bag = rosbag.Bag(bag_filename)
        t = threading.Thread(group=None, target=self.loadBagFileInternal, args=[bag])
        t.start()
        

    def mtraceCallback(self, msg, topic):
        desc_msg = MotorTraceDesc(topic, msg)
        with self._mutex:
            self._msg_list.append(desc_msg)


    def rosTopicThread(self):
        while not rospy.is_shutdown():
            # gets list of all topic with MotorTrace type
            topics = get_mtrace_topics()
            with self._mutex:                
                self_topics = self._topic_subs
            # find any new topics
            new_topics = []
            for topic in topics:
                if topic not in self_topics:
                    print "New MotorTrace topic", topic
                    new_topics.append(topic)
            # subscribe to new topics
            new_topic_subs = []
            for topic in new_topics:
                callbackWrapper = MtraceCallback(self.mtraceCallback, topic)
                sub = rospy.Subscriber(topic, MotorTrace, callbackWrapper.mtraceCallback)
                new_topic_subs.append( (topic, sub) )
            # add new subscribers to internal map 
            with self._mutex:
                for topic,sub in new_topic_subs:
                    self._topic_subs[topic] = sub
            # wait before doing this again
            rospy.sleep(1.0)


    def startRosRecording(self):
        """ Starts recoarding messages from any ROS topic that has a MotorTrace type
        scans for new ROS MotorModel topics every second and subscribes to every new one
        """
        if self._ros_topic_thread is not None:
            raise Exception("Already monitoring ROS topics")
        
        self._ros_topic_thread = threading.Thread(group=None, target=self.rosTopicThread)
        self._ros_topic_thread.start()


