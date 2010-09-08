#!/usr/bin/env python

# Sample code to check test data API

from __future__ import division

import roslib; roslib.load_manifest('wg_invent_client')

import sys, getpass

from wg_invent_client import Invent, TestData

from wg_invent_client.wgtest_client import *

import matplotlib
import matplotlib.pyplot as plt

def history_to_series(history):
    stamps = []
    vals = []
    mins = []
    maxs = []

    for k, v in history.iteritems():
        stamps.append(v[3])
        vals.append(v[2])
        mins.append(v[0])
        maxs.append(v[1])

    return stamps, vals, mins, maxs

def plot_ts(stamps, vals, mins, maxs):
    fig = plt.figure()
    plt.xlabel('Time (s)')
    plt.ylabel('Effort')
    
    plt.plot(stamps, vals, 'r.')
    plt.plot(stamps, mins, color='b')
    plt.plot(stamps, maxs, color='b')

    plt.show()

def percent_passed(results):
    count = 0
    passed = 0
    for k, v in results.iteritems():
        if v[0].lower() == 'pass':
            passed += 1
        count += 1

    return passed / count

if __name__ == '__main__':
    iv = Invent('watts', 'willow', True)
    
    if not iv.login():
        print >> sys.stderr, "Unable to login to Invent. Try again."
        sys.exit(1)

    print 'ST Info:', get_subtest_info(iv, '2')

    print 'Version Info:', get_version_info(iv, '30')

    sys.exit()

#    print get_test_runs('680420401009', iv)
    print 'Test ID', get_testid_by_name(iv, 'caster-post')
    print ''
    print ''
    print 'Test Info', get_test_info(iv, '33')

    print get_run_info(iv, 20)
    print ''
    print ''
    print ''
    print get_log_data(iv, 76)
    print ''
    print ''
    print ''
    print get_test_logs(iv, '680410801010')
    print ''
    print ''
    print ''
    print get_run_measurements(iv, 20)
    print ''
    print ''
    print ''
    results =  get_subtest_results(iv, 2)
    print percent_passed(results)
    print ''
    print ''
    print ''
    print get_version_results(iv, 2)

    print ''
    print ''
    print ''
    print get_version_info(iv, 2)
    print ''
    print ''
    print ''
    print get_version_params(iv, 2)
    print ''
    print ''
    print ''
    print get_subtest_versions(iv, 1)

    print ''
    print ''
    print ''

    print is_item_tested(iv, '680410801010')

    print get_test_log_sequence(iv, '680410801010')

    print get_reqd_sequence(iv, '4108')

    print ''
    print ''
    print ''
    print ''
    if False:
        history =  get_measurement_history(iv, 'Effort', 2)
        
        s, v, m, M = history_to_series(history)
        plot_ts(s, v, m, M)


