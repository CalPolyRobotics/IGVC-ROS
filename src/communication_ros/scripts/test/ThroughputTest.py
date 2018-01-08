"""
ThroughputTest

Tests the drop rate of communication with the golf cart

In order to run this test:
roscore must be running
BoardComms node must be running with a valid connection
"""

from time import sleep
from CommsHandler import CommsHandler

PASS_PERC = 0.98

def throughput_test():
    """
    Run the test and measure the drop rate
    """

    """
    Run multiple Echo packets
    Check response is equal to request
    Print out total data sent, total messages sent, %failure data, %failure mesages
    """

    echo = rospy.Publisher("Echo", UInt8MultiArray, queue_size=1000)
    for i in range(1000):
        echo.

def run_throughput_tests(test):
    """
    Runs all throughput tests
    """
    test.run_test("Throughput", throughput_test)
