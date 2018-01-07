"""
ThroughputTest

Tests the drop rate of communication with the golf cart
"""

from time import sleep
from CommsHandler import CommsHandler

PASS_PERC = 0.98

def throughput_test():
    """
    Run the test and measure the drop rate
    """
    CH = CommsHandler('/dev/ttyACM0', 115200, True)
    CH.run()
    sleep(3)

    run_fail = (CH.get_num_total_packets(), CH.get_num_failed_packets())
    print run_fail[0] + "Packets sent"
    print "\t" + 100 * run_fail[1]/run_fail[0] + "% Failed"

    return run_fail[1]/run_fail[0] > (1 - PASS_PERC)

def run_throughput_tests(test):
    """
    Runs all throughput tests
    """
    test.run_test("Throughput", throughput_test)
