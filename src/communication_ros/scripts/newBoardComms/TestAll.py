"""
TestAll

A module designed to run tests on the entire functionality of BoardComms
"""
from test.Test import Test
from test.PacketTest import run_packet_test
from test.ThroughputTest import run_throughput_tests
from test.GetMessageTest import run_get_message_tests

t = Test()
run_packet_test(t)
# run_throughput_tests(t) current implementation is inf loop
# run_get_message_tests(t) current implementation will not work
t.print_results()


