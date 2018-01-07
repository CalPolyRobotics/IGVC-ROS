"""
Test Suite

This module contains functions developed to ease in the process of developing tests
"""

def print_success(msg):
    """
    Print msg in green
    """
    print '\033[0;32m' + str(msg) + '\033[0;0m'

def print_warning(msg):
    """
    Print msg in yellow
    """
    print '\033[0;33m' + str(msg) + '\033[0;0m'

def print_fail(msg):
    """
    Print msg in red
    """
    print '\033[0;31m' + str(msg) + '\033[0;0m'

class Test(object):
    """
    Test

    An object to run tests, and print diagnostics
    """
    def __init__(self):
        """
        Init the Test object to keep track of tests
        """
        self.tests_run = 0
        self.tests_failed = 0

    def run_test(self, name, func):
        """
        Run test and print results
        """
        self.tests_run = self.tests_run + 1

        print 'Running ' + name + ' Test\n\t',
        if func():
            print_success('Pass')
        else:
            self.tests_failed = self.tests_failed + 1
            print_fail('Fail')

    def print_results(self):
        """
        Print the results from all of the tests
        """
        print '{} Tests Run\n\t{}% Failed'.format(self.tests_run, self.tests_failed/self.tests_run)
