def buildSuccessString(msg):
   return "\033[0;32m" + str(msg) + "\033[0;0m"

def buildWarningString(msg):
   return "\033[0;33m" + str(msg) + "\033[0;0m"

def buildFailString(msg):
   return "\033[0;31m" + str(msg) + "\033[0;0m"

def printSuccess(msg):
   print "\033[0;32m" + str(msg) + "\033[0;0m"

def printFail(msg):
   print "\033[0;31m" + str(msg) + "\033[0;0m"
