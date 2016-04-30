#!/usr/bin/python
import sys
from PyQt4 import QtGui

class Display(QtGui.QWidget):
   def __init__(self):
      super(Display, self).__init__()
      self.initUI()

   def initUI(self):
      grid = QtGui.QGridLayout()
      self.setLayout(grid)
      grid.setSpacing(10);

      speed = QtGui.QLabel('10')
      grid.addWidget(speed, 1,0)

      self.setGeometry(300,300, 350, 350)
      self.setWindowTitle('GUI')
      self.show()


          
def main():
    app = QtGui.QApplication(sys.argv)
    display = Display()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
