# My first PyQt Gui
import sys
from PyQt4 import QtGui

#pass in base class for all UI objects, window
class Button(QtGui.QWidget):
    def __init__(self):
        #returns parent object
        super(Button, self).__init__()

        self.initUI()

    def initUI(self):
        #set font
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))

        #create tooltip
        self.setToolTip('This is a <b>QWidget</b> widget')

        #create button widget and set tooltip
        btn = QtGui.QPushButton('Button', self)
        btn.setToolTip('This is a <b>QPushButton</b> widget')
        #resize button with recommended size
        btn.resize(btn.sizeHint())
        btn.move(50, 50)

        #locates window on screen and sets size
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Hello World')
        self.setWindowIcon(QtGui.QIcon('web.png'))
        self.show()

def main():
    app = QtGui.QApplication(sys.argv)
    butt = Button()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
