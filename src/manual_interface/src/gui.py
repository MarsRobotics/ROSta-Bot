# My first PyQt Gui
import sys
from PyQt4 import QtGui

#pass in base class for all UI objects, window
class GUI(QtGui.QWidget):
    def __init__(self):
        #returns parent object
        super(GUI, self).__init__()

        self.initUI()

    def initUI(self):
        #set font
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        #set window width
        w = 550
        h = 450

        #kill button
        btn = QtGui.QPushButton('KILL', self)
        btn.setToolTip('<b>Kill Power</b>')
        #resize button with recommended size
        btn.resize(btn.sizeHint())
        btn.move(w-150, h-60)

        #forward button
        btnFor = QtGui.QPushButton('Forward', self)
        btnFor.setToolTip('<b>Move forward</b>')
        #resize button with recommended size
        btnFor.resize(btn.sizeHint())
        btnFor.move(w-150, h-190)

        #backward button
        btnBack = QtGui.QPushButton('Backward', self)
        btnBack.setToolTip('<b>Move backward</b>')
        #resize button with recommended size
        btnBack.resize(btn.sizeHint())
        btnBack.move(w-150, h-120)

        #right turn button
        btnRgt = QtGui.QPushButton('Right', self)
        btnRgt.setToolTip('<b>Move right</b>')
        #resize button with recommended size
        btnRgt.resize(btn.sizeHint())
        btnRgt.move(w-100, h-155)

        #left turn button
        btnLft = QtGui.QPushButton('Left', self)
        btnLft.setToolTip('<b>Move left</b>')
        #resize button with recommended size
        btnLft.resize(btn.sizeHint())
        btnLft.move(w-200, h-155)

        #Textbox
        lblMsg = QtGui.QLabel('Message', self)
        lblMsg.move(w-300, h-80)
        message = QtGui.QLineEdit(self)
        message.move(w-300, h-50)

        #ROS topic
        lblROStop = QtGui.QLabel('ROS Topic', self)
        lblROStop.move(50, h-80)
        combo = QtGui.QComboBox(self)
        combo.addItem("Meow")
        combo.addItem("Chickens")
        combo.addItem("Rabbits")
        combo.addItem("Chinchillas")
        combo.addItem("Puppies")
        combo.move(50, h-50)

        #Numeric Data
        lblNum = QtGui.QLabel('Numeric Data', self)
        lblNum.move(w-200, 150)

        #Sensor Data
        lblSenData = QtGui.QLabel('Sensor Data', self)
        lblSenData.move(w-200, 20)

        #chick pic
        hbox = QtGui.QHBoxLayout(self)
        pixmap = QtGui.QPixmap("baby chick.png")
        chick = QtGui.QLabel(self)
        chick.setPixmap(pixmap)
        hbox.addWidget(chick)
        self.setLayout(hbox)

        #locates window on screen and sets size
        self.setGeometry(300, 300, w, h)
        self.setWindowTitle('Hello Meow')
        self.show()

def main():
    app = QtGui.QApplication(sys.argv)
    goo = GUI()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
