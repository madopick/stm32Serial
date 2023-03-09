# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'pyqt.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


import sys, os, serial, serial.tools.list_ports, warnings
from PyQt5.QtCore import *
from PyQt5.QtGui import QColor
import time
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
import serial.tools.list_ports as port_list


result = []
portindex = 0

def serial_ports():
    global portindex

    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass

    ports = list(port_list.comports())
    for p in ports:
        if "STLink" in str(p):
            break
        else:
            portindex += 1
    return result


class Worker(QObject):
    finished = pyqtSignal()
    intReady = pyqtSignal(str)

    @pyqtSlot()
    def __init__(self):
        super(Worker, self).__init__()
        self.working = True

    def work(self):
        while self.working:
            if ser.isOpen():
                line = ser.readline().decode('utf-8')
            else:
                line = ''

            if line != '':
                time.sleep(0.1)
                self.intReady.emit(line)

        self.finished.emit()


class qt(QMainWindow):

    def __init__(self):

        QMainWindow.__init__(self)
        loadUi('qt.ui', self)

        self.thread = None
        self.worker = None
        self.pushButton.clicked.connect(self.start_loop)
        self.pb_cfg1_wr.clicked.connect(self.send_cfg1)
        self.pb_cfg2_wr.clicked.connect(self.send_cfg2)
        self.pb_cfg1_rd.clicked.connect(self.read_cfg1)
        self.pb_cfg2_rd.clicked.connect(self.read_cfg2)
        self.pushBtnClicked = False
        self.CopyFlag = 0
        self.ConnectStatus = 0
        self.Port = "UART"
        self.values = []
        self.group = 'NONE'
        self.getAll = 0

        # global result
        print("Available ports:" + str(serial_ports()))
        for x in range(len(result)):
            name = result[x]
            self.cb_Port.addItem(name)
        self.cb_Port.addItem("USB")
        print('QT init')

    # WRITE CF1 array values
    def send_cfg1(self):
        print("send cfg1")
        if self.ConnectStatus == 0:
            return

        cfg1text = "{CF1:" + \
                   ('0' if (self.lineEdit_1.text()=='') else self.lineEdit_1.text()) + ',' + \
                   ('0' if (self.lineEdit_2.text()=='') else self.lineEdit_2.text()) + ',' + \
                   ('0' if (self.lineEdit_3.text()=='') else self.lineEdit_3.text()) + ',' + \
                   ('0' if (self.lineEdit_4.text()=='') else self.lineEdit_4.text()) + ',' + \
                   ('0' if (self.lineEdit_5.text()=='') else self.lineEdit_5.text()) + ',' + \
                   ('0' if (self.lineEdit_6.text()=='') else self.lineEdit_6.text()) + ',' + \
                   ('0' if (self.lineEdit_7.text()=='') else self.lineEdit_7.text()) + '}\r\n'

        blueColor = QColor(0, 0, 255)
        self.textEdit_3.setTextColor(blueColor)
        self.textEdit_3.append(str(cfg1text))
        blackColor = QColor(0, 0, 0)
        self.textEdit_3.setTextColor(blackColor)

        ser.write(cfg1text.encode())
        self.pushBtnClicked = True

    # WRITE CF2 array values
    def send_cfg2(self):
        print("send cfg2")
        if self.ConnectStatus == 0:
            return

        cfg2text = "{CF2:" + \
                   ('0' if (self.lineEdit_CF2_1.text()=='') else self.lineEdit_CF2_1.text()) + ',' + \
                   ('0' if (self.lineEdit_CF2_2.text()=='') else self.lineEdit_CF2_2.text()) + ',' + \
                   ('0' if (self.lineEdit_CF2_3.text()=='') else self.lineEdit_CF2_3.text()) + ',' + \
                   ('0' if (self.lineEdit_CF2_4.text()=='') else self.lineEdit_CF2_4.text()) + ',' + \
                   ('0' if (self.lineEdit_CF2_5.text()=='') else self.lineEdit_CF2_5.text()) + ',' + \
                   ('0' if (self.lineEdit_CF2_6.text()=='') else self.lineEdit_CF2_6.text()) + ',' + \
                   ('0' if (self.lineEdit_CF2_7.text()=='') else self.lineEdit_CF2_7.text()) + '}\r\n'

        blueColor = QColor(0, 0, 255)
        self.textEdit_3.setTextColor(blueColor)
        self.textEdit_3.append(str(cfg2text))
        blackColor = QColor(0, 0, 0)
        self.textEdit_3.setTextColor(blackColor)

        ser.write(cfg2text.encode())
        self.pushBtnClicked = True

    #READ CF1 array values
    def read_cfg1(self):
        print("read cfg1")
        if self.ConnectStatus == 0:
            return

        blueColor = QColor(0, 0, 255)
        self.textEdit_3.setTextColor(blueColor)
        cfg1text = "{RD1}\r\n"
        self.textEdit_3.append(str(cfg1text))
        blackColor = QColor(0, 0, 0)
        self.textEdit_3.setTextColor(blackColor)

        ser.write(cfg1text.encode())
        self.pushBtnClicked = True

    # READ CF2 array values
    def read_cfg2(self):
        print("read cfg2")
        if self.ConnectStatus == 0:
            return

        blueColor = QColor(0, 0, 255)
        self.textEdit_3.setTextColor(blueColor)
        cfg2text = "{RD2}\r\n"
        self.textEdit_3.append(str(cfg2text))
        blackColor = QColor(0, 0, 0)
        self.textEdit_3.setTextColor(blackColor)

        ser.write(cfg2text.encode())
        self.pushBtnClicked = True

    def loop_finished(self):
        print('Loop Finished')

    def clear_disconnect(self):
        self.lineEdit_1.clear()
        self.lineEdit_2.clear()
        self.lineEdit_3.clear()
        self.lineEdit_4.clear()
        self.lineEdit_5.clear()
        self.lineEdit_6.clear()
        self.lineEdit_7.clear()

        self.lineEdit_CF2_1.clear()
        self.lineEdit_CF2_2.clear()
        self.lineEdit_CF2_3.clear()
        self.lineEdit_CF2_4.clear()
        self.lineEdit_CF2_5.clear()
        self.lineEdit_CF2_6.clear()
        self.lineEdit_CF2_7.clear()

        redColor = QColor(255, 0, 0)
        self.textEdit_3.setTextColor(redColor)
        endText = "\r\n==================================== DISCONNECTED ====================================\r\n"
        self.textEdit_3.append(endText)

        blackColor = QColor(0, 0, 0)
        self.textEdit_3.setTextColor(blackColor)


    def stop_loop(self):
        print('stop loop')
        self.worker.working = False
        self.label_5.setText("Not Connected")
        self.label_5.setStyleSheet('color: red')
        ser.close()


    def start_loop(self):
        print('start loop')
        global ser

        #Disconnect
        if(self.ConnectStatus == 1):
            self.ConnectStatus = 0
            self.clear_disconnect()
            self.pushButton.setText("CONNECT")
            self.label_5.setText("Not Connected")
            self.label_5.setStyleSheet('color: red')
            self.stop_loop()
            return

        #Verify the correct COM Port
        try:
            mytext = "{RD1}\r\n"  # Send first enter
            self.getAll = 1

            if(self.cb_Port.currentText() == "USB"):
                print("USB HID Communication")
                self.Port = "USB"
            else:
                self.Port = "UART"

                blueColor = QColor(0, 0, 255)
                self.textEdit_3.setTextColor(blueColor)
                self.textEdit_3.append(mytext)
                blackColor = QColor(0, 0, 0)
                self.textEdit_3.setTextColor(blackColor)

                ser = serial.Serial(self.cb_Port.currentText(), 115200, timeout=1)
                ser.write(mytext.encode())

            if(self.ConnectStatus == 0):
                self.ConnectStatus = 1
                self.pushButton.setText("DISCONNECT")
                self.label_5.setText("Connected Via " + self.Port)
                self.label_5.setStyleSheet('color: black')

        except:
            msgBox = QMessageBox()
            msgBox.setWindowTitle("COM Port Error!")
            msgBox.setIcon(QMessageBox.Warning)
            msgBox.setText("Selected COM port does not exist. Please verify the COM port Number.")
            msgBox.exec()
            self.label_5.setText("Not Connected")
            self.label_5.setStyleSheet('color: red')
            print('start loop')
            return

        if (self.Port == "UART"):
            self.worker = Worker()
            self.thread = QThread()
            self.worker.moveToThread(self.thread)

            self.thread.started.connect(self.worker.work)

            self.worker.intReady.connect(self.onIntReady)

            self.worker.finished.connect(self.loop_finished)
            self.worker.finished.connect(self.thread.quit)
            self.worker.finished.connect(self.worker.deleteLater)
            self.thread.finished.connect(self.thread.deleteLater)
            self.thread.start()

        else:
            print("USB Thread here")

    #Parse incoming uart msg from STM32
    def parseSerialMsg(self, str):
        # print(str)

        # remove leading and trailing edge spaces of each item of the list
        #result = [value.strip() for value in str.split(':')[1].split(',')]
        result = [value.strip() for value in str.split(':')[1].replace('}','').split(',')]

        for idx in result:
            print(idx)

        return result

    #Serial Receiving Packets
    def onIntReady(self, i):
        print('SerialRead')

        if i != '':
            self.textEdit_3.append("{}".format(i))
            self.group = i.split(':')

            if len(self.group) == 2 :
                self.values = self.parseSerialMsg(i)

                if self.group[0] == '{CF1':
                    self.lineEdit_1.setText(self.values[0])
                    self.lineEdit_2.setText(self.values[1])
                    self.lineEdit_3.setText(self.values[2])
                    self.lineEdit_4.setText(self.values[3])
                    self.lineEdit_5.setText(self.values[4])
                    self.lineEdit_6.setText(self.values[5])
                    self.lineEdit_7.setText(self.values[6])

                    if self.getAll == 1:
                        self.getAll = 0

                        blueColor = QColor(0, 0, 255)
                        self.textEdit_3.setTextColor(blueColor)
                        cfg2text = "{RD2}\r\n"
                        self.textEdit_3.append(str(cfg2text))
                        blackColor = QColor(0, 0, 0)
                        self.textEdit_3.setTextColor(blackColor)

                        ser.write(cfg2text.encode())
                        self.pushBtnClicked = True

                if self.group[0] == '{CF2':
                    self.lineEdit_CF2_1.setText(self.values[0])
                    self.lineEdit_CF2_2.setText(self.values[1])
                    self.lineEdit_CF2_3.setText(self.values[2])
                    self.lineEdit_CF2_4.setText(self.values[3])
                    self.lineEdit_CF2_5.setText(self.values[4])
                    self.lineEdit_CF2_6.setText(self.values[5])
                    self.lineEdit_CF2_7.setText(self.values[6])


    # TXT Save
    def on_pushButton_5_clicked(self):
        if self.pushBtnClicked:
            self.pushBtnClicked = False
            return

        fileName = QFileDialog.getSaveFileName(self, 'Select location to Log', "", '*.txt')
        if not fileName:
            return

        with open(fileName[0], 'w') as f:
            my_text = self.textEdit_3.toPlainText()
            f.write(my_text)

        self.textEdit_3.append("Log Saved in :" + fileName[0] + "\r\n")
        self.pushBtnClicked = True

    def on_pushButton_2_clicked(self):
        self.textEdit.setText('Stopped! Please click CONNECT...')

    def on_pb_Clr_clicked(self):
        if self.pushBtnClicked:
            self.pushBtnClicked = False
            return

        self.textEdit_3.setText('')

    #Clear UART console history
    def on_pb_Clr_clicked(self):
        if self.pushBtnClicked:
            self.pushBtnClicked = False
            return

        self.textEdit_3.setText('')

    #SEND CMD BUTTON
    def on_pushButton_3_clicked(self):
        print('Send to serial')

        #Send data from serial port:
        if self.pushBtnClicked:
            self.pushBtnClicked = False
            return

        mytext = self.textEdit_2.toPlainText() + "\r\n"

        blueColor = QColor(0, 0, 255)
        self.textEdit_3.setTextColor(blueColor)
        self.textEdit_3.append(mytext)
        blackColor = QColor(0, 0, 0)
        self.textEdit_3.setTextColor(blackColor)

        ser.write(mytext.encode())
        self.pushBtnClicked = True

def run():
    app = QApplication(sys.argv)
    widget = qt()
    widget.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    run()


