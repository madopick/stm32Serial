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
from PyQt5.QtGui import QIntValidator
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
    byteReady = pyqtSignal(list)

    @pyqtSlot()
    def __init__(self):
        super(Worker, self).__init__()
        self.working = True
        self.byte_request = True

    def work(self):
        while self.working:
            if ser.isOpen():
                #byte operartion
                if self.byte_request == True:
                    line = ser.read(122)
                #ASCII operation
                else:
                    line = ser.readline()
            else:
                line = b''

            if line != b'':
                time.sleep(0.1)

                databyte = list(line)
                #print(databyte)

                if self.byte_request == True:
                    self.byteReady.emit(databyte)
                    self.byte_request = False

                else:
                    self.intReady.emit(line.decode('utf-8'))

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
        self.pb_cfg3_wr.clicked.connect(self.send_cfg3)
        self.pb_cfg1_rd.clicked.connect(self.read_cfg1)
        self.pb_cfg2_rd.clicked.connect(self.read_cfg2)
        self.pb_cfg3_rd.clicked.connect(self.read_cfg3)
        self.pushBtnClicked = False
        self.CopyFlag = 0
        self.ConnectStatus = 0
        self.Port = "UART"
        self.values = []
        self.group = 'NONE'
        self.getAll = 0

        onlyInt = QIntValidator()
        onlyInt.setRange(-2147483640, 2147483640)
        self.lineEdit_CF1_1.setValidator(onlyInt)
        self.lineEdit_CF1_2.setValidator(onlyInt)
        self.lineEdit_CF1_3.setValidator(onlyInt)
        self.lineEdit_CF1_4.setValidator(onlyInt)
        self.lineEdit_CF1_5.setValidator(onlyInt)
        self.lineEdit_CF1_6.setValidator(onlyInt)
        self.lineEdit_CF1_7.setValidator(onlyInt)
        self.lineEdit_CF1_8.setValidator(onlyInt)
        self.lineEdit_CF1_9.setValidator(onlyInt)
        self.lineEdit_CF1_10.setValidator(onlyInt)

        self.lineEdit_CF2_1.setValidator(onlyInt)
        self.lineEdit_CF2_2.setValidator(onlyInt)
        self.lineEdit_CF2_3.setValidator(onlyInt)
        self.lineEdit_CF2_4.setValidator(onlyInt)
        self.lineEdit_CF2_5.setValidator(onlyInt)
        self.lineEdit_CF2_6.setValidator(onlyInt)
        self.lineEdit_CF2_7.setValidator(onlyInt)
        self.lineEdit_CF2_8.setValidator(onlyInt)
        self.lineEdit_CF2_9.setValidator(onlyInt)
        self.lineEdit_CF2_10.setValidator(onlyInt)

        self.lineEdit_CF3_1.setValidator(onlyInt)
        self.lineEdit_CF3_2.setValidator(onlyInt)
        self.lineEdit_CF3_3.setValidator(onlyInt)
        self.lineEdit_CF3_4.setValidator(onlyInt)
        self.lineEdit_CF3_5.setValidator(onlyInt)
        self.lineEdit_CF3_6.setValidator(onlyInt)
        self.lineEdit_CF3_7.setValidator(onlyInt)
        self.lineEdit_CF3_8.setValidator(onlyInt)
        self.lineEdit_CF3_9.setValidator(onlyInt)
        self.lineEdit_CF3_10.setValidator(onlyInt)

        # global result
        print("Available ports:" + str(serial_ports()))
        for x in range(len(result)):
            name = result[x]
            self.cb_Port.addItem(name)
        self.cb_Port.addItem("USB")

        self.cb_baudrate.addItem("9600")
        self.cb_baudrate.addItem("38400")
        self.cb_baudrate.addItem("57600")
        self.cb_baudrate.addItem("115200")
        self.cb_baudrate.addItem("230400")
        self.cb_baudrate.setCurrentIndex(3)

        print('QT init')

    # WRITE CF1 array values
    def send_cfg1(self):
        print("send cfg1")
        if self.ConnectStatus == 0:
            return

        cfg1text = "{CF1:" + \
                   ('0' if (self.lineEdit_CF1_1.text()=='') else self.lineEdit_CF1_1.text()) + ',' + \
                   ('0' if (self.lineEdit_CF1_2.text()=='') else self.lineEdit_CF1_2.text()) + ',' + \
                   ('0' if (self.lineEdit_CF1_3.text()=='') else self.lineEdit_CF1_3.text()) + ',' + \
                   ('0' if (self.lineEdit_CF1_4.text()=='') else self.lineEdit_CF1_4.text()) + ',' + \
                   ('0' if (self.lineEdit_CF1_5.text()=='') else self.lineEdit_CF1_5.text()) + ',' + \
                   ('0' if (self.lineEdit_CF1_6.text()=='') else self.lineEdit_CF1_6.text()) + ',' + \
                   ('0' if (self.lineEdit_CF1_7.text()=='') else self.lineEdit_CF1_7.text()) + ',' + \
                   ('0' if (self.lineEdit_CF1_8.text()=='') else self.lineEdit_CF1_8.text()) + ',' + \
                   ('0' if (self.lineEdit_CF1_9.text()=='') else self.lineEdit_CF1_9.text()) + ',' + \
                   ('0' if (self.lineEdit_CF1_10.text()=='') else self.lineEdit_CF1_10.text()) + '}\r\n'

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
                   ('0' if (self.lineEdit_CF2_7.text()=='') else self.lineEdit_CF2_7.text()) + ',' + \
                   ('0' if (self.lineEdit_CF2_8.text()=='') else self.lineEdit_CF2_7.text()) + ',' + \
                   ('0' if (self.lineEdit_CF2_9.text()=='') else self.lineEdit_CF2_7.text()) + ',' + \
                   ('0' if (self.lineEdit_CF2_10.text()=='') else self.lineEdit_CF2_10.text()) + '}\r\n'

        blueColor = QColor(0, 0, 255)
        self.textEdit_3.setTextColor(blueColor)
        self.textEdit_3.append(str(cfg2text))
        blackColor = QColor(0, 0, 0)
        self.textEdit_3.setTextColor(blackColor)

        ser.write(cfg2text.encode())
        self.pushBtnClicked = True

    def send_cfg3(self):
        print("send cfg3")
        if self.ConnectStatus == 0:
            return

        cfg3text = "{CF3:" + \
                   ('0' if (self.lineEdit_CF3_1.text()=='') else self.lineEdit_CF3_1.text()) + ',' + \
                   ('0' if (self.lineEdit_CF3_2.text()=='') else self.lineEdit_CF3_2.text()) + ',' + \
                   ('0' if (self.lineEdit_CF3_3.text()=='') else self.lineEdit_CF3_3.text()) + ',' + \
                   ('0' if (self.lineEdit_CF3_4.text()=='') else self.lineEdit_CF3_4.text()) + ',' + \
                   ('0' if (self.lineEdit_CF3_5.text()=='') else self.lineEdit_CF3_5.text()) + ',' + \
                   ('0' if (self.lineEdit_CF3_6.text()=='') else self.lineEdit_CF3_6.text()) + ',' + \
                   ('0' if (self.lineEdit_CF3_7.text()=='') else self.lineEdit_CF3_7.text()) + ',' + \
                   ('0' if (self.lineEdit_CF3_8.text()=='') else self.lineEdit_CF3_8.text()) + ',' + \
                   ('0' if (self.lineEdit_CF3_9.text()=='') else self.lineEdit_CF3_9.text()) + ',' + \
                   ('0' if (self.lineEdit_CF3_10.text()=='') else self.lineEdit_CF3_10.text()) + '}\r\n'

        blueColor = QColor(0, 0, 255)
        self.textEdit_3.setTextColor(blueColor)
        self.textEdit_3.append(str(cfg3text))
        blackColor = QColor(0, 0, 0)
        self.textEdit_3.setTextColor(blackColor)

        ser.write(cfg3text.encode())
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

    def read_cfg3(self):
        print("read cfg3")
        if self.ConnectStatus == 0:
            return

        blueColor = QColor(0, 0, 255)
        self.textEdit_3.setTextColor(blueColor)
        cfg3text = "{RD3}\r\n"
        self.textEdit_3.append(str(cfg3text))
        blackColor = QColor(0, 0, 0)
        self.textEdit_3.setTextColor(blackColor)

        ser.write(cfg3text.encode())
        self.pushBtnClicked = True

    def loop_finished(self):
        print('Loop Finished')

    def clear_disconnect(self):
        self.lineEdit_CF1_1.clear()
        self.lineEdit_CF1_2.clear()
        self.lineEdit_CF1_3.clear()
        self.lineEdit_CF1_4.clear()
        self.lineEdit_CF1_5.clear()
        self.lineEdit_CF1_6.clear()
        self.lineEdit_CF1_7.clear()
        self.lineEdit_CF1_8.clear()
        self.lineEdit_CF1_9.clear()
        self.lineEdit_CF1_10.clear()

        self.lineEdit_CF2_1.clear()
        self.lineEdit_CF2_2.clear()
        self.lineEdit_CF2_3.clear()
        self.lineEdit_CF2_4.clear()
        self.lineEdit_CF2_5.clear()
        self.lineEdit_CF2_6.clear()
        self.lineEdit_CF2_7.clear()
        self.lineEdit_CF2_8.clear()
        self.lineEdit_CF2_9.clear()
        self.lineEdit_CF2_10.clear()

        self.lineEdit_CF3_1.clear()
        self.lineEdit_CF3_2.clear()
        self.lineEdit_CF3_3.clear()
        self.lineEdit_CF3_4.clear()
        self.lineEdit_CF3_5.clear()
        self.lineEdit_CF3_6.clear()
        self.lineEdit_CF3_7.clear()
        self.lineEdit_CF3_8.clear()
        self.lineEdit_CF3_9.clear()
        self.lineEdit_CF3_10.clear()

        redColor = QColor(255, 0, 0)
        self.textEdit_3.setTextColor(redColor)
        endText = "\r\n=============================== DISCONNECTED ===============================\r\n"
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
            mytext = "{RDA}\r\n"  # Get all config values
            self.getAll = 0

            if(self.cb_Port.currentText() == "USB"):
                print("USB HID Communication")
                self.Port = "USB"
            else:
                self.Port = "UART"

                ser = serial.Serial(self.cb_Port.currentText(), self.cb_baudrate.currentText(), timeout=1)
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

        else:
            redColor = QColor(255, 0, 0)
            self.textEdit_3.setTextColor(redColor)
            startText = "\r\n================================ CONNECTED ================================\r\n"
            self.textEdit_3.append(startText)

            blueColor = QColor(0, 0, 255)
            self.textEdit_3.setTextColor(blueColor)
            self.textEdit_3.append(mytext)
            blackColor = QColor(0, 0, 0)
            self.textEdit_3.setTextColor(blackColor)

        if (self.Port == "UART"):
            self.worker = Worker()
            self.thread = QThread()
            self.worker.moveToThread(self.thread)

            self.thread.started.connect(self.worker.work)

            self.worker.intReady.connect(self.onIntReady)
            self.worker.byteReady.connect(self.onByteReady)

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

        #for idx in result:
        #    print(idx)

        return result


    # Serial (in bytes) Receiving Packets
    def onByteReady(self, data):
        #print("onByteReady")
        #print(data)
        int_val = []

        if data[0] == 16 and data[1] == 17:
            print("byte header OK")

            for x in range(0,81):
                start = (x*4)+2
                end = start + 4
                res = int.from_bytes(data[start:end], "little")
                if res > 2147483647:
                     res = res - 4294967296
                int_val.append(res)

            #for x in range(0, 30):
            #    print(int_val[x])

            self.lineEdit_CF1_1.setText(str(int_val[0]))
            self.lineEdit_CF1_2.setText(str(int_val[1]))
            self.lineEdit_CF1_3.setText(str(int_val[2]))
            self.lineEdit_CF1_4.setText(str(int_val[3]))
            self.lineEdit_CF1_5.setText(str(int_val[4]))
            self.lineEdit_CF1_6.setText(str(int_val[5]))
            self.lineEdit_CF1_7.setText(str(int_val[6]))
            self.lineEdit_CF1_8.setText(str(int_val[7]))
            self.lineEdit_CF1_9.setText(str(int_val[8]))
            self.lineEdit_CF1_10.setText(str(int_val[9]))

            self.lineEdit_CF2_1.setText(str(int_val[10]))
            self.lineEdit_CF2_2.setText(str(int_val[11]))
            self.lineEdit_CF2_3.setText(str(int_val[12]))
            self.lineEdit_CF2_4.setText(str(int_val[13]))
            self.lineEdit_CF2_5.setText(str(int_val[14]))
            self.lineEdit_CF2_6.setText(str(int_val[15]))
            self.lineEdit_CF2_7.setText(str(int_val[16]))
            self.lineEdit_CF2_8.setText(str(int_val[17]))
            self.lineEdit_CF2_9.setText(str(int_val[18]))
            self.lineEdit_CF2_10.setText(str(int_val[19]))

            self.lineEdit_CF3_1.setText(str(int_val[20]))
            self.lineEdit_CF3_2.setText(str(int_val[21]))
            self.lineEdit_CF3_3.setText(str(int_val[22]))
            self.lineEdit_CF3_4.setText(str(int_val[23]))
            self.lineEdit_CF3_5.setText(str(int_val[24]))
            self.lineEdit_CF3_6.setText(str(int_val[25]))
            self.lineEdit_CF3_7.setText(str(int_val[26]))
            self.lineEdit_CF3_8.setText(str(int_val[27]))
            self.lineEdit_CF3_9.setText(str(int_val[28]))
            self.lineEdit_CF3_10.setText(str(int_val[29]))


    #Serial (in utf-8) Receiving Packets
    def onIntReady(self, i):
        print('SerialRead')

        if i != '':
            self.textEdit_3.append("{}".format(i))
            self.group = i.split(':')

            if len(self.group) == 2 :
                self.values = self.parseSerialMsg(i)

                if self.group[0] == '{CF1' and len(self.values) >= 10:
                    self.lineEdit_CF1_1.setText(self.values[0])
                    self.lineEdit_CF1_2.setText(self.values[1])
                    self.lineEdit_CF1_3.setText(self.values[2])
                    self.lineEdit_CF1_4.setText(self.values[3])
                    self.lineEdit_CF1_5.setText(self.values[4])
                    self.lineEdit_CF1_6.setText(self.values[5])
                    self.lineEdit_CF1_7.setText(self.values[6])
                    self.lineEdit_CF1_8.setText(self.values[7])
                    self.lineEdit_CF1_9.setText(self.values[8])
                    self.lineEdit_CF1_10.setText(self.values[9])

                    if self.getAll == 1:
                        blueColor = QColor(0, 0, 255)
                        self.textEdit_3.setTextColor(blueColor)
                        cfg2text = "{RD2}\r\n"
                        self.textEdit_3.append(str(cfg2text))
                        blackColor = QColor(0, 0, 0)
                        self.textEdit_3.setTextColor(blackColor)

                        ser.write(cfg2text.encode())
                        self.pushBtnClicked = True

                if self.group[0] == '{CF2' and len(self.values) >= 10:
                    self.lineEdit_CF2_1.setText(self.values[0])
                    self.lineEdit_CF2_2.setText(self.values[1])
                    self.lineEdit_CF2_3.setText(self.values[2])
                    self.lineEdit_CF2_4.setText(self.values[3])
                    self.lineEdit_CF2_5.setText(self.values[4])
                    self.lineEdit_CF2_6.setText(self.values[5])
                    self.lineEdit_CF2_7.setText(self.values[6])
                    self.lineEdit_CF2_8.setText(self.values[7])
                    self.lineEdit_CF2_9.setText(self.values[8])
                    self.lineEdit_CF2_10.setText(self.values[9])

                    if self.getAll == 1:
                        self.getAll = 0

                        blueColor = QColor(0, 0, 255)
                        self.textEdit_3.setTextColor(blueColor)
                        cfg3text = "{RD3}\r\n"
                        self.textEdit_3.append(str(cfg3text))
                        blackColor = QColor(0, 0, 0)
                        self.textEdit_3.setTextColor(blackColor)

                        ser.write(cfg3text.encode())
                        self.pushBtnClicked = True

                if self.group[0] == '{CF3' and len(self.values) >= 10:
                    self.lineEdit_CF3_1.setText(self.values[0])
                    self.lineEdit_CF3_2.setText(self.values[1])
                    self.lineEdit_CF3_3.setText(self.values[2])
                    self.lineEdit_CF3_4.setText(self.values[3])
                    self.lineEdit_CF3_5.setText(self.values[4])
                    self.lineEdit_CF3_6.setText(self.values[5])
                    self.lineEdit_CF3_7.setText(self.values[6])
                    self.lineEdit_CF3_8.setText(self.values[7])
                    self.lineEdit_CF3_9.setText(self.values[8])
                    self.lineEdit_CF3_10.setText(self.values[9])

    # Get CF1 Form Value in byte
    def getCF1values_inbytes(self):
        returnByte = bytearray()

        mytextInt = int(self.lineEdit_CF1_1.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF1_2.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF1_3.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF1_4.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF1_5.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF1_6.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF1_7.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF1_8.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF1_9.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF1_10.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        return returnByte

    # Get CF2 Form Value in byte
    def getCF2values_inbytes(self):
        returnByte = bytearray()

        mytextInt = int(self.lineEdit_CF2_1.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF2_2.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF2_3.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF2_4.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF2_5.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF2_6.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF2_7.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF2_8.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF2_9.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF2_10.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        return returnByte

    #Get CF3 Form Value in byte
    def getCF3values_inbytes(self):
        returnByte = bytearray()

        mytextInt = int(self.lineEdit_CF3_1.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF3_2.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF3_3.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF3_4.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF3_5.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF3_6.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF3_7.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF3_8.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF3_9.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        mytextInt = int(self.lineEdit_CF3_10.text())
        returnByte.extend(mytextInt.to_bytes(4, 'big', signed=True))

        return returnByte


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

    # READ CMD BUTTON
    def on_pushButton_4_clicked(self):
        print('Read All Value')

        if self.ConnectStatus == 0:
            return

        # Send data from serial port:
        if self.pushBtnClicked:
            self.pushBtnClicked = False
            return

        mytext = "{RDA}\r\n"
        ser.flushInput()

        blueColor = QColor(0, 0, 255)
        self.textEdit_3.setTextColor(blueColor)
        self.textEdit_3.append(mytext)
        blackColor = QColor(0, 0, 0)
        self.textEdit_3.setTextColor(blackColor)

        self.worker.byte_request = True
        ser.write(mytext.encode())
        self.pushBtnClicked = True


    #SEND CMD BUTTON
    def on_pushButton_3_clicked(self):
        print('Send to serial')

        if self.ConnectStatus == 0:
            return

        #Send data from serial port:
        if self.pushBtnClicked:
            self.pushBtnClicked = False
            return

        #if text edit is empty then send write command to update CF3 using byte methods
        if self.textEdit_2.toPlainText() == '':
            msgBox = QMessageBox()
            msgBox.setWindowTitle("Warning!!!")
            msgBox.setIcon(QMessageBox.Warning)
            msgBox.setText("All Config Value are over written")
            msgBox.exec()

            bufferByte = bytearray("{CFA:", 'utf-8')
            bufferByte.extend(self.getCF1values_inbytes())
            bufferByte.extend(self.getCF2values_inbytes())
            bufferByte.extend(self.getCF3values_inbytes())

            footer = bytearray("{\r\n", 'utf-8')
            bufferByte.extend(footer)
            print(bufferByte)

            print(len(bufferByte))
            ser.write(bufferByte)

        #send input command by user in text edit 2
        #Available command are: {MSG:, {CF1:, {CF2:, {CF3:, {CFA:, {RD1}, {RD2}, {RD3}, {RD4}, {RDA}
        else:
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


