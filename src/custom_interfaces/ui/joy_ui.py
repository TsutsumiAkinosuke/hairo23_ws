# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'joy.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.joy_left_xline = QtWidgets.QFrame(self.centralwidget)
        self.joy_left_xline.setGeometry(QtCore.QRect(230, 480, 140, 3))
        self.joy_left_xline.setFrameShape(QtWidgets.QFrame.HLine)
        self.joy_left_xline.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.joy_left_xline.setObjectName("joy_left_xline")
        self.joy_left_yline = QtWidgets.QFrame(self.centralwidget)
        self.joy_left_yline.setGeometry(QtCore.QRect(300, 410, 3, 140))
        self.joy_left_yline.setFrameShape(QtWidgets.QFrame.VLine)
        self.joy_left_yline.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.joy_left_yline.setObjectName("joy_left_yline")
        self.joy_left = QtWidgets.QDial(self.centralwidget)
        self.joy_left.setGeometry(QtCore.QRect(250, 430, 100, 100))
        self.joy_left.setWrapping(True)
        self.joy_left.setObjectName("joy_left")
        self.up_button = QtWidgets.QPushButton(self.centralwidget)
        self.up_button.setGeometry(QtCore.QRect(100, 370, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.up_button.setFont(font)
        self.up_button.setAutoDefault(False)
        self.up_button.setDefault(False)
        self.up_button.setFlat(False)
        self.up_button.setObjectName("up_button")
        self.joy_right_yline = QtWidgets.QFrame(self.centralwidget)
        self.joy_right_yline.setGeometry(QtCore.QRect(500, 410, 3, 140))
        self.joy_right_yline.setFrameShape(QtWidgets.QFrame.VLine)
        self.joy_right_yline.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.joy_right_yline.setObjectName("joy_right_yline")
        self.joy_right_xline = QtWidgets.QFrame(self.centralwidget)
        self.joy_right_xline.setGeometry(QtCore.QRect(430, 480, 140, 3))
        self.joy_right_xline.setFrameShape(QtWidgets.QFrame.HLine)
        self.joy_right_xline.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.joy_right_xline.setObjectName("joy_right_xline")
        self.joy_right = QtWidgets.QDial(self.centralwidget)
        self.joy_right.setGeometry(QtCore.QRect(450, 430, 100, 100))
        self.joy_right.setWrapping(True)
        self.joy_right.setObjectName("joy_right")
        self.down_button = QtWidgets.QPushButton(self.centralwidget)
        self.down_button.setGeometry(QtCore.QRect(100, 490, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.down_button.setFont(font)
        self.down_button.setAutoDefault(False)
        self.down_button.setDefault(False)
        self.down_button.setFlat(False)
        self.down_button.setObjectName("down_button")
        self.right_button = QtWidgets.QPushButton(self.centralwidget)
        self.right_button.setGeometry(QtCore.QRect(160, 430, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.right_button.setFont(font)
        self.right_button.setAutoDefault(False)
        self.right_button.setDefault(False)
        self.right_button.setFlat(False)
        self.right_button.setObjectName("right_button")
        self.left_button = QtWidgets.QPushButton(self.centralwidget)
        self.left_button.setGeometry(QtCore.QRect(40, 430, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.left_button.setFont(font)
        self.left_button.setAutoDefault(False)
        self.left_button.setDefault(False)
        self.left_button.setFlat(False)
        self.left_button.setObjectName("left_button")
        self.circle_button = QtWidgets.QPushButton(self.centralwidget)
        self.circle_button.setGeometry(QtCore.QRect(710, 430, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.circle_button.setFont(font)
        self.circle_button.setObjectName("circle_button")
        self.triangle_button = QtWidgets.QPushButton(self.centralwidget)
        self.triangle_button.setGeometry(QtCore.QRect(650, 370, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.triangle_button.setFont(font)
        self.triangle_button.setObjectName("triangle_button")
        self.cross_button = QtWidgets.QPushButton(self.centralwidget)
        self.cross_button.setGeometry(QtCore.QRect(650, 490, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.cross_button.setFont(font)
        self.cross_button.setObjectName("cross_button")
        self.square_button = QtWidgets.QPushButton(self.centralwidget)
        self.square_button.setGeometry(QtCore.QRect(590, 430, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.square_button.setFont(font)
        self.square_button.setObjectName("square_button")
        self.joy_left_xlabel = QtWidgets.QLabel(self.centralwidget)
        self.joy_left_xlabel.setGeometry(QtCore.QRect(260, 360, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.joy_left_xlabel.setFont(font)
        self.joy_left_xlabel.setAlignment(QtCore.Qt.AlignCenter)
        self.joy_left_xlabel.setObjectName("joy_left_xlabel")
        self.joy_left_ylabel = QtWidgets.QLabel(self.centralwidget)
        self.joy_left_ylabel.setGeometry(QtCore.QRect(260, 380, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.joy_left_ylabel.setFont(font)
        self.joy_left_ylabel.setAlignment(QtCore.Qt.AlignCenter)
        self.joy_left_ylabel.setObjectName("joy_left_ylabel")
        self.joy_right_xlabel = QtWidgets.QLabel(self.centralwidget)
        self.joy_right_xlabel.setGeometry(QtCore.QRect(460, 360, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.joy_right_xlabel.setFont(font)
        self.joy_right_xlabel.setAlignment(QtCore.Qt.AlignCenter)
        self.joy_right_xlabel.setObjectName("joy_right_xlabel")
        self.joy_right_ylabel = QtWidgets.QLabel(self.centralwidget)
        self.joy_right_ylabel.setGeometry(QtCore.QRect(460, 380, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.joy_right_ylabel.setFont(font)
        self.joy_right_ylabel.setAlignment(QtCore.Qt.AlignCenter)
        self.joy_right_ylabel.setObjectName("joy_right_ylabel")
        self.l1_button = QtWidgets.QPushButton(self.centralwidget)
        self.l1_button.setGeometry(QtCore.QRect(90, 320, 71, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.l1_button.setFont(font)
        self.l1_button.setAutoDefault(False)
        self.l1_button.setDefault(False)
        self.l1_button.setFlat(False)
        self.l1_button.setObjectName("l1_button")
        self.r1_button = QtWidgets.QPushButton(self.centralwidget)
        self.r1_button.setGeometry(QtCore.QRect(640, 320, 71, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.r1_button.setFont(font)
        self.r1_button.setAutoDefault(False)
        self.r1_button.setDefault(False)
        self.r1_button.setFlat(False)
        self.r1_button.setObjectName("r1_button")
        self.l2_button = QtWidgets.QPushButton(self.centralwidget)
        self.l2_button.setGeometry(QtCore.QRect(90, 280, 71, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.l2_button.setFont(font)
        self.l2_button.setAutoDefault(False)
        self.l2_button.setDefault(False)
        self.l2_button.setFlat(False)
        self.l2_button.setObjectName("l2_button")
        self.r2_button = QtWidgets.QPushButton(self.centralwidget)
        self.r2_button.setGeometry(QtCore.QRect(640, 280, 71, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.r2_button.setFont(font)
        self.r2_button.setAutoDefault(False)
        self.r2_button.setDefault(False)
        self.r2_button.setFlat(False)
        self.r2_button.setObjectName("r2_button")
        self.l2_label = QtWidgets.QLabel(self.centralwidget)
        self.l2_label.setGeometry(QtCore.QRect(170, 285, 67, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.l2_label.setFont(font)
        self.l2_label.setAlignment(QtCore.Qt.AlignCenter)
        self.l2_label.setObjectName("l2_label")
        self.r2_label = QtWidgets.QLabel(self.centralwidget)
        self.r2_label.setGeometry(QtCore.QRect(560, 285, 67, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.r2_label.setFont(font)
        self.r2_label.setAlignment(QtCore.Qt.AlignCenter)
        self.r2_label.setObjectName("r2_label")
        self.share_button = QtWidgets.QPushButton(self.centralwidget)
        self.share_button.setGeometry(QtCore.QRect(285, 300, 31, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.share_button.setFont(font)
        self.share_button.setAutoDefault(False)
        self.share_button.setDefault(False)
        self.share_button.setFlat(False)
        self.share_button.setObjectName("share_button")
        self.option_button = QtWidgets.QPushButton(self.centralwidget)
        self.option_button.setGeometry(QtCore.QRect(485, 300, 31, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.option_button.setFont(font)
        self.option_button.setAutoDefault(False)
        self.option_button.setDefault(False)
        self.option_button.setFlat(False)
        self.option_button.setObjectName("option_button")
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(50, 250, 700, 3))
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.timeline = QtWidgets.QTextBrowser(self.centralwidget)
        self.timeline.setGeometry(QtCore.QRect(530, 70, 256, 131))
        self.timeline.setObjectName("timeline")
        self.timer = QtWidgets.QTimeEdit(self.centralwidget)
        self.timer.setGeometry(QtCore.QRect(655, 10, 131, 51))
        font = QtGui.QFont()
        font.setPointSize(32)
        font.setBold(False)
        font.setWeight(50)
        font.setStrikeOut(False)
        self.timer.setFont(font)
        self.timer.setObjectName("timer")
        self.timer_reset_button = QtWidgets.QPushButton(self.centralwidget)
        self.timer_reset_button.setGeometry(QtCore.QRect(540, 210, 111, 25))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.timer_reset_button.setFont(font)
        self.timer_reset_button.setObjectName("timer_reset_button")
        self.log_reset_button = QtWidgets.QPushButton(self.centralwidget)
        self.log_reset_button.setGeometry(QtCore.QRect(660, 210, 111, 25))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.log_reset_button.setFont(font)
        self.log_reset_button.setObjectName("log_reset_button")
        self.windup_button = QtWidgets.QPushButton(self.centralwidget)
        self.windup_button.setGeometry(QtCore.QRect(20, 170, 141, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.windup_button.setFont(font)
        self.windup_button.setObjectName("windup_button")
        self.eject_button = QtWidgets.QPushButton(self.centralwidget)
        self.eject_button.setGeometry(QtCore.QRect(20, 100, 141, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.eject_button.setFont(font)
        self.eject_button.setObjectName("eject_button")
        self.cable_length_segment = QtWidgets.QLCDNumber(self.centralwidget)
        self.cable_length_segment.setGeometry(QtCore.QRect(20, 20, 141, 71))
        self.cable_length_segment.setDigitCount(4)
        self.cable_length_segment.setProperty("value", 0.0)
        self.cable_length_segment.setObjectName("cable_length_segment")
        self.inverse_button = QtWidgets.QPushButton(self.centralwidget)
        self.inverse_button.setGeometry(QtCore.QRect(530, 20, 111, 25))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.inverse_button.setFont(font)
        self.inverse_button.setObjectName("inverse_button")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.up_button.setText(_translate("MainWindow", "UP"))
        self.down_button.setText(_translate("MainWindow", "DN"))
        self.right_button.setText(_translate("MainWindow", "RT"))
        self.left_button.setText(_translate("MainWindow", "LT"))
        self.circle_button.setText(_translate("MainWindow", "CIR"))
        self.triangle_button.setText(_translate("MainWindow", "TRI"))
        self.cross_button.setText(_translate("MainWindow", "CRS"))
        self.square_button.setText(_translate("MainWindow", "SQR"))
        self.joy_left_xlabel.setText(_translate("MainWindow", "Lx : -0.00"))
        self.joy_left_ylabel.setText(_translate("MainWindow", "Ly : -0.00"))
        self.joy_right_xlabel.setText(_translate("MainWindow", "Rx : -0.00"))
        self.joy_right_ylabel.setText(_translate("MainWindow", "Ry : -0.00"))
        self.l1_button.setText(_translate("MainWindow", "L1"))
        self.r1_button.setText(_translate("MainWindow", "R1"))
        self.l2_button.setText(_translate("MainWindow", "L2"))
        self.r2_button.setText(_translate("MainWindow", "R2"))
        self.l2_label.setText(_translate("MainWindow", "L2 : 0.0"))
        self.r2_label.setText(_translate("MainWindow", "R2 : 0.0"))
        self.share_button.setText(_translate("MainWindow", "SH"))
        self.option_button.setText(_translate("MainWindow", "OP"))
        self.timer.setDisplayFormat(_translate("MainWindow", "mm:ss"))
        self.timer_reset_button.setText(_translate("MainWindow", "TimerReset"))
        self.log_reset_button.setText(_translate("MainWindow", "LogReset"))
        self.windup_button.setText(_translate("MainWindow", "巻取"))
        self.eject_button.setText(_translate("MainWindow", "排出"))
        self.inverse_button.setText(_translate("MainWindow", "Inverse"))
