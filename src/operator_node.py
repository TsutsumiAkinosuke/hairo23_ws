from PyQt5 import QtWidgets
from PyQt5 import QtCore
from PyQt5 import QtGui

import sys

# sys.path.append("/home/akki/qt_ws/src/qt_sample/qt_sample/ui")
# from sample_ui2 import Ui_MainWindow

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from custom_interfaces.msg import RobotCommand
from custom_interfaces.msg import RobotInfo
from custom_interfaces.msg import LimitSwitch

import time

class OperatorNode(QtWidgets.QMainWindow):

    def __init__(self, parent=None):

        super(OperatorNode, self).__init__(parent)

        # ボタンとラベルのリスト
        self.qt_buttons = []
        self.label = []
        self.label_name = ["Lx", "Ly", "L2", "Rx", "Ry", "R2"]

        # self.ui = Ui_MainWindow
        # self.ui.setupUi(self)
        self.setupUi()

        # QtTimerのインスタンスを生成してros_spin関数をバインド
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.ros_spin)

        # プッシュボタンに関数をバインド
        self.cable_length = 0.0
        self.timer_reset_button.clicked.connect(self.reset_timer)
        self.log_reset_button.clicked.connect(self.reset_log)
        self.eject_button.clicked.connect(self.eject_cable)
        self.windup_button.clicked.connect(self.windup_cable)

        # ノードの初期化
        rclpy.init(args=None)
        self.node = Node("operator_node")

        # ジョイスティックのサブスクライバを定義
        self.joy_subscription = self.node.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.joy = Joy()

        # 変換コマンドのパブリッシャを定義
        self.cmd_publisher = self.node.create_publisher(RobotCommand, "cmd", 10)
        self.rf_mode = False # True:サブ昇降・False:メイン昇降
        self.decon_auto = False # 自動除染ON/OFF
        self.cmd = RobotCommand()

        # ロボット情報のサブスクライバを定義
        self.info_subscription = self.node.create_subscription(RobotInfo, "info", self.info_callback, 10)
        self.robot_info = RobotInfo()

        # タイマー設定
        self.start_time = time.time()
        self.current_time = 0

        # 処理周期の設定
        self.update_period = 10
        self.timer.start(self.update_period)

        # コントローラ用変数・マクロ

        self.pre_buttons = [False]*12

        self.AXES_JOY_LX = 0
        self.AXES_JOY_LY = 1
        self.AXES_L2 = 2
        self.AXES_JOY_RX = 3
        self.AXES_JOY_RY = 4
        self.AXES_R2 = 5
        self.AXES_LR = 6
        self.AXES_UD = 7

        self.BTN_CROSS = 0
        self.BTN_CIRCLE = 1
        self.BTN_TRIANGLE = 2
        self.BTN_SQUARE = 3
        self.BTN_L1 = 4
        self.BTN_R1 = 5
        self.BTN_L2 = 6
        self.BTN_R2 = 7
        self.BTN_SHARE = 8
        self.BTN_OPTION = 9
        self.BTN_PS = 10
        self.BTN_JOY_L = 11
        self.BTN_JOY_R = 12

        # コマンドメッセージ用マクロ

        self.CRAWLER_L = 0
        self.CRAWLER_R = 1

        self.ELECAS_FORWARD = 0
        self.ELECAS_BACK = 1
        self.RF_MODE = 2
        self.RF_RISE = 3
        self.RF_FALL = 4
        self.DECON_AUTO = 5
        self.DECON_LEFT = 6
        self.DECON_RIGHT = 7
        self.CABLE_WIND = 8
        self.CABLE_RELEASE = 9
    
    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)
        self.update_timer()
        self.update_view()
        self.show()
        self.timer.start(self.update_period)
    
    def joy_callback(self, msg):

        self.joy = msg

        # ジョイスティック
        self.cmd.value[0] = max(-127, min(127, int(self.joy.axes[1]*127)))  # left-y
        self.cmd.value[1] = max(-127, min(127, int(self.joy.axes[3]*127)))  # right-x

        # 昇降モード
        if self.pre_buttons[self.BTN_L2] == False and self.joy.buttons[self.BTN_L2] == True: # L2ボタンが押されたとき
            self.cmd.mode[self.RF_MODE] = not self.cmd.mode[self.RF_MODE]   # 昇降モード代入

        # 昇降
        if self.joy.axes[self.AXES_UD] == 1:    # 上ボタンが押さているなら
            self.cmd.mode[self.RF_RISE] = True
            self.cmd.mode[self.RF_FALL] = False
        elif self.joy.axes[self.AXES_UD] == -1: # 下ボタンが押されているなら
            self.cmd.mode[self.RF_RISE] = False
            self.cmd.mode[self.RF_FALL] = True
        else:                                   # 上下ボタンが押されていなければ
            self.cmd.mode[self.RF_RISE] = False
            self.cmd.mode[self.RF_FALL] = False

        if self.pre_buttons[self.BTN_R2] == False and self.joy.buttons[self.BTN_R2] == True: # R2ボタンが押されたとき
            self.cmd.mode[self.DECON_AUTO] = not self.cmd.mode[self.DECON_AUTO]   # 自動除染ON/OFF切替

        self.cmd.mode[self.DECON_LEFT] = bool(self.joy.buttons[self.BTN_L1])  # 除染機構を左へ移動
        self.cmd.mode[self.DECON_RIGHT] = bool(self.joy.buttons[self.BTN_R1]) # 除染機構を右へ移動

        if self.joy.axes[self.AXES_LR] == 1:    # 左ボタンが押されているなら
            self.cmd.mode[self.CABLE_WIND] = True
            self.cmd.mode[self.CABLE_RELEASE] = False
        elif self.joy.axes[self.AXES_LR] == -1:  # 右ボタンが押されているなら
            self.cmd.mode[self.CABLE_WIND] = False
            self.cmd.mode[self.CABLE_RELEASE] = True
        else:                                   # 左右ボタンが押されていなければ
            self.cmd.mode[self.CABLE_WIND] = False
            self.cmd.mode[self.CABLE_RELEASE] = False
        
        # エレキャス
        if self.pre_buttons[self.BTN_TRIANGLE] == False and self.joy.buttons[self.BTN_TRIANGLE] == True: # 三角ボタンが押されたとき
            self.cmd.mode[self.ELECAS_FORWARD] = not self.cmd.mode[self.ELECAS_FORWARD]

        if self.pre_buttons[self.BTN_CROSS] == False and self.joy.buttons[self.BTN_CROSS] == True: # バツボタンが押されたとき
            self.cmd.mode[self.ELECAS_BACK] = not self.cmd.mode[self.ELECAS_BACK]

        self.cmd_publisher.publish(self.cmd)

        self.pre_buttons = self.joy.buttons
    
    # RobotInfoメッセージ受信時のコールバック
    def info_callback(self, msg):

        self.robot_info = msg

    # タイマーをリセット
    def reset_timer(self):

        self.start_time = time.time()

    # タイムラインをリセット
    def reset_log(self):

        self.timeline.clear()
    
    # ケーブルを排出してセグメントの表示を更新
    def eject_cable(self):

        self.cable_length += 0.25
        self.cable_length_segment.setProperty("value", self.cable_length)
    
    # ケーブルを巻き取ってセグメントの表示を更新
    def windup_cable(self):

        self.cable_length -= 0.25
        self.cable_length_segment.setProperty("value", self.cable_length)
    
    def update_timer(self):
        # タイマーの時間を更新
        self.current_time = int(time.time() - self.start_time)
        self.current_min = int(self.current_time / 60)
        self.current_sec = self.current_time % 60
        self.running_timer.setTime(QtCore.QTime(0, self.current_min, self.current_sec))

    def update_view(self):

        try:
            # UI上のジョイスティックを移動させる
            self.joy_left.move(250-int(self.joy.axes[0]*20), 430-int(self.joy.axes[1]*20))
            self.joy_right.move(450-int(self.joy.axes[3]*20), 430-int(self.joy.axes[4]*20))

            # Lx ~ R2までの値をラベルに表示
            for i in range(6):
                self.label[i].setText(self.translate("MainWindow", self.label_name[i] + ": " + "{0:.2f}".format(self.joy.axes[i])))
            
            # 左右ボタンが押されたら赤表示
            if self.joy.axes[6] == 1:
                self.left_button.setStyleSheet("background-color: red;")
                self.right_button.setStyleSheet("")
            elif self.joy.axes[6] == -1:
                self.left_button.setStyleSheet("")
                self.right_button.setStyleSheet("background-color: red;")
            else:
                self.left_button.setStyleSheet("")
                self.right_button.setStyleSheet("")
            
            # 上下ボタンが押されたら赤表示
            if self.joy.axes[7] == 1:
                self.up_button.setStyleSheet("background-color: red;")
                self.down_button.setStyleSheet("")
            elif self.joy.axes[7] == -1:
                self.up_button.setStyleSheet("")
                self.down_button.setStyleSheet("background-color: red;")
            else:
                self.up_button.setStyleSheet("")
                self.down_button.setStyleSheet("")

            # CROSS ~ OPTIONまでボタンが押されたら赤表示
            for i in range(10):
                if self.joy.buttons[i] == 1:
                    self.qt_buttons[i].setStyleSheet("background-color: red;")
                else:
                    self.qt_buttons[i].setStyleSheet("")

        except:
            pass
    
    # def checkbutton_callback(self, state):
    #     if state == QtCore.Qt.Checked:
    #         self.checkBox.setText(self.translate("MainWindow", "M_Checked"))
    #     else:
    #         self.checkBox.setText(self.translate("MainWindow", "M_Unchecked"))
    
    def setupUi(self):
        self.setObjectName("MainWindow")
        self.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(self)
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

        font = QtGui.QFont()
        font.setPointSize(14)

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[0].setGeometry(QtCore.QRect(260, 360, 81, 21))
        self.label[0].setFont(font)
        self.label[0].setAlignment(QtCore.Qt.AlignCenter)
        self.label[0].setObjectName("joy_left_xlabel")

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[1].setGeometry(QtCore.QRect(260, 380, 81, 21))
        self.label[1].setFont(font)
        self.label[1].setAlignment(QtCore.Qt.AlignCenter)
        self.label[1].setObjectName("joy_left_ylabel")

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[2].setGeometry(QtCore.QRect(170, 285, 81, 21))
        self.label[2].setFont(font)
        self.label[2].setAlignment(QtCore.Qt.AlignCenter)
        self.label[2].setObjectName("l2_label")

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[3].setGeometry(QtCore.QRect(460, 360, 81, 21))
        self.label[3].setFont(font)
        self.label[3].setAlignment(QtCore.Qt.AlignCenter)
        self.label[3].setObjectName("joy_right_xlabel")

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[4].setGeometry(QtCore.QRect(460, 380, 81, 21))
        self.label[4].setFont(font)
        self.label[4].setAlignment(QtCore.Qt.AlignCenter)
        self.label[4].setObjectName("joy_right_ylabel")

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[5].setGeometry(QtCore.QRect(560, 285, 81, 21))
        self.label[5].setFont(font)
        self.label[5].setAlignment(QtCore.Qt.AlignCenter)
        self.label[5].setObjectName("r2_label")

        # 矢印ボタン

        self.up_button = QtWidgets.QPushButton(self.centralwidget)
        self.up_button.setGeometry(QtCore.QRect(100, 370, 51, 51))
        self.up_button.setFont(font)
        self.up_button.setObjectName("up_button")

        self.down_button = QtWidgets.QPushButton(self.centralwidget)
        self.down_button.setGeometry(QtCore.QRect(100, 490, 51, 51))
        self.down_button.setFont(font)
        self.down_button.setObjectName("down_button")

        self.right_button = QtWidgets.QPushButton(self.centralwidget)
        self.right_button.setGeometry(QtCore.QRect(160, 430, 51, 51))
        self.right_button.setFont(font)
        self.right_button.setObjectName("right_button")

        self.left_button = QtWidgets.QPushButton(self.centralwidget)
        self.left_button.setGeometry(QtCore.QRect(40, 430, 51, 51))
        self.left_button.setFont(font)
        self.left_button.setObjectName("left_button")

        font = QtGui.QFont()
        font.setPointSize(14)

        # ボタン

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[0].setGeometry(QtCore.QRect(650, 490, 51, 51))
        self.qt_buttons[0].setFont(font)
        self.qt_buttons[0].setObjectName("cross_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[1].setGeometry(QtCore.QRect(710, 430, 51, 51))
        self.qt_buttons[1].setFont(font)
        self.qt_buttons[1].setObjectName("circle_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[2].setGeometry(QtCore.QRect(650, 370, 51, 51))
        self.qt_buttons[2].setFont(font)
        self.qt_buttons[2].setObjectName("triangle_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[3].setGeometry(QtCore.QRect(590, 430, 51, 51))
        self.qt_buttons[3].setFont(font)
        self.qt_buttons[3].setObjectName("square_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[4].setGeometry(QtCore.QRect(90, 320, 71, 31))
        self.qt_buttons[4].setFont(font)
        self.qt_buttons[4].setObjectName("l1_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[5].setGeometry(QtCore.QRect(640, 320, 71, 31))
        self.qt_buttons[5].setFont(font)
        self.qt_buttons[5].setObjectName("r1_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[6].setGeometry(QtCore.QRect(90, 280, 71, 31))
        self.qt_buttons[6].setFont(font)
        self.qt_buttons[6].setObjectName("l2_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[7].setGeometry(QtCore.QRect(640, 280, 71, 31))
        self.qt_buttons[7].setFont(font)
        self.qt_buttons[7].setObjectName("r2_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[8].setGeometry(QtCore.QRect(285, 300, 31, 41))
        self.qt_buttons[8].setFont(font)
        self.qt_buttons[8].setObjectName("share_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[9].setGeometry(QtCore.QRect(485, 300, 31, 41))
        self.qt_buttons[9].setFont(font)
        self.qt_buttons[9].setObjectName("option_button")

        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(50, 250, 700, 3))
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")

        # 巻取ボタン
        self.windup_button = QtWidgets.QPushButton(self.centralwidget)
        self.windup_button.setGeometry(QtCore.QRect(20, 170, 141, 61))
        self.windup_button.setFont(font)
        self.windup_button.setObjectName("windup_button")

        # 排出ボタン
        self.eject_button = QtWidgets.QPushButton(self.centralwidget)
        self.eject_button.setGeometry(QtCore.QRect(20, 100, 141, 61))
        self.eject_button.setFont(font)
        self.eject_button.setObjectName("eject_button")

        # ケーブル長セグメント
        self.cable_length_segment = QtWidgets.QLCDNumber(self.centralwidget)
        self.cable_length_segment.setGeometry(QtCore.QRect(20, 20, 141, 71))
        self.cable_length_segment.setDigitCount(4)
        self.cable_length_segment.setProperty("value", 0.0)
        self.cable_length_segment.setObjectName("cable_length_segment")

        # タイマーリセットボタン
        self.timer_reset_button = QtWidgets.QPushButton(self.centralwidget)
        self.timer_reset_button.setGeometry(QtCore.QRect(540, 210, 111, 25))
        self.timer_reset_button.setFont(font)
        self.timer_reset_button.setObjectName("timer_reset_button")

        # タイムラインリセットボタン
        self.log_reset_button = QtWidgets.QPushButton(self.centralwidget)
        self.log_reset_button.setGeometry(QtCore.QRect(660, 210, 111, 25))
        self.log_reset_button.setFont(font)
        self.log_reset_button.setObjectName("log_reset_button")

        # タイムライン
        self.timeline = QtWidgets.QTextBrowser(self.centralwidget)
        self.timeline.setGeometry(QtCore.QRect(530, 70, 256, 131))
        self.timeline.setObjectName("timeline")

        # タイマー
        font.setPointSize(32)
        font.setWeight(50)
        self.running_timer = QtWidgets.QTimeEdit(self.centralwidget)
        self.running_timer.setFont(font)
        self.running_timer.setGeometry(QtCore.QRect(655, 10, 131, 51))
        self.running_timer.setObjectName("running_timer")

        self.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(self)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)
    
    def retranslateUi(self):
        self.translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(self.translate("MainWindow", "MainWindow"))

        self.up_button.setText(self.translate("MainWindow", "UP"))
        self.down_button.setText(self.translate("MainWindow", "DN"))
        self.right_button.setText(self.translate("MainWindow", "RT"))
        self.left_button.setText(self.translate("MainWindow", "LT"))

        self.label[0].setText(self.translate("MainWindow", "X : 0.00"))
        self.label[1].setText(self.translate("MainWindow", "Y : 0.00"))
        self.label[2].setText(self.translate("MainWindow", "L2 : 0.0"))
        self.label[3].setText(self.translate("MainWindow", "X : 0.00"))
        self.label[4].setText(self.translate("MainWindow", "Y : 0.00"))
        self.label[5].setText(self.translate("MainWindow", "R2 : 0.0"))

        self.qt_buttons[0].setText(self.translate("MainWindow", "CRS"))
        self.qt_buttons[1].setText(self.translate("MainWindow", "CIR"))
        self.qt_buttons[2].setText(self.translate("MainWindow", "TRI"))
        self.qt_buttons[3].setText(self.translate("MainWindow", "SQR"))
        self.qt_buttons[4].setText(self.translate("MainWindow", "L1"))
        self.qt_buttons[5].setText(self.translate("MainWindow", "R1"))
        self.qt_buttons[6].setText(self.translate("MainWindow", "L2"))
        self.qt_buttons[7].setText(self.translate("MainWindow", "R2"))
        self.qt_buttons[8].setText(self.translate("MainWindow", "SH"))
        self.qt_buttons[9].setText(self.translate("MainWindow", "OP"))
        self.qt_buttons[2].setText(self.translate("MainWindow", "TRI"))
        self.qt_buttons[3].setText(self.translate("MainWindow", "SQR"))
        self.qt_buttons[4].setText(self.translate("MainWindow", "L1"))
        self.qt_buttons[5].setText(self.translate("MainWindow", "R1"))
        self.qt_buttons[6].setText(self.translate("MainWindow", "L2"))
        self.qt_buttons[7].setText(self.translate("MainWindow", "R2"))
        self.qt_buttons[8].setText(self.translate("MainWindow", "SH"))
        self.qt_buttons[9].setText(self.translate("MainWindow", "OP"))

        self.running_timer.setDisplayFormat(self.translate("MainWindow", "mm:ss"))
        self.timer_reset_button.setText(self.translate("MainWindow", "TimerReset"))
        self.log_reset_button.setText(self.translate("MainWindow", "LogReset"))
        self.windup_button.setText(self.translate("MainWindow", "巻取"))
        self.eject_button.setText(self.translate("MainWindow", "排出"))

def ros_main(args=None):

    app = QtWidgets.QApplication(sys.argv)
    win = OperatorNode()
    win.show()
    sys.exit(app.exec_())
        
        
if __name__ == "__main__":
    ros_main()