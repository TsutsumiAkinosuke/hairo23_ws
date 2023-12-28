from PyQt5 import QtWidgets
from PyQt5 import QtCore
from PyQt5 import QtGui

import sys
import os

# sys.path.append("/home/akki/qt_ws/src/qt_sample/qt_sample/ui")
# from sample_ui2 import Ui_MainWindow

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy, QoSPresetProfiles, QoSProfile

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
        self.release_button.pressed.connect(self.pressed_release)
        self.release_button.released.connect(self.released_release)
        self.windup_button.pressed.connect(self.pressed_windup)
        self.windup_button.released.connect(self.released_windup)
        self.inverse_button.clicked.connect(self.inverse_direction)

        # ノードの初期化
        rclpy.init(args=None)
        self.node = Node("operator_node")

        # ジョイスティックのサブスクライバを定義
        self.joy_subscription = self.node.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.joy = Joy()

        # RobotCommandのパブリッシャを定義
        self.cmd_publisher = self.node.create_publisher(RobotCommand, "cmd", 10)
        self.rf_mode = False # True:サブ昇降・False:メイン昇降
        self.decon_auto = False # 自動除染ON/OFF
        self.cmd = RobotCommand()

        # RobotInfoのサブスクライバを定義
        self.info_subscription = self.node.create_subscription(RobotInfo, "info", self.info_callback, qos_profile=QoSPresetProfiles.SENSOR_DATA.value)
        self.robot_info = RobotInfo()

        # LimitSwitchのサブスクライバを定義
        self.ls_subscription = self.node.create_subscription(LimitSwitch, "ls", self.ls_callback, qos_profile=QoSPresetProfiles.SENSOR_DATA.value)
        self.ls = LimitSwitch()
        self.pre_ls = LimitSwitch()

        # タイマー設定
        self.start_time = time.time()
        self.current_time = 0

        # 処理周期の設定
        self.update_period = 10
        self.timer.start(self.update_period)

        # コントローラ用変数・マクロ

        self.pre_buttons = [False]*12
        self.is_forward = True
        self.windup_flag = False
        self.release_flag = False
        self.is_controlled = False

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

        self.LS_ELECAS_FORWARD_TOP     = 0
        self.LS_ELECAS_FORWARD_BOTTOM  = 1
        self.LS_ELECAS_BACK_TOP        = 2
        self.LS_ELECAS_BACK_BOTTOM     = 3
        self.LS_RF_MAIN_TOP            = 4
        self.LS_RF_MAIN_BOTTOM         = 5
        self.LS_RF_SUB_TOP             = 6
        self.LS_RF_SUB_BOTTOM          = 7
        self.LS_DECON_LEFT             = 8
        self.LS_DECON_RIGHT            = 9

        self.ls_dict = {
                        self.LS_ELECAS_FORWARD_TOP:"前方エレキャス上",
                        self.LS_ELECAS_FORWARD_BOTTOM:"前方エレキャス下",
                        self.LS_ELECAS_BACK_TOP:"後方エレキャス上",
                        self.LS_ELECAS_BACK_BOTTOM:"後方エレキャス下",
                        self.LS_RF_MAIN_TOP:"メイン昇降上",
                        self.LS_RF_MAIN_BOTTOM:"メイン昇降下",
                        self.LS_RF_SUB_TOP:"サブ昇降上",
                        self.LS_RF_SUB_BOTTOM:"サブ昇降下",
                        self.LS_DECON_LEFT:"除染機構左",
                        self.LS_DECON_RIGHT:"除染機構右"
                        }
    
    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)
        self.update_timer()
        self.update_view()
        self.show()
        self.timer.start(self.update_period)
    
    def joy_callback(self, msg):

        self.joy = msg

        # ジョイスティック
        if self.is_forward is True:
            self.cmd.value[0] = max(-127, min(127, int(self.joy.axes[1]*127)))  # left-y
            self.cmd.value[1] = max(-127, min(127, int(self.joy.axes[3]*127)))  # right-x
        else:
            self.cmd.value[0] = max(-127, min(127, int(self.joy.axes[1]*127))) * -1 # left-y
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

        # ケーブル長がUIによって操作されていなければ
        if self.is_controlled == False:
            if self.joy.axes[self.AXES_LR] == 1:
                # 左ボタンが押されているならケーブル巻取りオン・排出オフ
                self.windup_flag = True
                self.release_flag = False
            elif self.joy.axes[self.AXES_LR] == -1:
                # 右ボタンが押されているならケーブル巻取りオフ・排出オン
                self.windup_flag = False
                self.release_flag = True
            else:
                # 左右ボタンが押されていなければケーブル巻取りオフ・排出オフ
                self.windup_flag = False
                self.release_flag = False
        
        self.cmd.mode[self.CABLE_WIND] = self.windup_flag
        self.cmd.mode[self.CABLE_RELEASE] = self.release_flag

        # エレキャス

        # 三角ボタンが押されたとき前方エレキャスの展開・格納モードを切り替え
        if self.pre_buttons[self.BTN_TRIANGLE] == False and self.joy.buttons[self.BTN_TRIANGLE] == True:
            self.cmd.mode[self.ELECAS_FORWARD] = not self.cmd.mode[self.ELECAS_FORWARD]

        # バツボタンが押されたとき前方エレキャスの展開・格納モードを切り替え
        if self.pre_buttons[self.BTN_CROSS] == False and self.joy.buttons[self.BTN_CROSS] == True: # バツボタンが押されたとき
            self.cmd.mode[self.ELECAS_BACK] = not self.cmd.mode[self.ELECAS_BACK]

        # RobotCommandメッセージをパブリッシュ
        self.cmd_publisher.publish(self.cmd)

        # 1ループ前のコントローラの情報として保存
        self.pre_buttons = self.joy.buttons
    
    # RobotInfoメッセージ受信時のコールバック
    def info_callback(self, msg):

        # メッセージをメンバ変数に代入
        self.robot_info = msg

        # 7セグにケーブルの排出長さ(cm)を表示
        self.cable_length_segment.setProperty("value", self.robot_info.cable_length)
    
    def ls_callback(self, msg):

        self.ls = msg

        for i in range(10):
            if self.ls.ls[i] is not self.pre_ls.ls[i]:
                text = str(self.current_min).zfill(2)+":"+str(self.current_sec).zfill(2)+" | "+self.ls_dict[i]+"のリミットスイッチが押されました"
                self.timeline.append(text)
                print(text)

        self.pre_ls = self.ls

        if self.ls.ls[self.LS_RF_MAIN_TOP] == True:
            if self.ls.ls[self.LS_ELECAS_BACK_BOTTOM] == True:
                self.robot_image_label.setPixmap(self.robot_ee_pixmap)
            else:
                self.robot_image_label.setPixmap(self.robot_es_pixmap)
        else:
            self.robot_image_label.setPixmap(self.robot_ss_pixmap)

    # タイマーリセットボタン押下時のコールバック
    def reset_timer(self):

        # タイマーをリセット
        self.start_time = time.time()

    # ログリセットボタンクリック時のコールバック
    def reset_log(self):

        # タイムラインのログをリセット
        self.timeline.clear()

    # 排出ボタン押下時のコールバック
    def pressed_release(self):

        # 排出フラグをオンにする
        self.release_flag = True

        # 巻取りフラグをオフにする
        self.windup_flag = False

        # ケーブル長がUIによって操作されているフラグをオンにする
        self.is_controlled = True
    
    # 排出ボタン引上時のコールバック
    def released_release(self):

        # 排出フラグをオフにする
        self.release_flag = False

        # ケーブル長がUIによって操作されているフラグをオフにする
        self.is_controlled = False
    
    # 巻取りボタン押下時のコールバック
    def pressed_windup(self):

        # 巻取りフラグをオンにする
        self.windup_flag = True

        # 排出フラグをオフにする
        self.release_flag = False

        # ケーブル長がUIによって操作されているフラグをオンにする
        self.is_controlled = True
    
    # 巻取りボタン引上時のコールバック
    def released_windup(self):

        # 巻取りフラグをオフにする
        self.windup_flag = False

        # ケーブル長がUIによって操作されているフラグをオフにする
        self.is_controlled = False

    # 動作反転ボタン押下時のコールバック
    def inverse_direction(self):

        # クローラーの動作を反転させる
        self.is_forward = not self.is_forward
    
    def update_timer(self):
        # タイマーの時間を更新
        self.current_time = int(time.time() - self.start_time)
        self.current_min = int(self.current_time / 60)
        self.current_sec = self.current_time % 60
        self.running_timer.setTime(QtCore.QTime(0, self.current_min, self.current_sec))
        self.timer_bar.setProperty("value", min(100, int(self.current_time / 6)))

    def update_view(self):

        try:
            # UI上のジョイスティックを移動させる
            self.joy_left.move(310-int(self.joy.axes[0]*30), 840-int(self.joy.axes[1]*30))
            self.joy_right.move(530-int(self.joy.axes[3]*30), 840-int(self.joy.axes[4]*30))

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
        
        if self.cmd.mode[self.DECON_AUTO] == True:
            self.decon_mode_label.setText(self.translate("MainWindow", "除染モード：自動"))
        else:
            self.decon_mode_label.setText(self.translate("MainWindow", "除染モード：手動"))
        
        if self.cmd.mode[self.RF_MODE] == True:
            self.rf_mode_label.setText(self.translate("MainWindow", "昇降モード：サブ"))
        else:
            self.rf_mode_label.setText(self.translate("MainWindow", "昇降モード：メイン"))
        
        if self.cmd.mode[self.ELECAS_BACK] == True:
            self.elecas_back_label.setText(self.translate("MainWindow", "後足：展開"))
        else:
            self.elecas_back_label.setText(self.translate("MainWindow", "後足：格納"))
        
        if self.cmd.mode[self.ELECAS_FORWARD] == True:
            self.elecas_forward_label.setText(self.translate("MainWindow", "前足：展開"))
        else:
            self.elecas_forward_label.setText(self.translate("MainWindow", "前足：格納"))
        
        if self.is_forward == True:
            self.control_mode_label.setText(self.translate("MainWindow", "前進モード"))
        else:
            self.control_mode_label.setText(self.translate("MainWindow", "後進モード"))
    
    # def checkbutton_callback(self, state):
    #     if state == QtCore.Qt.Checked:
    #         self.checkBox.setText(self.translate("MainWindow", "M_Checked"))
    #     else:
    #         self.checkBox.setText(self.translate("MainWindow", "M_Unchecked"))
    
    def setupUi(self):
        self.setObjectName("MainWindow")
        self.resize(960, 1080)
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")

        # ジョイスティック左
        self.joy_left_xline = QtWidgets.QFrame(self.centralwidget)
        self.joy_left_xline.setGeometry(QtCore.QRect(279, 900, 180, 3))
        self.joy_left_xline.setFrameShape(QtWidgets.QFrame.HLine)
        self.joy_left_xline.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.joy_left_xline.setObjectName("joy_left_xline")
        self.joy_left_yline = QtWidgets.QFrame(self.centralwidget)
        self.joy_left_yline.setGeometry(QtCore.QRect(370, 810, 3, 180))
        self.joy_left_yline.setFrameShape(QtWidgets.QFrame.VLine)
        self.joy_left_yline.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.joy_left_yline.setObjectName("joy_left_yline")
        self.joy_left = QtWidgets.QDial(self.centralwidget)
        self.joy_left.setGeometry(QtCore.QRect(310, 840, 120, 120))
        self.joy_left.setWrapping(True)
        self.joy_left.setObjectName("joy_left")

        # ジョイスティック右
        self.joy_right_yline = QtWidgets.QFrame(self.centralwidget)
        self.joy_right_yline.setGeometry(QtCore.QRect(590, 810, 3, 180))
        self.joy_right_yline.setFrameShape(QtWidgets.QFrame.VLine)
        self.joy_right_yline.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.joy_right_yline.setObjectName("joy_right_yline")
        self.joy_right_xline = QtWidgets.QFrame(self.centralwidget)
        self.joy_right_xline.setGeometry(QtCore.QRect(500, 900, 180, 3))
        self.joy_right_xline.setFrameShape(QtWidgets.QFrame.HLine)
        self.joy_right_xline.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.joy_right_xline.setObjectName("joy_right_xline")
        self.joy_right = QtWidgets.QDial(self.centralwidget)
        self.joy_right.setGeometry(QtCore.QRect(530, 840, 120, 120))
        self.joy_right.setWrapping(True)
        self.joy_right.setObjectName("joy_right")

        font = QtGui.QFont()
        font.setPointSize(14)

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[0].setGeometry(QtCore.QRect(330, 760, 81, 21))
        self.label[0].setFont(font)
        self.label[0].setAlignment(QtCore.Qt.AlignCenter)
        self.label[0].setObjectName("joy_left_xlabel")

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[1].setGeometry(QtCore.QRect(330, 780, 81, 21))
        self.label[1].setFont(font)
        self.label[1].setAlignment(QtCore.Qt.AlignCenter)
        self.label[1].setObjectName("joy_left_ylabel")

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[2].setGeometry(QtCore.QRect(160, 680, 91, 21))
        self.label[2].setFont(font)
        self.label[2].setAlignment(QtCore.Qt.AlignCenter)
        self.label[2].setObjectName("l2_label")

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[3].setGeometry(QtCore.QRect(550, 760, 81, 21))
        self.label[3].setFont(font)
        self.label[3].setAlignment(QtCore.Qt.AlignCenter)
        self.label[3].setObjectName("joy_right_xlabel")

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[4].setGeometry(QtCore.QRect(550, 780, 81, 21))
        self.label[4].setFont(font)
        self.label[4].setAlignment(QtCore.Qt.AlignCenter)
        self.label[4].setObjectName("joy_right_ylabel")

        self.label.append(QtWidgets.QLabel(self.centralwidget))
        self.label[5].setGeometry(QtCore.QRect(706, 680, 81, 21))
        self.label[5].setFont(font)
        self.label[5].setAlignment(QtCore.Qt.AlignCenter)
        self.label[5].setObjectName("r2_label")

        # 矢印ボタン

        font = QtGui.QFont()
        font.setPointSize(18)

        self.up_button = QtWidgets.QPushButton(self.centralwidget)
        self.up_button.setGeometry(QtCore.QRect(90, 780, 61, 61))
        self.up_button.setFont(font)
        self.up_button.setObjectName("up_button")

        self.down_button = QtWidgets.QPushButton(self.centralwidget)
        self.down_button.setGeometry(QtCore.QRect(90, 920, 61, 61))
        self.down_button.setFont(font)
        self.down_button.setObjectName("down_button")

        self.right_button = QtWidgets.QPushButton(self.centralwidget)
        self.right_button.setGeometry(QtCore.QRect(160, 850, 61, 61))
        self.right_button.setFont(font)
        self.right_button.setObjectName("right_button")

        self.left_button = QtWidgets.QPushButton(self.centralwidget)
        self.left_button.setGeometry(QtCore.QRect(20, 850, 61, 61))
        self.left_button.setFont(font)
        self.left_button.setObjectName("left_button")

        # ボタン

        font = QtGui.QFont()
        font.setPointSize(18)

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[0].setGeometry(QtCore.QRect(810, 920, 61, 61))
        self.qt_buttons[0].setFont(font)
        self.qt_buttons[0].setObjectName("cross_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[1].setGeometry(QtCore.QRect(880, 850, 61, 61))
        self.qt_buttons[1].setFont(font)
        self.qt_buttons[1].setObjectName("circle_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[2].setGeometry(QtCore.QRect(810, 780, 61, 61))
        self.qt_buttons[2].setFont(font)
        self.qt_buttons[2].setObjectName("triangle_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[3].setGeometry(QtCore.QRect(740, 850, 61, 61))
        self.qt_buttons[3].setFont(font)
        self.qt_buttons[3].setObjectName("square_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[4].setGeometry(QtCore.QRect(80, 720, 81, 41))
        self.qt_buttons[4].setFont(font)
        self.qt_buttons[4].setObjectName("l1_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[5].setGeometry(QtCore.QRect(800, 720, 81, 41))
        self.qt_buttons[5].setFont(font)
        self.qt_buttons[5].setObjectName("r1_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[6].setGeometry(QtCore.QRect(80, 670, 81, 41))
        self.qt_buttons[6].setFont(font)
        self.qt_buttons[6].setObjectName("l2_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[7].setGeometry(QtCore.QRect(800, 670, 81, 41))
        self.qt_buttons[7].setFont(font)
        self.qt_buttons[7].setObjectName("r2_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[8].setGeometry(QtCore.QRect(200, 750, 41, 51))
        self.qt_buttons[8].setFont(font)
        self.qt_buttons[8].setObjectName("share_button")

        self.qt_buttons.append(QtWidgets.QPushButton(self.centralwidget))
        self.qt_buttons[9].setGeometry(QtCore.QRect(720, 750, 41, 51))
        self.qt_buttons[9].setFont(font)
        self.qt_buttons[9].setObjectName("option_button")

        # 横線

        self.line_1 = QtWidgets.QFrame(self.centralwidget)
        self.line_1.setGeometry(QtCore.QRect(9, 640, 941, 20))
        self.line_1.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_1.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_1.setObjectName("line")

        self.line_2 = QtWidgets.QFrame(self.centralwidget)
        self.line_2.setGeometry(QtCore.QRect(450, 460, 491, 20))
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")

        self.line_3 = QtWidgets.QFrame(self.centralwidget)
        self.line_3.setGeometry(QtCore.QRect(450, 190, 491, 20))
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")

        font = QtGui.QFont()
        font.setPointSize(14)

        # 操作反転ボタン
        self.inverse_button = QtWidgets.QPushButton(self.centralwidget)
        self.inverse_button.setGeometry(QtCore.QRect(420, 710, 121, 41))
        self.inverse_button.setFont(font)
        self.inverse_button.setObjectName("inverse_button")

        # ログCSV出力ボタン
        self.csv_output_button = QtWidgets.QPushButton(self.centralwidget)
        self.csv_output_button.setGeometry(QtCore.QRect(700, 410, 231, 41))
        self.csv_output_button.setFont(font)
        self.csv_output_button.setObjectName("csv_output_button")

        # 巻取ボタン
        self.windup_button = QtWidgets.QPushButton(self.centralwidget)
        self.windup_button.setGeometry(QtCore.QRect(450, 570, 141, 61))
        self.windup_button.setFont(font)
        self.windup_button.setObjectName("windup_button")

        # 排出ボタン
        self.release_button = QtWidgets.QPushButton(self.centralwidget)
        self.release_button.setGeometry(QtCore.QRect(450, 490, 141, 61))
        self.release_button.setFont(font)
        self.release_button.setObjectName("release_button")

        # タイマーリセットボタン
        self.timer_reset_button = QtWidgets.QPushButton(self.centralwidget)
        self.timer_reset_button.setGeometry(QtCore.QRect(770, 58, 171, 121))
        self.timer_reset_button.setFont(font)
        self.timer_reset_button.setObjectName("timer_reset_button")

        # ログラインリセットボタン
        self.log_reset_button = QtWidgets.QPushButton(self.centralwidget)
        self.log_reset_button.setGeometry(QtCore.QRect(450, 410, 231, 41))
        self.log_reset_button.setFont(font)
        self.log_reset_button.setObjectName("log_reset_button")

        # 前エレキャスモードラベル
        self.elecas_forward_label = QtWidgets.QLabel(self.centralwidget)
        self.elecas_forward_label.setGeometry(QtCore.QRect(340, 600, 101, 31))
        self.elecas_forward_label.setFont(font)
        self.elecas_forward_label.setObjectName("elecas_forward_label")

        # 後エレキャスモードラベル
        self.elecas_back_label = QtWidgets.QLabel(self.centralwidget)
        self.elecas_back_label.setGeometry(QtCore.QRect(10, 600, 101, 31))
        self.elecas_back_label.setFont(font)
        self.elecas_back_label.setObjectName("elecas_back_label")

        # 除染モードラベル
        self.decon_mode_label = QtWidgets.QLabel(self.centralwidget)
        self.decon_mode_label.setGeometry(QtCore.QRect(250, 20, 151, 31))
        self.decon_mode_label.setFont(font)
        self.decon_mode_label.setObjectName("decon_mode_label")

        # 昇降モードラベル
        self.rf_mode_label = QtWidgets.QLabel(self.centralwidget)
        self.rf_mode_label.setGeometry(QtCore.QRect(30, 20, 171, 31))
        self.rf_mode_label.setFont(font)
        self.rf_mode_label.setObjectName("rf_mode_label")

        # 制御モードラベル
        self.control_mode_label = QtWidgets.QLabel(self.centralwidget)
        self.control_mode_label.setGeometry(QtCore.QRect(430, 680, 101, 21))
        self.control_mode_label.setFont(font)
        self.control_mode_label.setAlignment(QtCore.Qt.AlignCenter)
        self.control_mode_label.setObjectName("control_mode_label")

        # ケーブル長セグメント
        self.cable_length_segment = QtWidgets.QLCDNumber(self.centralwidget)
        self.cable_length_segment.setGeometry(QtCore.QRect(600, 490, 261, 141))
        self.cable_length_segment.setProperty("value", 0)
        self.cable_length_segment.setObjectName("cable_length_segment")

        # タイムライン
        self.timeline = QtWidgets.QTextBrowser(self.centralwidget)
        self.timeline.setGeometry(QtCore.QRect(451, 220, 481, 171))
        self.timeline.setObjectName("timeline")

        # タイマー
        font = QtGui.QFont()
        font.setPointSize(77)
        self.running_timer = QtWidgets.QTimeEdit(self.centralwidget)
        self.running_timer.setFont(font)
        self.running_timer.setAlignment(QtCore.Qt.AlignCenter)
        self.running_timer.setGeometry(QtCore.QRect(460, 58, 301, 121))
        self.running_timer.setObjectName("running_timer")

        # プログレスバー
        self.timer_bar = QtWidgets.QProgressBar(self.centralwidget)
        self.timer_bar.setGeometry(QtCore.QRect(460, 20, 481, 31))
        self.timer_bar.setProperty("value", 0)
        self.timer_bar.setFormat("")
        self.timer_bar.setObjectName("timer_bar")

        # cm
        font = QtGui.QFont()
        font.setPointSize(36)
        self.cm_label = QtWidgets.QLabel(self.centralwidget)
        self.cm_label.setGeometry(QtCore.QRect(870, 590, 61, 41))
        self.cm_label.setFont(font)
        self.cm_label.setObjectName("cm_label")

        # ロボット画像
        self.robot_image_label = QtWidgets.QLabel(self.centralwidget)
        self.robot_image_label.setGeometry(QtCore.QRect(120, 80, 200, 550))
        self.robot_image_label.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.robot_image_label.setObjectName("robot_image_label")
        self.robot_ss_pixmap = QtGui.QPixmap(os.path.expanduser("~/hairo23_ws/src/robot_ss.png"))
        self.robot_ss_pixmap = self.robot_ss_pixmap.scaled(200, 550)
        self.robot_es_pixmap = QtGui.QPixmap(os.path.expanduser("~/hairo23_ws/src/robot_es.png"))
        self.robot_es_pixmap = self.robot_es_pixmap.scaled(200, 550)
        self.robot_ee_pixmap = QtGui.QPixmap(os.path.expanduser("~/hairo23_ws/src/robot_ee.png"))
        self.robot_ee_pixmap = self.robot_ee_pixmap.scaled(200, 550)
        self.robot_image_label.setPixmap(self.robot_ss_pixmap)

        self.setCentralWidget(self.centralwidget)
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
        self.timer_reset_button.setText(self.translate("MainWindow", "タイマーリセット"))
        self.log_reset_button.setText(self.translate("MainWindow", "ログリセット"))
        self.windup_button.setText(self.translate("MainWindow", "巻取（ー）"))
        self.release_button.setText(self.translate("MainWindow", "排出（＋）"))
        self.inverse_button.setText(self.translate("MainWindow", "操作反転"))
        self.csv_output_button.setText(self.translate("MainWindow", "CSV出力"))
        self.control_mode_label.setText(self.translate("MainWindow", "前進モード"))
        self.elecas_back_label.setText(self.translate("MainWindow", "後足：格納"))
        self.elecas_forward_label.setText(self.translate("MainWindow", "前足：格納"))
        self.rf_mode_label.setText(self.translate("MainWindow", "昇降モード：メイン"))
        self.decon_mode_label.setText(self.translate("MainWindow", "除染モード：手動"))
        self.cm_label.setText(self.translate("MainWindow", "cm"))

def ros_main(args=None):

    app = QtWidgets.QApplication(sys.argv)
    win = OperatorNode()
    win.show()
    sys.exit(app.exec_())
        
        
if __name__ == "__main__":
    ros_main()