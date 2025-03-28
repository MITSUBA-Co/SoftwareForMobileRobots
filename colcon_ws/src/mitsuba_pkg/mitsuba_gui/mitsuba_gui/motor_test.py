#!/usr/bin/env python3
# Copyright 2024 MITSUBA Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import csv
import os
import threading
import time
import tkinter as tk
import tkinter.ttk as ttk
from tkinter import messagebox

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from can_msgs.msg import Frame
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from rclpy.node import Node
import subprocess

from mitsuba_msgs.msg import MitsubaState


class MotorTest(Node):

    def __init__(self, tab):
        super().__init__('motor_test')
        self.tab = tab
        self.timeout = 1
        self.pub_can_frame = self.create_publisher(Frame, 'to_can_bus', 10)

        self.mitsuba_gui_dir = get_package_share_directory('mitsuba_gui')
        self.gui_set_csv_path = os.path.join(self.mitsuba_gui_dir, 'guisetting/guisetting.csv')
        print('gui_set_csv_path:', self.gui_set_csv_path)

        if self.gui_set_csv_load('OperationMode') == '0':
            self.current_op_mode = 0x00
        #            self.switch_duty_mode()       # 初期状態は制御モードをDuty制御としておく
        else:
            self.current_op_mode = 0x01
        #            self.switch_pi_mode()

        self.create_widgets()

        self.flg_error1 = False
        self.flg_error2 = False
        self.flg_fatal_error1 = False
        self.flg_fatal_error2 = False
        self.flg_low_voltage_fault = False

    # 初期設定用CSVファイルから設定値を読み込む関数
    def gui_set_csv_load(self, param_name):
        with open(self.gui_set_csv_path, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                # フィールドの数が2(パラメータ名,設定値 の記述)以外は無視する
                if len(row) == 2 and row[0] == param_name:
                    return row[1]
        return 'Error'

    # 初期設定用CSVファイルの設定値を更新する関数
    def gui_set_csv_update(self, new_val, param_name):
        with open(self.gui_set_csv_path, 'r') as file:
            reader = csv.reader(file)
            rows = list(reader)

        with open(self.gui_set_csv_path, 'w', newline='') as file:
            writer = csv.writer(file)
            for row in rows:
                # フィールドの数が2(パラメータ名,設定値 の記述)以外は無視する
                if len(row) == 2 and row[0] == param_name:
                    row[1] = str(new_val)
                writer.writerow(row)

    def create_widgets(self):
        color_motor_info = '#c7ffca'
        color_motor_cont = '#c7ffca'

        # main frame
        main_frame = tk.Frame(self.tab, bg='#c7ffca')
        main_frame.pack(expand=True, fill='both', padx=0, pady=0)

        # ------------------------------------ 制御モード切替 ------------------------------------#
        frame_op_mode_sel = tk.LabelFrame(
            main_frame, text='制御モード切替', pady=0, bd=1, relief='solid', bg=color_motor_cont
        )
        frame_op_mode_sel.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky=tk.W)

        label_op_mode_sel = tk.Label(frame_op_mode_sel, text='制御モード', bg=color_motor_cont)
        label_op_mode_sel.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # 制御モード選択用プルダウン
        op_mode_list = ['duty制御', '回転数PI制御']
        self.combobox_op_mode_sel = ttk.Combobox(frame_op_mode_sel, width=10, values=op_mode_list)
        self.combobox_op_mode_sel.set(op_mode_list[self.current_op_mode])
        self.combobox_op_mode_sel.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        # 制御モード反映用ボタン
        self.button_sel_op_mode = tk.Button(
            frame_op_mode_sel, text='モード切替', command=self.set_op_mode
        )
        self.button_sel_op_mode.grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)

    def set_op_mode(self):
        ret = messagebox.askokcancel('制御モード切替', 'モードを切り替えますか？')
        if ret:
            # 制御モード変更中は回転ボタンを無効化
            self.button1_toolbar_m1.config(state='disabled')
            self.button2_toolbar_m1.configure(state='disabled')
            self.button1_toolbar_M2.config(state='disabled')
            self.button2_toolbar_M2.configure(state='disabled')

            #self.set_scale()  # スライドバーの単位とレンジを設定

            if self.combobox_op_mode_sel.get() == 'duty制御':
                self.switch_duty_mode()
                print('duty制御')
            elif self.combobox_op_mode_sel.get() == '回転数PI制御':
                self.switch_pi_mode()
                print('回転数PI制御')
            else:
                print('Mode Error')

            # スライドバーを0へ
            self.scale_m1.set(0)
            self.scale_m2.set(0)

            # 回転ボタンを有効化
            self.button1_toolbar_m1.config(state='normal')
            self.button2_toolbar_m1.configure(state='normal')
            self.button1_toolbar_M2.config(state='normal')
            self.button2_toolbar_M2.configure(state='normal')

    def publish_can_frame(self, can_id, can_data):
        can_frame = Frame()
        can_frame.id = can_id
        can_frame.dlc = 8
        can_frame.is_rtr = False
        can_frame.is_extended = False
        can_frame.data = can_data
        self.pub_can_frame.publish(can_frame)

    def switch_duty_mode(self):
        subprocess.run(['./duty_to_can'])

    def switch_pi_mode(self):
        subprocess.run(['./pi_to_can'])

def main():
    root = tk.Tk()
    root.title('main window')
    root.geometry('')

    tab = tk.Frame(root)
    tab.pack(expand=True, fill='both', padx=10, pady=10)  # ウィンドウを大きくできる。上下左右の余白を10に設定。

    rclpy.init()
    node = MotorTest(tab)
    rclpy_executor = rclpy.executors.MultiThreadedExecutor()
    rclpy_executor.add_node(node)
    rclpy_executor_thread = threading.Thread(target=rclpy_executor.spin, daemon=True)
    rclpy_executor_thread.start()
    root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
