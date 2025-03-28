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

import os
import sys
import threading
import tkinter as tk
import tkinter.ttk as ttk

import rclpy
from ament_index_python.packages import get_package_share_directory

from mitsuba_gui import (
    autonom_run,
    map_save,
    motor_test,
    param_set,
    route_set,
    sdf_param_set,
)


def main():
    args = sys.argv[1:]  # コマンドライン引数の取得、引数なしの場合は[]になる
    if len(args) == 0:
        sim = False  # 引数が空配列の場合
    else:
        if args[0] == 'sim':
            sim = True  # 引数が'sim'の場合
        else:
            sim = False  # 引数が'sim'以外の場合 launchファイルで起動すると引数が'--ros-args'に設定される

    root = tk.Tk()
    root.title('main window')
    root.geometry('')

    notebook = ttk.Notebook(root)
    notebook.pack(expand=True, fill='both', padx=10, pady=10)

    if not sim:
        tab1 = tk.Frame(notebook, padx=10, pady=10)  # モータテストタブ(引数が設定されていない場合は実験用)
    tab2 = tk.Frame(notebook, padx=0, pady=0)  # パラメータ設定タブ（サブタブがあるので、padx,padyは0に設定）
    tab3 = tk.Frame(notebook, padx=10, pady=10)  # 地図作成タブ
    tab4 = tk.Frame(notebook, padx=10, pady=10)  # 経路作成タブ
    tab5 = tk.Frame(notebook, padx=10, pady=10)  # 自律走行タブ

    if not sim:
        notebook.add(tab1, text='モード切替え')  # 引数が設定されていない場合は実験用
    notebook.add(tab2, text='パラメータ設定')
    notebook.add(tab3, text='地図作成')
    notebook.add(tab4, text='経路作成')
    notebook.add(tab5, text='自律走行')

    rclpy.init()
    rclpy_executor = rclpy.executors.MultiThreadedExecutor()

    style = ttk.Style()
    style.configure('Horizontal.TNotebook', tabposition='wn')
    inner_notebook = ttk.Notebook(tab2, style='Horizontal.TNotebook')
    inner_notebook.pack(expand=True, fill='both')

    param_setting_files = sorted(
        os.listdir(get_package_share_directory('mitsuba_gui') + '/paramsetting')
    )  # フォルダ内のファイルリストを取得し、ファイル名でソート
    for param_setting_file in param_setting_files:
        inner_frame = ttk.Frame(inner_notebook)
        name = os.path.splitext(param_setting_file)[0]  # ファイル名の拡張子を除く
        inner_notebook.add(inner_frame, text=name)  # ファイル名の拡張子を除いた名前の縦タブを作成
        param_set.ParamSet(inner_frame, node_name=name)  # ファイル名の拡張子を除いた名前のノードを作成

    inner_frame = ttk.Frame(inner_notebook)
    inner_notebook.add(inner_frame, text='sensor mount')  # センサ取付位置の縦タブを作成
    sdf_param_set.Toplevel1(inner_frame)  # タブページを作成

    # タブを切り替えたときにnotebookにフォーカスを移動（これをしないとタブの中のEntryやbuttonをフォーカスしてしまう）
    notebook.bind('<<NotebookTabChanged>>', lambda event: event.widget.focus())
    # タブを切り替えたときにnotebookにフォーカスを移動（これをしないとタブの中のEntryやbuttonをフォーカスしてしまう）
    inner_notebook.bind('<<NotebookTabChanged>>', lambda event: event.widget.focus())

    if not sim:
        node1 = motor_test.MotorTest(tab1)  # 引数が設定されていない場合は実験用
    node3 = map_save.MapSave(tab3)
    node4 = route_set.RouteSet(tab4)
    if not sim:  # 引数が設定されていない場合は実験用
        node5 = autonom_run.AutonomRun(tab5, use_sim_time=False)
    else:  # 引数が設定されている場合はシミュレーション用
        node5 = autonom_run.AutonomRun(tab5, use_sim_time=True)

    if not sim:
        rclpy_executor.add_node(node1)
    rclpy_executor.add_node(node3)
    rclpy_executor.add_node(node4)
    rclpy_executor.add_node(node5)
    rclpy_executor_thread = threading.Thread(target=rclpy_executor.spin, daemon=True)
    rclpy_executor_thread.start()

    root.mainloop()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
