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
import signal
import subprocess  # terminalでのプロンプト実行に必要
import sys
import threading
import tkinter as tk
from tkinter import messagebox

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class AutonomRun(Node):

    def __init__(self, tab, use_sim_time=False, multi_sim=False):
        super().__init__('autonom_run')
        self.get_logger().info(f'use_sim_time {use_sim_time}')
        self.tab = tab
        self.use_sim_time = use_sim_time
        self.multi_sim = multi_sim
        self.namespace = self.get_namespace()
        self.get_logger().info(f'namespace {self.namespace}')
        self.mitsuba_launch_dir = get_package_share_directory('mitsuba_launch')
        idir = os.path.join(self.mitsuba_launch_dir, 'map')
        #        files = os.listdir(idir)
        files = [f for f in os.listdir(idir) if os.path.isfile(os.path.join(idir, f))]
        if len(files) > 0:
            # shareディレクトリではなく、srcディレクトリのmapフォルダを取得
            self.idir = os.path.dirname(os.path.realpath(os.path.join(idir, files[0])))
        self.pub_odom_reset = self.create_publisher(String, 'mitsuba/odom_reset', 10)
        self.pub_initial_pose = self.create_publisher(String, 'mitsuba/route_initialpose', 10)
        self.pub_load = self.create_publisher(String, 'mitsuba/route_load', 10)
        self.pub_start = self.create_publisher(String, 'mitsuba/route_start', 10)
        self.pub_restart = self.create_publisher(String, 'mitsuba/route_restart', 10)
        self.pub_cancel = self.create_publisher(String, 'mitsuba/route_cancel', 10)

        self.create_widgets()  # ヴィジットの作成
        self.get_map_folder_list()  # 地図フォルダ一覧更新
        self.frame_map_view.tkraise()  # 起動時はMapプレビューを表示

        # self.update()

    # def update(self):
    #     rclpy.spin_once(self, timeout_sec=0.0)
    #     self.tab.after(200, self.update)

    def btn_launch_slamtoolbox(self):  # 起動ボタンが押されたとき
        if self.listbox_map_folder.curselection():  # Map未選択時は回避
            # auto_run_slamtoolbox.launchを呼ぶ
            launch_file_path = 'launch/auto_run_slamtoolbox.launch.py'
            idir = os.path.join(self.mitsuba_launch_dir, launch_file_path)
            mitsuba_sim_dir = get_package_share_directory('mitsuba_sim')
            namespace = self.namespace[1:]    #先頭の『/』を取る
            if self.multi_sim:
                params_file = os.path.join(self.mitsuba_launch_dir, 'yaml',
                                           namespace, 'nav2_params.yaml')
                sdf_file = os.path.join(mitsuba_sim_dir, 'sdf', 'models', 
                                        namespace, 'robot_model.sdf')
                print('namespace:', namespace)
                print('params_file:', params_file)
            else:
                params_file = os.path.join(self.mitsuba_launch_dir, 'yaml', 'nav2_params.yaml')
                sdf_file = os.path.join(mitsuba_sim_dir, 'sdf', 'models', 'robot_model.sdf')

            if self.namespace =='/':
                self.auto_run_slamtoolbox_launch_process = subprocess.Popen(
                    [
                        'ros2',
                        'launch',
                        idir,
                        'select_map_file:={}/map'.format(self.selected_map_folder_path),
                        'use_sim_time:={}'.format(self.use_sim_time),
                        'params_file:={}'.format(params_file),
                        'sdf_file:={}'.format(sdf_file),                        
                    ]
                )
            else:
                self.auto_run_slamtoolbox_launch_process = subprocess.Popen(
                    [
                        'ros2',
                        'launch',
                        idir,
                        'select_map_file:={}/map'.format(self.selected_map_folder_path),
                        'use_sim_time:={}'.format(self.use_sim_time),
                        'namespace:={}'.format(self.namespace),
                        'params_file:={}'.format(params_file),
                        'sdf_file:={}'.format(sdf_file),   
                    ]
                )
            # mitsuba_diff_driveパッケージのcan_to_odoのオドメトリをリセットする
            self.pub_odom_reset.publish(String())
            # 起動ボタンを無効化
            self.button_launch_slamtoolbox.config(state=tk.DISABLED)
            # 自律走行終了ボタンを有効化
            self.button_close_autonom_run.config(state=tk.NORMAL)
            # Mapプレビュー → 自律走行操作画面切り替え
            self.frame_autonom_set.tkraise()
            # Mapリストの操作を無効化
            self.listbox_map_folder.configure(state=tk.DISABLED)

        else:
            messagebox.showwarning(title='確認', message='地図が選択されていません')

    def btn_route_initialpose(self):  # 初期位置設定ボタンが押されたとき
        self.pub_initial_pose.publish(String())

    def btn_route_start(self):  # 自律走行開始ボタンが押されたとき
        msg = String()
        if self.checkbox_var.get():
            msg.data = self.entry_var_loop_cnt.get()
        self.pub_start.publish(msg)
        self.button_start['state'] = tk.DISABLED  # 自律走行開始ボタンを押せない状態にする

    def btn_route_restart(self):  # 一時停止解除ボタンが押されたとき
        self.pub_restart.publish(String())

    def btn_route_cancel(self):  # 自律走行キャンセルボタンが押されたとき
        self.pub_cancel.publish(String())

    #        self.buttonCancel['state'] = tk.DISABLED     # 自律走行キャンセルボタンを押せない状態にする

    def focus_out_listbox_route_list(self, event=None):
        self.button_start['state'] = tk.DISABLED  # 自律走行開始ボタンを押せない状態にする
        self.listbox_route_list.selection_clear(0, tk.END)  # 経路リストの選択を解除

    def focus_in_listbox_route_list(self, event=None):  # 自律走行用経路がリスト上で選択された場合
        # 経路リストが空、未選択、自律走行launch起動前の場合は回避
        if (
            self.listbox_route_list.size() == 0
            or len(self.listbox_route_list.curselection()) < 1
            or self.listbox_map_folder.cget('state') == tk.NORMAL
        ):
            return
        filename = self.listbox_route_list.get(self.listbox_route_list.curselection())
        filepath = String(
            data=str(os.path.join(self.idir, self.selected_map_folder_name, filename))
        )
        self.pub_load.publish(filepath)
        self.button_start['state'] = tk.NORMAL  # 自律走行開始ボタンを押せる状態にする

    def validate_integer(self, text):  # entryに整数のみの入力制限を設定するための関数
        if text.isdigit() or text == '':
            return True
        else:
            return False

    def toggle_entry_state(self):
        if self.checkbox_var.get() == 1:
            self.entry_loop.config(state='normal')
        else:
            self.entry_loop.config(state='disabled')

    def close_autonom_run(self):
        ret = messagebox.askokcancel(title='終了', message='自律走行を終了しますか？')
        if ret:
            # auto_run_slamtoolbox.launch関連ノードを終了する
            output = subprocess.check_output(['ros2', 'lifecycle', 'nodes']).decode()
            lifecycle_nodes = output.splitlines()
            for lifecycle_node in lifecycle_nodes:
                subprocess.call(['ros2', 'lifecycle', 'set', lifecycle_node, 'shutdown'])
            self.auto_run_slamtoolbox_launch_process.send_signal(signal.SIGINT)
            self.auto_run_slamtoolbox_launch_process.wait()
            exit_code = self.auto_run_slamtoolbox_launch_process.returncode
            print(f'終了コード: {exit_code}')

            # 起動ボタンを有効化
            self.button_launch_slamtoolbox.config(state=tk.NORMAL)
            # 自律走行終了ボタンを無効化
            self.button_close_autonom_run.config(state=tk.DISABLED)
            # 自律走行操作画面切り替え → Mapプレビュー切り替え
            self.frame_map_view.tkraise()
            # Mapリストの操作を有効化
            self.listbox_map_folder.configure(state=tk.NORMAL)

            messagebox.showinfo(title='完了', message='終了しました')

    def select_listbox_map_list(self, event=None):
        if self.listbox_map_folder.curselection():  # リスト未選択時は回避
            # 地図選択後に経路ファイル選択で地図リストのカーソルが消えるため色指定
            # 背景色をリセットする
            for i in range(self.listbox_map_folder.size()):
                self.listbox_map_folder.itemconfigure(i, background='white')

            # 選択された要素の背景色を変更する
            selected_index = self.listbox_map_folder.curselection()
            if selected_index:
                self.listbox_map_folder.itemconfigure(
                    selected_index, background='LightGoldenrodYellow'
                )

            self.selected_map_folder_name = self.listbox_map_folder.get(
                self.listbox_map_folder.curselection()
            )  # 現在選択されているMapフォルダ名の更新（ここ以外で更新しない）
            # 現在選択されているMapフォルダパスの更新（ここ以外で更新しない）
            self.selected_map_folder_path = os.path.join(self.idir, self.selected_map_folder_name)

            image_files = [
                f for f in os.listdir(self.selected_map_folder_path) if f.endswith('.pgm')
            ]
            if image_files:
                image_path = os.path.join(self.selected_map_folder_path, image_files[0])
                image = tk.PhotoImage(file=image_path)
                img_width, img_height = image.width(), image.height()

                # 画像の縦横比を維持しつつ、Canvasのサイズに合わせる
                if (
                    self.canvas_map_preview.winfo_width() < img_width
                    or self.canvas_map_preview.winfo_height() < img_height
                ):
                    # 画像がCanvasよりも大きい場合は、Canvasの幅または高さに合わせて画像サイズを縮小
                    image_ratio = min(
                        self.canvas_map_preview.winfo_width() / img_width,
                        self.canvas_map_preview.winfo_height() / img_height,
                    )
                    img_width = int(img_width * image_ratio)
                    img_height = int(img_height * image_ratio)
                    image = image.subsample(int(1 / image_ratio))
                else:
                    # 画像がCanvasよりも小さい場合は、Canvasの幅または高さに合わせて画像サイズを拡大
                    image_ratio = min(
                        self.canvas_map_preview.winfo_width() / img_width,
                        self.canvas_map_preview.winfo_height() / img_height,
                    )
                    img_width = int(img_width * image_ratio)
                    img_height = int(img_height * image_ratio)
                    image = image.zoom(int(image_ratio))

                self.canvas_map_preview.delete('all')
                self.canvas_map_preview.create_image(
                    self.canvas_map_preview.winfo_width() / 2,
                    self.canvas_map_preview.winfo_height() / 2,
                    anchor='c',
                    image=image,
                )
                self.canvas_map_preview.image = image
            else:
                # メッセージを表示する
                self.canvas_map_preview.delete('all')
                self.canvas_map_preview.create_text(
                    self.canvas_map_preview.winfo_width() / 2,
                    self.canvas_map_preview.winfo_height() / 2,
                    text='プレビュー可能なファイルが存在しません。',
                )

            self.get_route_file_list()  # 選択されたMapフォルダ内の経路ファイルをリストに表示

    def on_tab_select(self, event):  # フォルダリストを取得する
        self.get_map_folder_list()

    def get_map_folder_list(self):
        folders = [f.name for f in os.scandir(self.idir) if f.is_dir()]
        folders.sort()
        self.listbox_map_folder.delete(0, tk.END)
        # リストボックスにフォルダ一覧を表示
        for folder in folders:
            self.listbox_map_folder.insert(tk.END, folder)

    def get_route_file_list(self):
        files = [
            file for file in os.listdir(self.selected_map_folder_path) if file.endswith('.csv')
        ]
        files.sort()
        self.listbox_route_list.delete(0, tk.END)
        # リストボックスにファイル一覧を表示
        for file in files:
            self.listbox_route_list.insert(tk.END, file)

    # def on_root_click(self,event):      #何もないところをクリックしたときにentryからフォーカスを外す
    # event.widget.focus_set()
    # self.buttonsRoute[self.iSelect]['relief'] = tk.RAISED       #ボタンが押されてない状態にする
    # self.button_start['state'] = tk.DISABLED                    #経路削除ボタンを押せないようにする

    def create_widgets(self):
        self.tab.bind('<Visibility>', self.on_tab_select)  # タブが選択されたときにMAPフォルダの更新を行う
        # 何もないところをクリックした時にフォーカスをもらう（entryからフォーカスを無くす）
        # self.tab.bind('<Button-1>', self.on_root_click)
        color_autonom_run = '#FFDAB9'
        self.tab.configure(bg=color_autonom_run)

        # 自律走行起動 Button
        self.button_launch_slamtoolbox = tk.Button(
            self.tab, text='自律走行起動', command=self.btn_launch_slamtoolbox
        )
        self.button_launch_slamtoolbox.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # 自律走行終了 Button
        self.button_close_autonom_run = tk.Button(
            self.tab, text='自律走行終了', state=tk.DISABLED, command=self.close_autonom_run
        )
        # self.button_close_autonom_run.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        # 空白のフレーム（列幅100）
        spacer = tk.Frame(self.tab, width=100, height=0, bg=color_autonom_run)
        spacer.grid(row=0, column=2)

        # 地図フォルダラベル
        label_map_list = tk.Label(self.tab, text='Mapリスト', bg=color_autonom_run)
        label_map_list.grid(row=1, column=0, padx=5, pady=5, sticky=tk.SW)

        # 地図フォルダ一覧用 Frame
        self.frame_folder_list = tk.Frame(self.tab, bg=color_autonom_run)
        self.frame_folder_list.grid(row=2, column=0, columnspan=3, padx=5, pady=5, sticky=tk.NSEW)

        # 地図フォルダ一覧用 Scrollbar
        self.scrollbar_folder_list = tk.Scrollbar(self.frame_folder_list)
        self.scrollbar_folder_list.pack(side=tk.RIGHT, fill=tk.Y)

        # 地図フォルダ一覧用 Listbox
        self.listbox_map_folder = tk.Listbox(
            self.frame_folder_list, yscrollcommand=self.scrollbar_folder_list.set, width=40
        )
        self.listbox_map_folder.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.listbox_map_folder.bind('<<ListboxSelect>>', self.select_listbox_map_list)
        self.scrollbar_folder_list.config(command=self.listbox_map_folder.yview)

        # 経路ファイルラベル
        label = tk.Label(self.tab, text='経路ファイル', bg=color_autonom_run)
        label.grid(row=3, column=0, padx=5, pady=5, sticky=tk.SW)

        # 経路ファイル一覧用 Frame
        self.frame_route_list = tk.Frame(self.tab, bg=color_autonom_run)
        self.frame_route_list.grid(row=4, column=0, columnspan=3, padx=5, pady=5, sticky=tk.NSEW)

        # 経路ファイル一覧用 Scrollbar
        self.y_scrollbar_route_list = tk.Scrollbar(self.frame_route_list, orient='vertical')
        self.y_scrollbar_route_list.pack(side=tk.RIGHT, fill=tk.Y)
        # self.x_scrollbar_RouteList = tk.Scrollbar(self.frame_route_list, orient='horizontal')
        # self.x_scrollbar_RouteList.pack(side=tk.BOTTOM, fill=tk.X)

        # 経路ファイル一覧用 Listbox
        self.listbox_route_list = tk.Listbox(
            self.frame_route_list, yscrollcommand=self.y_scrollbar_route_list.set, width=40
        )
        # self.listbox_route_list = tk.Listbox(
        #     self.frame_route_list,
        #     yscrollcommand=self.y_scrollbar_route_list.set,
        #     xscrollcommand=self.x_scrollbar_RouteList.set,
        #     width=40,
        # )
        self.listbox_route_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.listbox_route_list.bind('<FocusOut>', self.focus_out_listbox_route_list)
        self.listbox_route_list.bind('<<ListboxSelect>>', self.focus_in_listbox_route_list)
        self.y_scrollbar_route_list.config(command=self.listbox_route_list.yview)
        # self.x_scrollbar_RouteList.config(command=self.listbox_route_list.xview)

        # 地図画像プレビュー・経路設定ウィジェット切り替え用 Frame
        self.frame_switch_mapview_route_set = tk.Frame(self.tab, bg=color_autonom_run)
        self.frame_switch_mapview_route_set.grid(
            row=2, rowspan=4, column=3, padx=5, pady=5, sticky=tk.NSEW
        )
        # rootメインウィンドウのグリッドを 1x1 にする
        self.frame_switch_mapview_route_set.grid_rowconfigure(0, weight=1)
        self.frame_switch_mapview_route_set.grid_columnconfigure(0, weight=1)

        # 地図画像プレビュー用 LabelFrame
        self.frame_map_view = tk.LabelFrame(
            self.frame_switch_mapview_route_set,
            text='Mapプレビュー',
            bd=1,
            relief='solid',
            bg=color_autonom_run,
        )
        self.frame_map_view.grid(row=0, column=0, padx=5, pady=5, sticky=tk.NSEW)

        # 地図画像プレビュー用 Canvas
        self.canvas_map_preview = tk.Canvas(self.frame_map_view, width=500, height=500)
        self.canvas_map_preview.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # 自律走行操作用 LabelFrame
        self.frame_autonom_set = tk.LabelFrame(
            self.frame_switch_mapview_route_set,
            text='自律走行設定',
            bd=1,
            relief='solid',
            bg=color_autonom_run,
        )
        self.frame_autonom_set.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=tk.NSEW)

        self.button_initialpose = tk.Button(
            self.frame_autonom_set, text='初期位置設定', command=self.btn_route_initialpose
        )
        # self.button_initialpose.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # ループ走行ラベルフレーム
        label_frame_loop = tk.LabelFrame(
            self.frame_autonom_set, text='ループ走行', bd=1, relief='solid', bg=color_autonom_run
        )
        label_frame_loop.grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)

        self.checkbox_var = tk.BooleanVar()
        checkbox_loop = tk.Checkbutton(
            label_frame_loop,
            variable=self.checkbox_var,
            command=self.toggle_entry_state,
            bg=color_autonom_run,
            highlightbackground=color_autonom_run,
        )
        self.entry_var_loop_cnt = tk.StringVar(value='0')
        validate_cmd = (
            label_frame_loop.register(self.validate_integer),
            '%P',
        )  # entryに整数のみの入力制限を設定する
        self.entry_loop = tk.Entry(
            label_frame_loop,
            state='disabled',
            textvariable=self.entry_var_loop_cnt,
            width=8,
            justify='right',
            validate='key',
            validatecommand=validate_cmd,
            highlightbackground=color_autonom_run,
        )
        label_loop1 = tk.Label(label_frame_loop, text='回', bg=color_autonom_run)
        label_loop2 = tk.Label(label_frame_loop, text='0回:無限ループ', bg=color_autonom_run)

        checkbox_loop.grid(row=0, column=0)
        self.entry_loop.grid(row=0, column=1)
        label_loop1.grid(row=0, column=2)
        label_loop2.grid(row=1, column=1)

        self.button_start = tk.Button(
            self.frame_autonom_set, text='自律走行開始', state=tk.DISABLED, command=self.btn_route_start
        )
        self.button_start.grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)

        self.buttonRestart = tk.Button(
            self.frame_autonom_set, text='一時停止解除', state=tk.NORMAL, command=self.btn_route_restart
        )
        self.buttonRestart.grid(row=2, column=1, padx=5, pady=5, sticky=tk.W)

        self.buttonCancel = tk.Button(
            self.frame_autonom_set, text='自律走行キャンセル', state=tk.NORMAL, command=self.btn_route_cancel
        )
        self.buttonCancel.grid(row=3, column=0, padx=5, pady=5, sticky=tk.W)


def main():
    args = sys.argv[1:]  # コマンドライン引数の取得、引数なしの場合は[]になる
    print(args)
    if len(args) == 0:
        mode = 'experiment'  # 引数が空配列の場合
    else:
        if args[0] == 'sim':
            mode = 'sim'  # 引数が'sim'の場合
        elif args[0] == 'multi_sim':
            mode = 'multi_sim'
        else:
            mode = 'experiment'  # 引数が'sim'以外の場合 launchファイルで起動すると引数が'--ros-args'に設定される

    root = tk.Tk()
    root.title('main window')
    root.geometry('')

    tab = tk.Frame(root)
    tab.pack(expand=True, fill='both', padx=10, pady=10)

    rclpy.init()
    if mode == 'experiment':  # 引数が設定されていない場合は実験用
        node = AutonomRun(tab, use_sim_time=False, multi_sim=False)
    elif mode == 'sim':  # 引数が設定されている場合はシミュレーション用
        node = AutonomRun(tab, use_sim_time=True, multi_sim=False)
    elif mode == 'multi_sim':
        node = AutonomRun(tab, use_sim_time=True, multi_sim=True)

    rclpy_executor = rclpy.executors.MultiThreadedExecutor()
    rclpy_executor.add_node(node)
    rclpy_executor_thread = threading.Thread(target=rclpy_executor.spin, daemon=True)
    rclpy_executor_thread.start()

    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
