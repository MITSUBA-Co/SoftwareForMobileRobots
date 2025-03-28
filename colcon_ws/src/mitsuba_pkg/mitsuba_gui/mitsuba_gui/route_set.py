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
import signal
import subprocess  # terminalでのプロンプト実行に必要
import threading
import tkinter as tk
from tkinter import messagebox

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class RouteSet(Node):

    def __init__(self, tab):
        super().__init__('route_set')
        self.tab = tab
        self.namespace = self.get_namespace()
        self.marker_array = MarkerArray()
        self.cmd_vel_array = []
        self.obst_array = []
        self.proc_array = []

        self.mitsuba_launch_dir = get_package_share_directory('mitsuba_launch')
        idir = os.path.join(self.mitsuba_launch_dir, 'map')
        files = [f for f in os.listdir(idir) if os.path.isfile(os.path.join(idir, f))]
        if len(files) > 0:
            self.idir = os.path.dirname(
                os.path.realpath(os.path.join(idir, files[0]))
            )  # shareディレクトリではなく、srcディレクトリのmapフォルダを取得

        self.create_widgets()

        self.get_map_folder_list()  # 地図フォルダ一覧更新

        # 起動時はMapプレビューを表示
        self.frame_map_view.tkraise()

        self.subGoalPose = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_pose_callback, 10
        )

        self.pub_marker_array = self.create_publisher(MarkerArray, 'waypoints', 10)

    #     self.update()

    # def update(self):
    #     rclpy.spin_once(self, timeout_sec=0.0)
    #     self.tab.after(200,self.update)

    def goal_pose_callback(self, msg):  # rvizでposeを設定したときのcallback
        print(msg.pose)
        self.marker = Marker()
        self.marker.header.frame_id = '/map'
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.5  # 矢印の長さ[m]
        self.marker.scale.y = 0.2  # 矢印の幅[m]
        self.marker.scale.z = 0.1  # 矢印の高さ[m]
        self.marker.color.a = 1.0  # 矢印の透明度（0〜1、0：透明）
        self.marker.color.r = 0.0  # 矢印の色、R要素、0〜1
        self.marker.color.g = 1.0  # 矢印の色、G要素、0〜1
        self.marker.color.b = 0.0  # 矢印の色、B要素、0〜1
        self.marker.id = len(self.marker_array.markers)  # id
        self.marker.pose = msg.pose  # position,orientation
        self.marker_array.markers.append(self.marker)  # marker_arrayにappend
        # marker_arrayをパブリッシュ（rvizに表示用）
        self.pub_marker_array.publish(self.marker_array)
        self.cmd_vel_array.append(self.entry_val_vel.get())
        # 指令速度をappend
        self.obst_array.append(self.var_obst.get())  # 障害物設定をappend

        proc = str()
        for entry_var1, entry_var2, checkbox_var in self.entries_topic_string:
            if (
                checkbox_var.get() and entry_var1.get().strip()
            ):  # チェックボックスにチェックが入っている、かつ、topic_nameが空やスペースではない（strip()はスペースを削除）
                proc = proc + ',' + entry_var1.get() + '=' + entry_var2.get()
        proc = proc[1:]  # 最初のカンマを消す
        self.proc_array.append(proc)  # 他の処理の設定用procをappend

    def btn_route_pause(self):  # 一時停止挿入ボタンが押されたとき
        self.marker.type = Marker.SPHERE
        self.marker.scale.x = 0.2  # 球の直径[m]
        self.marker.scale.y = 0.2  # 球の直径[m]
        self.marker.scale.z = 0.2  # 球の直径[m]
        self.marker.color.a = 1.0  # 球の透明度（0〜1、0：透明）
        self.marker.color.r = 1.0  # 球の色、R要素、0〜1
        self.marker.color.g = 0.0  # 球の色、G要素、0〜1
        self.marker.color.b = 0.0  # 球の色、B要素、0〜1
        self.marker.id = len(self.marker_array.markers)  # id
        self.marker_array.markers.append(self.marker)  # marker_arrayにappend
        # marker_arrayをパブリッシュ（rvizに表示用）
        self.pub_marker_array.publish(self.marker_array)
        self.cmd_vel_array.append(0.0)
        # 一時停止
        self.obst_array.append(self.var_obst.get())  # 障害物設定をappend
        proc = str()
        for entry_var1, entry_var2, checkbox_var in self.entries_topic_string:
            if (
                checkbox_var.get() and entry_var1.get().strip()
            ):  # チェックボックスにチェックが入っている、かつ、topic_nameが空やスペースではない（strip()はスペースを削除）
                proc = proc + ',' + entry_var1.get() + '=' + entry_var2.get()
        proc = proc[1:]  # 最初のカンマを消す
        self.proc_array.append(proc)  # 他の処理の設定用procをappend

    def btn_route_new(self):  # 新規経路ボタンが押されたとき
        if self.listbox_map_folder.curselection():  # Map未選択時は回避
            # marker_array delete
            self.delete_marker()

            # route_setting.launchを実行する
            launch_file_path = 'launch/route_setting.launch.py'
            idir = os.path.join(self.mitsuba_launch_dir, launch_file_path)
            if self.namespace =='/':
                self.route_setting_launch_process = subprocess.Popen(
                    [
                        'ros2',
                        'launch',
                        idir,
                        'select_map_file:={}/map'.format(self.selected_map_folder_path),
                    ]
                )
            else:
                self.route_setting_launch_process = subprocess.Popen(
                    [
                        'ros2',
                        'launch',
                        idir,
                        'select_map_file:={}/map'.format(self.selected_map_folder_path),
                        'namespace:={}'.format(self.namespace),
                    ]
                )

            # 新規経路作成ボタンを無効化
            self.button_new.config(state=tk.DISABLED)
            # 経路作成終了ボタンを有効化
            self.button_close_route_set.config(state=tk.NORMAL)
            # 経路保存ボタンを有効化
            self.button_save.config(state=tk.NORMAL)
            # Mapプレビュー → 経路設定画面切り替え
            self.frame_route_set.tkraise()
            # Mapリストの操作を無効化
            self.listbox_map_folder.configure(state=tk.DISABLED)
        else:
            messagebox.showwarning(title='確認', message='地図が選択されていません')

    def delete_marker(self):
        for marker in self.marker_array.markers:
            marker.action = Marker.DELETE
        # marker_arrayをパブリッシュ（rvizに表示用）
        self.pub_marker_array.publish(self.marker_array)

        self.marker_array.markers.clear()  # Arrayクリア
        self.cmd_vel_array.clear()  # Arrayクリア
        self.obst_array.clear()  # Arrayクリア
        self.proc_array.clear()  # Arrayクリア

    def close_route_set(self):
        ret = messagebox.askokcancel(title='終了', message='経路の作成を終了しますか？')
        if ret:
            # route_setting.launch関連ノードを終了する
            self.route_setting_launch_process.send_signal(signal.SIGINT)
            exit_code = self.route_setting_launch_process.returncode

            self.route_setting_launch_process.wait(timeout=15)
            exit_code = self.route_setting_launch_process.returncode
            messagebox.showinfo(title='完了', message='終了しました')
            print(f'終了コード: {exit_code}')

            # Entryの入力をリセット
            self.txt_route_name.delete(0, tk.END)
            # 新規経路作成ボタンを有効化
            self.button_new.config(state=tk.NORMAL)
            # 経路作成終了ボタンを無効化
            self.button_close_route_set.config(state=tk.DISABLED)
            # 経路保存ボタンを無効化
            self.button_save.config(state=tk.DISABLED)
            # 経路設定画面切り替え → Mapプレビュー切り替え
            self.frame_map_view.tkraise()
            # 全widgetからフォーカスを外す
            self.tab.focus_set()
            # Mapリストの操作を有効化
            self.listbox_map_folder.configure(state=tk.NORMAL)

    def btn_route_undo(self):  # １つ戻るボタンが押されたとき
        if len(self.marker_array.markers) == 0:
            return  # waypointが1つも設定されていないときは何もしない
        # marker_arrayの最後のmarkerを非表示に設定
        self.marker_array.markers[-1].action = Marker.DELETE
        self.pub_marker_array.publish(self.marker_array)

        self.marker_array.markers.pop()  # marker_arrayの最後のmarkerを削除
        self.cmd_vel_array.pop()  # Arrayの一番最後を削除
        self.obst_array.pop()  # Arrayの一番最後を削除
        self.proc_array.pop()  # Arrayの一番最後を削除

    def btn_route_save(self):  # 経路保存ボタンが押されたとき
        if len(self.marker_array.markers) == 0:
            return  # waypointが1つも設定されていないときは保存しない
        filename = os.path.join(self.selected_map_folder_path, self.txt_route_name.get() + '.csv')
        try:
            with open(filename, mode='x') as file:  # mode=xは既に同じファイルが存在する場合は保存しない
                writer = csv.writer(file)
                for i in range(len(self.marker_array.markers)):
                    x = self.marker_array.markers[i].pose.position.x
                    y = self.marker_array.markers[i].pose.position.y
                    z = self.marker_array.markers[i].pose.position.z
                    qx = self.marker_array.markers[i].pose.orientation.x
                    qy = self.marker_array.markers[i].pose.orientation.y
                    qz = self.marker_array.markers[i].pose.orientation.z
                    qw = self.marker_array.markers[i].pose.orientation.w
                    v = self.cmd_vel_array[i]
                    obst = self.obst_array[i]
                    proc = self.proc_array[i]
                    print(proc)
                    proc_list = proc.split(',')
                    data = [
                        f'x={x}',
                        f'y={y}',
                        f'z={z}',
                        f'qx={qx}',
                        f'qy={qy}',
                        f'qz={qz}',
                        f'qw={qw}',
                        f'v={v}',
                        f'obst={obst}',
                    ] + proc_list
                    writer.writerow(data)

        except FileExistsError:
            messagebox.showerror(
                title='Error', message=str(self.txt_route_name.get()) + ' は既に存在します。'
            )

        self.get_route_file_list()  # 経路リストを更新

    def btn_route_delete(self):  # 経路削除ボタンが押されたとき
        if self.listbox_route_list.curselection():  # 経路リスト未選択時は回避
            filename = self.listbox_route_list.get(self.listbox_route_list.curselection())
            filepath = os.path.join(self.idir, self.selected_map_folder_name, filename)
            print('DelFile:', os.path.join(self.idir, self.selected_map_folder_name, filename))
            self.listbox_route_list.selection_clear(0, tk.END)  # 経路リストの選択を解除
            self.tab.focus_set()
            subprocess.run(['gio', 'trash', filepath], check=True)
            self.button_del['state'] = tk.DISABLED  # 経路削除ボタンを押せないようにする

            self.get_route_file_list()  # 経路リストを更新

    def on_root_click(self, event):  # 何もないところをクリックしたときにentryからフォーカスを外す
        event.widget.focus_set()
        self.listbox_route_list.selection_clear(0, tk.END)  # 経路リストの選択を解除
        self.button_del['state'] = tk.DISABLED  # 経路削除ボタンを押せないようにする

    def focus_out_listbox_route_list(self, event=None):
        self.button_del['state'] = tk.DISABLED  # 経路削除ボタンを押せないようにする
        self.listbox_route_list.selection_clear(0, tk.END)  # 経路リストの選択を解除

    def focus_in_listbox_route_list(self, event=None):
        if self.listbox_route_list.size() == 0:
            return  # リストが空の場合は削除ボタンを有効化しない
        self.button_del['state'] = tk.NORMAL  # 経路削除ボタンを押せる状態にする

    def create_widgets(self):
        color_route_set = '#F0FFFF'
        # タブが選択されたときにMAPフォルダの更新を行う
        self.tab.bind('<Visibility>', self.on_tab_select)
        # 何もないところをクリックした時にフォーカスをもらう（entryからフォーカスを無くす）
        self.tab.bind('<Button-1>', self.on_root_click)

        self.tab.configure(bg=color_route_set)

        # 新規経路作成 Button
        self.button_new = tk.Button(self.tab, text='経路作成起動', command=self.btn_route_new)
        self.button_new.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # 経路作成終了 Button
        self.button_close_route_set = tk.Button(
            self.tab, text='経路作成終了', state=tk.DISABLED, command=self.close_route_set
        )
        self.button_close_route_set.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        # 経路名ラベル
        label = tk.Label(self.tab, text='経路名', bg=color_route_set)
        label.grid(row=0, column=3, padx=5, pady=5, sticky=tk.E)

        # 経路名指定用 Entry
        self.txt_route_name = tk.Entry(self.tab, width=20, justify=tk.LEFT)
        self.txt_route_name.grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)

        # 経路保存ボタン
        self.button_save = tk.Button(
            self.tab, text='経路保存', state=tk.DISABLED, command=self.btn_route_save
        )
        self.button_save.grid(row=0, column=5, padx=5, pady=5, sticky=tk.W)

        # 地図フォルダラベル
        label_map_list = tk.Label(self.tab, text='Mapリスト', bg=color_route_set)
        label_map_list.grid(row=1, column=0, padx=5, pady=5, sticky=tk.SW)

        # 地図フォルダ一覧用 Frame
        self.frame_folder_list = tk.Frame(self.tab, bg=color_route_set)
        self.frame_folder_list.grid(row=2, column=0, columnspan=3, padx=5, pady=5, sticky=tk.NSEW)

        # 地図フォルダ一覧用 Scrollbar
        self.scrollbar_folder_list = tk.Scrollbar(self.frame_folder_list)
        self.scrollbar_folder_list.pack(side=tk.RIGHT, fill=tk.Y)

        # 地図フォルダ一覧用 Listbox
        self.listbox_map_folder = tk.Listbox(
            self.frame_folder_list, yscrollcommand=self.scrollbar_folder_list.set, width=40
        )
        self.listbox_map_folder.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.listbox_map_folder.bind('<<ListboxSelect>>', self.find_disp_map_images)
        self.scrollbar_folder_list.config(command=self.listbox_map_folder.yview)

        # 経路ファイルラベル
        label = tk.Label(self.tab, text='経路ファイル', bg=color_route_set)
        label.grid(row=3, column=0, padx=5, pady=5, sticky=tk.SW)

        # 経路ファイル一覧用 Frame
        self.frame_route_list = tk.Frame(self.tab, bg=color_route_set)
        self.frame_route_list.grid(row=4, column=0, columnspan=3, padx=5, pady=5, sticky=tk.NSEW)

        # 経路ファイル一覧用 Scrollbar
        self.scrollbar_route_list = tk.Scrollbar(self.frame_route_list)
        self.scrollbar_route_list.pack(side=tk.RIGHT, fill=tk.Y)

        # 経路ファイル一覧用 Listbox
        self.listbox_route_list = tk.Listbox(
            self.frame_route_list, yscrollcommand=self.scrollbar_route_list.set, width=40
        )
        self.listbox_route_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.listbox_route_list.bind('<FocusOut>', self.focus_out_listbox_route_list)
        self.listbox_route_list.bind('<FocusIn>', self.focus_in_listbox_route_list)
        self.scrollbar_route_list.config(command=self.listbox_route_list.yview)

        # 地図画像プレビュー・経路設定ウィジェット切り替え用 Frame
        self.frame_switch_map_view_route_set = tk.Frame(self.tab, bg=color_route_set)
        self.frame_switch_map_view_route_set.grid(
            row=2, rowspan=4, column=3, columnspan=3, padx=5, pady=5, sticky=tk.NSEW
        )
        # rootメインウィンドウのグリッドを 1x1 にする
        self.frame_switch_map_view_route_set.grid_rowconfigure(0, weight=1)
        self.frame_switch_map_view_route_set.grid_columnconfigure(0, weight=1)

        # 地図画像プレビュー用 LabelFrame
        self.frame_map_view = tk.LabelFrame(
            self.frame_switch_map_view_route_set,
            text='Mapプレビュー',
            bd=1,
            relief='solid',
            bg=color_route_set,
        )
        self.frame_map_view.grid(row=0, column=0, padx=5, pady=5, sticky=tk.NSEW)

        # 地図画像プレビュー用 Canvas
        self.canvas_map_preview = tk.Canvas(self.frame_map_view, width=500, height=500)
        self.canvas_map_preview.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # 経路設定用 LabelFrame
        self.frame_route_set = tk.LabelFrame(
            self.frame_switch_map_view_route_set,
            text='経路設定',
            bd=1,
            relief='solid',
            bg=color_route_set,
        )
        self.frame_route_set.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=tk.NSEW)

        # 経路削除ボタン
        self.button_del = tk.Button(
            self.tab, text='経路削除', state=tk.DISABLED, command=self.btn_route_delete
        )
        self.button_del.grid(row=5, column=2, padx=10)

        # 1つ戻るボタン
        button_undo = tk.Button(self.frame_route_set, text='１つ戻る', command=self.btn_route_undo)
        button_undo.grid(row=2, column=2, padx=5, pady=5, sticky=tk.W)

        # 全経路ポイントクリアボタン
        button_clear = tk.Button(self.frame_route_set, text='全ポイントクリア', command=self.delete_marker)
        button_clear.grid(row=2, column=7, padx=5, pady=5, sticky=tk.E)

        # 一時停止挿入ボタン
        button_pause = tk.Button(self.frame_route_set, text='一時停止挿入', command=self.btn_route_pause)
        button_pause.grid(row=3, column=2, padx=5, pady=5, sticky=tk.W)

        # 走行速度設定ラベル
        label_frame_vel = tk.LabelFrame(
            self.frame_route_set, text='走行速度設定', bd=1, relief='solid', bg=color_route_set
        )
        label_frame_vel.grid(row=4, column=2, padx=5, pady=5, sticky=tk.W)

        # 速度設定エントリー
        self.entry_val_vel = tk.DoubleVar(value=1.666)
        entry_vel = tk.Entry(
            label_frame_vel,
            textvariable=self.entry_val_vel,
            width=10,
            highlightbackground=color_route_set,
        )
        entry_vel.pack()

        # 障害物設定ラベル
        label_frame_obst = tk.LabelFrame(
            self.frame_route_set, text='障害物設定', bd=1, relief='solid', bg=color_route_set
        )
        label_frame_obst.grid(row=4, column=3, padx=5, pady=5, sticky=tk.W)

        # 障害物設定ラジオボタン
        self.var_obst = tk.StringVar(value='avoid')
        radio1 = tk.Radiobutton(
            label_frame_obst,
            value='avoid',
            variable=self.var_obst,
            text='回避',
            bg=color_route_set,
            highlightbackground=color_route_set,
        )
        radio2 = tk.Radiobutton(
            label_frame_obst,
            value='stop',
            variable=self.var_obst,
            text='停止',
            bg=color_route_set,
            highlightbackground=color_route_set,
        )
        radio1.grid(row=0, column=0)
        radio2.grid(row=0, column=1)

        # topic-string設定ラベルフレーム
        label_frame_proc = tk.LabelFrame(
            self.frame_route_set, text='topic-string設定', bd=1, relief='solid', bg=color_route_set
        )
        label_frame_proc.grid(row=5, column=2, columnspan=5, padx=5, pady=5, sticky=tk.W)

        self.entries_topic_string = []  # topic-string設定エントリ値とチェックボックスの値
        for i in range(6):  # topic-string設定エントリとチェックボックス
            checkbox_var = tk.BooleanVar()
            entry_var1 = tk.StringVar()
            entry_var2 = tk.StringVar()

            checkbox = tk.Checkbutton(
                label_frame_proc,
                variable=checkbox_var,
                bg=color_route_set,
                highlightbackground=color_route_set,
            )
            entry_topic = tk.Entry(
                label_frame_proc,
                textvariable=entry_var1,
                width=15,
                highlightbackground=color_route_set,
            )
            entry_string = tk.Entry(
                label_frame_proc,
                textvariable=entry_var2,
                width=15,
                highlightbackground=color_route_set,
            )

            self.entries_topic_string.append((entry_var1, entry_var2, checkbox_var))

            checkbox.grid(row=i, column=0)
            entry_topic.grid(row=i, column=1)
            entry_string.grid(row=i, column=2)

    def find_disp_map_images(self, event=None):
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
                # self.listbox_map_folder.itemconfigure(selected_index, background='Silver')
                # 標準のカーソルと同色

            self.selected_map_folder_name = self.listbox_map_folder.get(
                self.listbox_map_folder.curselection()
            )  # 現在選択されているMapフォルダ名の更新（ここ以外で更新しない）
            self.selected_map_folder_path = os.path.join(
                self.idir, self.selected_map_folder_name
            )  # 現在選択されているMapフォルダパスの更新（ここ以外で更新しない）
            image_files = [
                f for f in os.listdir(self.selected_map_folder_path) if f.endswith('.pgm')
            ]
            if image_files:
                image_path = os.path.join(self.selected_map_folder_path, image_files[0])
                image = tk.PhotoImage(file=image_path)
                img_width, img_height = image.width(), image.height()
                # print(img_width, img_height)
                # print(self.canvas_map_preview.winfo_width(),
                #     self.canvas_map_preview.winfo_height())
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
                    # print('image_ratio', image_ratio)
                    img_width = int(img_width * image_ratio)
                    img_height = int(img_height * image_ratio)
                    image = image.subsample(int(1 / image_ratio))
                    # print('min img_resize', img_width, img_height)
                else:
                    # 画像がCanvasよりも小さい場合は、Canvasの幅または高さに合わせて画像サイズを拡大
                    image_ratio = min(
                        self.canvas_map_preview.winfo_width() / img_width,
                        self.canvas_map_preview.winfo_height() / img_height,
                    )
                    # print('image_ratio', image_ratio)
                    img_width = int(img_width * image_ratio)
                    img_height = int(img_height * image_ratio)
                    image = image.zoom(int(image_ratio))
                    # print('max img_resize', img_width, img_height)

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
        for folder in folders:
            self.listbox_map_folder.insert(tk.END, folder)

    def get_route_file_list(self):
        #        if self.listbox_map_folder.curselection(): # リスト未選択時は回避
        files = [
            file for file in os.listdir(self.selected_map_folder_path) if file.endswith('.csv')
        ]
        files.sort()
        self.listbox_route_list.delete(0, tk.END)
        # リストボックスにファイル一覧を表示
        for file in files:
            self.listbox_route_list.insert(tk.END, file)


def main():
    root = tk.Tk()
    root.title('main window')
    root.geometry('')

    tab = tk.Frame(root)
    # ウィンドウを大きくできる。上下左右の余白を10に設定。
    tab.pack(expand=True, fill='both', padx=10, pady=10)

    rclpy.init()
    node = RouteSet(tab)
    rclpy_executor = rclpy.executors.MultiThreadedExecutor()
    rclpy_executor.add_node(node)
    rclpy_executor_thread = threading.Thread(target=rclpy_executor.spin, daemon=True)
    rclpy_executor_thread.start()
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
