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
import tkinter as tk
from tkinter import messagebox

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String


class MapSave(Node):

    def __init__(self, tab):
        super().__init__('map_save')
        self.tab = tab
        self.namespace = self.get_namespace()
        self.pub_odom_reset = self.create_publisher(String, 'mitsuba/odom_reset', 10)
        self.client_save = self.create_client(SaveMap, 'slam_toolbox/save_map')
        self.client_serialize = self.create_client(
            SerializePoseGraph, 'slam_toolbox/serialize_map'
        )
        self.create_widgets()  # ヴィジットの作成

        self.mitsuba_launch_dir = get_package_share_directory('mitsuba_launch')
        idir = os.path.join(self.mitsuba_launch_dir, 'map')
        files = [f for f in os.listdir(idir) if os.path.isfile(os.path.join(idir, f))]

        print('files:', files)  # mapフォルダ内のファイル/フォルダを表示
        if len(files) > 0:
            # shareディレクトリではなく、srcディレクトリのmapフォルダを取得
            self.idir = os.path.dirname(os.path.realpath(os.path.join(idir, files[0])))
        self.map_file = os.path.join(self.idir, 'map')
        print('self.idir:', self.idir)
        print('self.map_file:', self.map_file)

        self.get_map_folder_list()  # 地図フォルダ一覧更新

        # self.update()

    # def update(self):
    #     rclpy.spin_once(self, timeout_sec=0.0)
    #     self.tab.after(200, self.update)

    def btn_map_save(self):  # 地図保存ボタンが押されたとき
        req_save = SaveMap.Request()
        req_save.name = String(data=self.map_file)
        self.client_save.call_async(req_save)
        req_serialize = SerializePoseGraph.Request()
        req_serialize.filename = self.map_file
        self.client_serialize.call_async(req_serialize)

    def on_root_click(self, event):  # 何もないところをクリックしたときにentryからフォーカスを外す
        event.widget.focus_set()

    def create_widgets(self):
        color_map_save = '#FFFACD'

        self.tab.configure(bg=color_map_save)

        # main Frame
        self.main_frame = tk.Frame(self.tab, bg=color_map_save)
        self.main_frame.pack(expand=True, fill='both', padx=0, pady=0)

        # 地図作成起動 Button
        self.button_create_map = tk.Button(
            self.main_frame, text='地図作成起動', command=self.start_create_map
        )
        self.button_create_map.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # 地図作成終了 Button
        self.button_close_create_map = tk.Button(
            self.main_frame,
            text='地図作成終了',
            state=tk.DISABLED,
            command=self.close_create_map,
        )
        self.button_close_create_map.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        # フォルダ名指定 Label
        self.label_folder_name = tk.Label(self.main_frame, text='保存フォルダ名', bg=color_map_save)
        self.label_folder_name.grid(row=0, column=3, padx=5, pady=5, sticky=tk.E)

        # フォルダ名指定用 Entry
        self.entry_save_folder_name = tk.Entry(self.main_frame, width=20, justify=tk.LEFT)
        self.entry_save_folder_name.grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)

        # 地図保存 Button
        # 何もないところをクリックした時にフォーカスをもらう（entryからフォーカスを無くす）
        self.main_frame.bind('<Button-1>', self.on_root_click)
        self.button_save_map = tk.Button(
            self.main_frame, text='地図保存', state=tk.DISABLED, command=self.create_folder
        )
        self.button_save_map.grid(row=0, column=5, padx=5, pady=5, sticky=tk.W)

        # 地図フォルダ一覧 Label
        self.label_folder_name = tk.Label(self.main_frame, text='Mapリスト', bg=color_map_save)
        self.label_folder_name.grid(row=1, column=0, padx=5, pady=5, sticky=tk.SW)

        # 地図削除 Button
        self.button_delete_map = tk.Button(
            self.main_frame, text='地図削除', command=self.delete_map_folder
        )
        self.button_delete_map.grid(row=9, column=2, padx=5, pady=5, sticky=tk.W)

        # 地図フォルダ一覧用 Frame
        self.frame_folder_list = tk.Frame(self.main_frame, bg=color_map_save)
        self.frame_folder_list.grid(
            row=2, rowspan=2, column=0, columnspan=3, padx=5, pady=5, sticky=tk.NSEW
        )

        # 地図フォルダ一覧用 Scrollbar
        self.scrollbar = tk.Scrollbar(self.frame_folder_list)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 地図フォルダ一覧用 Listbox
        self.listbox_map_folder = tk.Listbox(
            self.frame_folder_list, yscrollcommand=self.scrollbar.set, width=40
        )
        self.listbox_map_folder.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.listbox_map_folder.bind('<<ListboxSelect>>', self.find_disp_map_images)
        self.scrollbar.config(command=self.listbox_map_folder.yview)

        # 地図画像プレビュー用 LabelFrame
        self.frame_map_view = tk.LabelFrame(
            self.main_frame, text='Mapプレビュー', bd=1, relief='solid', bg=color_map_save
        )
        self.frame_map_view.grid(row=2, column=3, columnspan=3, padx=5, pady=5, sticky=tk.NSEW)

        # 地図画像プレビュー用 Canvas
        self.canvas_map_preview = tk.Canvas(self.frame_map_view, width=500, height=500)
        self.canvas_map_preview.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

    def start_create_map(self):
        # 地図作成起動ボタンを無効化
        self.button_create_map.config(state=tk.DISABLED)
        # 地図作成終了ボタンを有効化
        self.button_close_create_map.config(state=tk.NORMAL)
        # 地図保存ボタンを有効化
        self.button_save_map.config(state=tk.NORMAL)

        # create_map.launchを実行する
        launch_file_path = 'launch/create_map.launch.py'
        idir = os.path.join(self.mitsuba_launch_dir, launch_file_path)
        print('idir', self.mitsuba_launch_dir)
        if self.namespace =='/':
            self.create_map_launch_process = subprocess.Popen(['ros2', 'launch', idir])
        else:
            self.create_map_launch_process = \
                subprocess.Popen(['ros2', 'launch', idir, 'namespace:={}'.format(self.namespace)])
        self.pub_odom_reset.publish(String())  # mitsuba_diff_driveパッケージのcan_to_odoのオドメトリをリセットする

    def close_create_map(self):
        ret = messagebox.askokcancel(title='終了', message='地図の作成を終了しますか？')
        if ret:
            # create_map.launch関連ノードを終了する
            self.create_map_launch_process.send_signal(signal.SIGINT)
            exit_code = self.create_map_launch_process.returncode

            self.create_map_launch_process.wait(timeout=15)
            exit_code = self.create_map_launch_process.returncode
            messagebox.showinfo(title='完了', message='終了しました')
            print(f'終了コード: {exit_code}')

        # 地図作成起動ボタンを有効化
        self.button_create_map.config(state=tk.NORMAL)
        # 地図作成終了ボタンを無効化
        self.button_close_create_map.config(state=tk.DISABLED)
        # 地図保存ボタンを無効化
        self.button_save_map.config(state=tk.DISABLED)

    def get_map_folder_list(self):
        folders = [f.name for f in os.scandir(self.idir) if f.is_dir()]
        folders.sort()
        self.listbox_map_folder.delete(0, tk.END)
        for folder in folders:
            self.listbox_map_folder.insert(tk.END, folder)

    def create_folder(self):
        folder_name = self.entry_save_folder_name.get()
        if folder_name:
            path = os.path.join(self.idir, folder_name)
            if os.path.exists(path):
                messagebox.showerror(title='Error', message=str(folder_name) + ' は既に存在します。')
                return

            os.mkdir(path)
            map_folder = os.path.join(path, 'map')
            print('map_folder:' + map_folder)
            req_save = SaveMap.Request()
            req_save.name = String(data=map_folder)
            self.future = self.client_save.call_async(req_save)
            # galacticではsrvsにresponseが定義されていないためresultの参照は不可
            # rclpy.spin_until_future_complete(self, self.future)
            # req_save_result = self.future.result()
            # req_save_result = req_save_result.result
            # print('req_save_result:', req_save_result)

            req_serialize = SerializePoseGraph.Request()
            req_serialize.filename = map_folder
            self.future = self.client_serialize.call_async(req_serialize)
            # galacticではsrvsにresponseが定義されていないためresultの参照は不可
            # rclpy.spin_until_future_complete(self, self.future)
            # req_serialize_result = self.future.result()
            # req_serialize_result = req_serialize_result.result
            # print('req_serialize_result:', req_serialize_result)

            messagebox.showinfo(title='保存完了', message=str(folder_name) + ' を保存しました。')
            self.entry_save_folder_name.delete(0, tk.END)
            self.get_map_folder_list()
        else:
            messagebox.showwarning(title='確認', message='フォルダ名が未入力です。')

    def delete_map_folder(self):
        if self.listbox_map_folder.curselection():  # リスト未選択時は回避
            selected_folder = self.listbox_map_folder.get(self.listbox_map_folder.curselection())
            folder_path = os.path.join(self.idir, selected_folder)
            print(folder_path)
            ret = messagebox.askokcancel(
                title='フォルダの削除',
                message='このフォルダを削除しますか？',
                detail='フォルダ名：' + str(selected_folder),
            )

            if ret:
                subprocess.run(['gio', 'trash', folder_path], check=True)  # ゴミ箱へ移動
                self.get_map_folder_list()
                # Canvasをクリア
                self.canvas_map_preview.delete('all')

    def find_disp_map_images(self, event=None):
        if self.listbox_map_folder.curselection():  # リスト未選択時は回避
            selected_folder = self.listbox_map_folder.get(self.listbox_map_folder.curselection())
            directory = os.path.join(self.idir, selected_folder)
            image_files = [f for f in os.listdir(directory) if f.endswith('.pgm')]
            if image_files:
                image_path = os.path.join(directory, image_files[0])
                image = tk.PhotoImage(file=image_path)
                img_width, img_height = image.width(), image.height()
                # print(img_width, img_height)
                # print(self.canvas_map_preview.winfo_width(),
                #       self.canvas_map_preview.winfo_height())
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


def main():
    root = tk.Tk()
    root.title('main window')
    root.geometry('')

    tab = tk.Frame(root)
    tab.pack(expand=True, fill='both', padx=10, pady=10)

    rclpy.init()
    node = MapSave(tab)

    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
