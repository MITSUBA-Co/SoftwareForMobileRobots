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
from operator import truediv
import os
import tkinter as tk
import tkinter.ttk as ttk
from functools import partial  # 複数ボタンに引数を渡すために必要

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from mitsuba_gui import sdf_param_set
from mitsuba_gui.parameter_getter import ParameterGetter
from mitsuba_gui.parameter_setter import ParameterSetter


class ParamSet(Node):

    filenames_fullpath = set()
    yaml_data = {}  # パラメータ設定タブ間で共有するパラメータ辞書

    def __init__(self, tab, node_name='parame_set'):
        super().__init__(node_name)
        self.tab = tab
        self.tab_name = node_name
        self.read_csv()  # CSVファイルを読み込む
        self.read_yaml()  # CSVファイルで指定されたyamlファイルを読み込む
        self.parameter_setter = ParameterSetter(node_name)  # パラメータ変更クラス
        self.parameter_getter = ParameterGetter(node_name)  # パラメータ取得クラス
        self.create_widgets()  # ヴィジットの作成

    #     self.update()

    # def update(self):
    #     rclpy.spin_once(self, timeout_sec=0.0)
    #     self.tab.after(200,self.update)

    def read_csv(self):  # CSVファイル（変更するパラメータ）読み込み
        csv_header = [
            'name',
            'pkg_name',
            'file_name',
            'node_name',
            'parameter_name',
            'parameter_range'
        ]  # CSVヘッダの設定（表示名、パッケージ名、パラメータファイル名、ノード名、パラメータ名、パラメータ範囲）
        self.csvdata = []
        mitsuba_gui_dir = get_package_share_directory('mitsuba_gui')
        filename = os.path.join(
            mitsuba_gui_dir, 'paramsetting', self.tab_name + '.csv'
        )  # CSVファイルのフルパス
        with open(filename, newline='') as csvfile:
            reader = csv.DictReader(csvfile, csv_header)
            for row in reader:
                row['filename_fullpath'] = os.path.join(
                    get_package_share_directory(row['pkg_name']), row['file_name']
                )  # filename_fullpathを新しいキーとして、csvファイルで指定したyamlファイルのフルパスを設定
                if row['parameter_range'] is not None:
                    row['parameter_range'] = \
                        [float(value) for value in row['parameter_range'].split()]
                self.csvdata.append(row)
        #        print(self.csvdata)

        for row in self.csvdata:
            ParamSet.filenames_fullpath.add(row['filename_fullpath'])  # yamlファイルのフルパスをクラス変数に追加

    #        print(ParamSet.filenames_fullpath)

    # node_names = set()      #CSVで設定された全てのノード名を取得
    # for row in self.csvdata:
    # node_names.add(row['node_name'])
    # self.node_names = list(node_names)
    # print(self.node_names)

    def read_yaml(self):  # yamlファイル読み込み
        for filename_fullpath in ParamSet.filenames_fullpath:
            with open(filename_fullpath, 'r') as file:  # フルパスを使って、ファイルを読み込み
                data_load = yaml.safe_load(file)  # yamlファイルをロード
                data = {filename_fullpath: data_load}  # パラメータファイル名(yamlファイル)を先頭のキーとする
                ParamSet.yaml_data.update(data)  # パラメータ辞書に追加（同じデータの場合は上書きされる）

    #                print(data)

    def get_param_request(self, i, event=None):  # パラメータ値取得
        node_names = self.csvdata[i]['node_name'].split()
        parameter_names = self.csvdata[i]['parameter_name'].split()
        #        print(node_name, parameter_name)
        for node_name, parameter_name in zip(node_names, parameter_names):
            get_value = self.parameter_getter.get_remote_parameter(node_name, [parameter_name])

        if get_value:  # パラメータサーバが起動している時
            self.entries_get[i].configure(state=tk.NORMAL)  # disableを解除
            self.entries_get[i].delete(0, tk.END)  # テキストを削除
            # get_remote_parameteは複数のパラメーターの値を得られるがこの場合は０番目を表示する
            self.entries_get[i].insert(0, str(get_value[0]))
            self.entries_get[i].configure(state=tk.DISABLED)  # disableに設定

            self.entries_set[i].delete(0, tk.END)  # テキストを削除
            # get_remote_parameteは複数のパラメーターの値を得られるがこの場合は０番目を表示する
            self.entries_set[i].insert(0, str(get_value[0]))

    def get_element_type_in_list(self, nested_list):  # ネストされたリストの中の要素のtypeを取得
        while isinstance(nested_list, list):
            nested_list = nested_list[0]
        return type(nested_list)
    
    def convert_to_type(self, input_data, target_type):  # input_dataをtarget_typeに変換
        if target_type == str:
            self.get_logger().info('type str')
            return input_data
        elif isinstance(input_data, list):
            return [self.convert_to_type(element, target_type) for element in input_data]
        else:
            try:
                return target_type(input_data)
            except ValueError:
                return input_data
 
    def set_param_request(self, i, event=None):  # パラメータ値変更
        try:
            set_value = eval(self.entries_set[i].get())
            if isinstance(set_value, float) or isinstance(set_value, int):  # 設定値がfloat or intの場合、listにする
                set_value_list = [set_value]
            else:
                set_value_list = set_value
        except ValueError:  # floatに変換できなかった場合の処理
            print('eval convert error')
            return

        file_name = self.csvdata[i]['filename_fullpath']
        node_names = self.csvdata[i]['node_name'].split()
        parameter_names = self.csvdata[i]['parameter_name'].split()
        parameter_range = self.csvdata[i]['parameter_range']

        if parameter_range is None:
            flg_param_set = True
            self.get_logger().info(f'set_value: {set_value} parameter_range: {parameter_range}')
        elif all(((min <= value <= max) or (max <= value<= min)) for min, max, value 
                 in zip(parameter_range[::2], parameter_range[1::2], set_value_list)):
            flg_param_set = True
            self.get_logger().info(f'set_value: {set_value} parameter_range: {parameter_range}')
        else:
            flg_param_set = False
            self.get_logger().warn('parameter(s) out of range: '
                                   f'set_value: {set_value} parameter_range: {parameter_range}')

        if flg_param_set:  # パラメータ設定範囲が設定されていない、もしくは、パラメータが設定範囲内の場合
            print(node_names, parameter_names)
            for node_name, parameter_name in zip(node_names, parameter_names):
                get_value = self.parameter_getter.get_remote_parameter(node_name, [parameter_name])
                if get_value:  # パラメータサーバが起動している時
                    get_value_type = self.get_element_type_in_list(get_value)
                    self.get_logger().info(f'get_value: {get_value} type: {get_value_type}')
                    self.get_logger().info(f'set_value: {set_value}')
                    set_value_converted = self.convert_to_type(set_value, get_value_type)
                    self.get_logger().info(f'set_value_converted: {set_value_converted}')
                    self.parameter_setter.set_remote_parameter(node_name, [parameter_name], [set_value_converted])
                    get_value = self.parameter_getter.get_remote_parameter(node_name, [parameter_name])

                    if '/' in node_name:  # ノード名に『/』があった場合、『/』の前と後ろの両方をキーとする
                        node_name_split = node_name.split('/')
                        if '.' in parameter_name:
                            parameter_name_split = parameter_name.split('.')
                            ParamSet.yaml_data[file_name][node_name_split[0]][node_name_split[1]][
                                'ros__parameters'
                            ][parameter_name_split[0]][parameter_name_split[1]] = get_value[0]
                        else:
                            print(node_name_split[0], node_name_split[1])
                            ParamSet.yaml_data[file_name][node_name_split[0]][node_name_split[1]][
                                'ros__parameters'
                            ][parameter_name] = get_value[0]
                            print('yaml file update:', ParamSet.yaml_data[file_name]
                                  [node_name_split[0]][node_name_split[1]][
                                    'ros__parameters'][parameter_name])
                    else:
                        if '.' in parameter_name:
                            parameter_name_split = parameter_name.split('.')
                            ParamSet.yaml_data[file_name][node_name]['ros__parameters'][
                                parameter_name_split[0]
                            ][parameter_name_split[1]] = get_value[0]
                        else:
                            ParamSet.yaml_data[file_name][node_name]['ros__parameters'][
                                parameter_name
                            ] = get_value[0]

        self.get_param_request(i)
        self.tab.focus_set()

    def create_widgets(self):
        self.tab.bind('<Visibility>', self.on_tab_select)  # タブが選択されたときにパラメータの更新を行う
        # 何もないところをクリックした時にフォーカスをもらう（entryからフォーカスを無くす）
        self.tab.bind('<Button-1>', self.on_root_click)

        label1 = tk.Label(self.tab, text='パラメータ名')
        label1.grid(row=0, column=0)
        label2 = tk.Label(self.tab, text='現在値')
        label2.grid(row=0, column=1)
        label3 = tk.Label(self.tab, text='設定値')
        label3.grid(row=0, column=2)

        self.labels = []  # パラメータ名
        self.entries_get = []  # 現在のパラメータの値
        self.entries_set = []  # 変更するパラメータの値
        self.buttons = []  # パラメータ変更ボタン
        for i in range(len(self.csvdata)):
            i_row = i + 1  # 1行目にテキストを挿入しているためオフセットする
            self.labels.append(tk.Label(self.tab, text=self.csvdata[i]['name']))
            self.labels[i].grid(row=i_row, column=0)

            # パラメータ設定値入力用エントリ(下のget_param_requestでアクセスするので、先に作成する)
            self.entries_set.append(tk.Entry(self.tab, width=50, justify=tk.RIGHT))
            self.entries_set[i].grid(row=i_row, column=2)
            # パラメータを変更して、リターンを押したらパラメータ変更する
            self.entries_set[i].bind('<Return>', partial(self.set_param_request, i))
            self.entries_set[i].bind('<FocusIn>', self.select_all)  # entryをクリックしたら全選択状態とする
            # 数値を変更したが、パラメータを変更しないで、他のエントリーをクリックした時や、他のところをクリックした時に値を戻す
            self.entries_set[i].bind('<FocusOut>', partial(self.get_param_request, i))

            self.entries_get.append(
                tk.Entry(self.tab, width=20, justify=tk.RIGHT, disabledforeground='black')
            )  # パラメータ取得値用エントリ
            self.get_param_request(i)  # パラメータ値の取得
            self.entries_get[i].grid(row=i_row, column=1)

            self.buttons.append(
                tk.Button(self.tab, text='set', command=partial(self.set_param_request, i))
            )  # パラメータ設定ボタン
            self.buttons[i].grid(row=i_row, column=3)

        button_save = tk.Button(self.tab, text='パラメータ保存', command=self.btn_save)
        button_save.grid(row=i_row + 1, column=2)

    def on_root_click(self, event):  # 何もないところをクリックしたときにentryからフォーカスを外す
        event.widget.focus_set()

    def select_all(self, event):  # entryをクリックしたときに数値を全選択状態にする
        event.widget.select_range(0, tk.END)

    def on_tab_select(self, event):  # 全パラメータを取得する
        for i in range(len(self.csvdata)):
            self.get_param_request(i)

    def btn_save(self):  # 変更したパラメータをyamlファイルに上書き保存する
        noalias_dumper = yaml.Dumper  # yamlファイルに保存するときに、アンカーとアライアスを使わないようにする（使うと読み込めなくなる）
        noalias_dumper.ignore_aliases = lambda *args: True

        for filename_fullpath in ParamSet.filenames_fullpath:
            print(filename_fullpath)
            with open(filename_fullpath, 'w') as file:
                yaml.dump(ParamSet.yaml_data[filename_fullpath], file, Dumper=noalias_dumper)
        print('save end')


def main():
    root = tk.Tk()
    root.title('main window')
    root.geometry('')

    tab = tk.Frame(root)
    tab.pack(expand=True, fill='both', padx=10, pady=10)

    rclpy.init()

    style = ttk.Style()
    style.configure('Horizontal.TNotebook', tabposition='wn')
    inner_notebook = ttk.Notebook(tab, style='Horizontal.TNotebook')
    inner_notebook.pack(expand=True, fill='both')

    param_setting_files = sorted(
        os.listdir(get_package_share_directory('mitsuba_launch') + '/paramsetting')
    )  # フォルダ内のファイルリストを取得し、ファイル名でソート
    for param_setting_file in param_setting_files:
        inner_frame = ttk.Frame(inner_notebook)
        name = os.path.splitext(param_setting_file)[0]  # ファイル名の拡張子を除く
        inner_notebook.add(inner_frame, text=name)  # ファイル名の拡張子を除いた名前の縦タブを作成
        ParamSet(inner_frame, node_name=name)  # ファイル名の拡張子を除いた名前のノードを作成

    inner_frame = ttk.Frame(inner_notebook)
    inner_notebook.add(inner_frame, text='sensor mount')  # センサ取付位置の縦タブを作成
    sdf_param_set.Toplevel1(inner_frame)  # タブページを作成

    root.mainloop()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
