#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sound_play.msg import SoundRequest
from nav2_msgs.srv import ClearEntireCostmap

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from mitsuba_gui.parameter_setter import ParameterSetter
from mitsuba_gui.parameter_getter import ParameterGetter

import os
import csv
import subprocess  # terminalでのプロンプト実行に必要
import time

"""
Basic navigation demo to go to poses.
"""


class csv2Waypoints(Node):
    def __init__(self):
        super().__init__('nav2_simple_commander')
        self.init_proc()
        self.subRouteInitialpose = self.create_subscription(
            String, 'mitsuba/route_initialpose', self.route_initialpose_callback, 10)
        self.subRouteLoad = self.create_subscription(
            String, 'mitsuba/route_load', self.route_load_callback, 10)
        self.subRouteStart = self.create_subscription(
            String, 'mitsuba/route_start', self.route_start_callback, 10)
        self.subRouteRestart = self.create_subscription(
            String, 'mitsuba/route_restart', self.route_restart_callback, 10)
        self.subRouteCancel = self.create_subscription(
            String, 'mitsuba/route_cancel', self.route_cancel_callback, 10)
        self.pubSoundRequest = self.create_publisher(
            SoundRequest, 'robotsound', 10)
        timer_period = 0.5  # seconds
        timer_05s = self.create_timer(timer_period, self.timer_callback)

#        self.sound_request_msg = SoundRequest(sound=SoundRequest.NEEDS_PLUGGING, command=SoundRequest.PLAY_ONCE, volume=0.1)    #3秒のコールバックで都度再生を行う場合
#        timer_period = 3.0 # seconds                                                                                            #3秒のコールバックで都度再生を行う場合
#        timer_3s = self.create_timer(timer_period, self.timer_3s_callback)                                                      #3秒のコールバックで都度再生を行う場合
        sound_file = os.path.join(get_package_share_directory(
            'sound_play'), 'sounds', 'auto_driving.ogg')  # サウンドファイルを再生する場合
        self.sound_request_msg_start = SoundRequest(
            sound=SoundRequest.PLAY_FILE, command=SoundRequest.PLAY_START, volume=1.0, arg=sound_file)  # サウンドファイルを再生する場合
        self.sound_request_msg_stop = SoundRequest(
            sound=SoundRequest.PLAY_FILE, command=SoundRequest.PLAY_STOP, volume=1.0, arg=sound_file)  # サウンドファイルを再生する場合
#        self.sound_request_msg_start = SoundRequest(sound=SoundRequest.NEEDS_PLUGGING, command=SoundRequest.PLAY_START, volume=0.1)           #ビルトインサウンドファイルを再生する場合
#        self.sound_request_msg_stop  = SoundRequest(sound=SoundRequest.NEEDS_PLUGGING, command=SoundRequest.PLAY_STOP , volume=0.1)           #ビルトインサウンドファイルを再生する場合
#        self.sound_request_msg_start = SoundRequest(sound=SoundRequest.SAY, command=SoundRequest.PLAY_START, volume=1.0, arg="autonomous driving") #音声合成を行う場合
#        self.sound_request_msg_stop  = SoundRequest(sound=SoundRequest.SAY, command=SoundRequest.PLAY_STOP , volume=1.0, arg="autonomous driving") #音声合成を行う場合

        self.client = self.create_client(
            ClearEntireCostmap, "global_costmap/clear_entirely_global_costmap")

        self.parameter_setter = ParameterSetter()  # パラメータ変更クラス
        self.parameter_getter = ParameterGetter()  # パラメータ取得クラス

        self.behavior_xml = os.path.join(get_package_share_directory(
            'nav2_bt_navigator'), 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery_only_wait.xml')

    def init_proc(self):
        self.navigator = BasicNavigator()
        self.goal_poses = []
        self.cmd_vels = []
        self.obsts = []
        self.procs = []
        self.i_search = 0
        self.i_cur = 0
        self.flg_start = False  # 自律走行開始指令を受け取り、自律走行終了するまでTrue
        self.flg_stop = False  # 一時停止ボタンが押されたか、経路中の一時停止ポイントに到達したらTrue
        self.flg_gothrouph = False  # goThrouphPosesの実行中True
        self.loop = 0  # １回の走行は0、ループ走行の場合はループ走行の回数

    def end_proc(self):
        self.pubSoundRequest.publish(self.sound_request_msg_stop)  # sound stop
        self.flg_start = False

    def timer_3s_callback(self):
        if self.flg_gothrouph == True:  # 目標地点に移動時
            self.pubSoundRequest.publish(self.sound_request_msg)  # sound start
#            subprocess.call('ros2 topic pub --once /robotsound sound_play/msg/SoundRequest\
#                    "{sound: -2, command: 1, arg: "/$HOME/.mitsuba/sounds/NEEDS_PLUGGING.ogg", volume: 1.0}"', shell=True)

    def timer_callback(self):
        if self.flg_start == False or self.flg_stop == True:
            return  # 自律走行未スタートか一時停止中はすぐに抜ける
        # nav_start = navigator.get_clock().now()
        if self.flg_gothrouph == False:  # 初期状態、もしくは、目標地点に到着時
            i_start = self.i_search
            self.i_cur = self.i_search
            self.get_logger().info(f'current index: {self.i_cur}')
            self.get_logger().info(f'i_start: {i_start}')
            while self.i_search < len(self.goal_poses):  # 一時停止ポイントまでインデックスを進める
                if self.cmd_vels[self.i_search] == 0.0:
                    break
                self.i_search += 1
            i_end = self.i_search
            self.get_logger().info(f'i_end: {i_end}')
            if i_start == i_end:  # 一時停止だったら
                self.get_logger().info('pause point')
                self.flg_stop = True
                self.publish_proc_and_set_param()  # 一時停止時に行う処理をパブリッシュ
            else:  # 一時停止ポイントではない場合、次の一時停止ポイント、もしくは、ゴールポイントまでのgoal_posesをパブリッシュ
                self.get_logger().info(f'goThroughPoses: i_start: {i_start}, i_end: {i_end}, \
                                       goal_poses: {self.goal_poses[i_start:i_end]}')
                self.publish_proc_and_set_param()  # 最初のウェイポイントに行くまでに行う処理をパブリッシュ
                self.navigator.goThroughPoses(
                    self.goal_poses[i_start:i_end], self.behavior_xml)
                self.flg_gothrouph = True
                self.pubSoundRequest.publish(self.sound_request_msg_start)  # sound start

        elif self.flg_gothrouph == True:  # 目標地点に移動時
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'current index: {self.i_cur}')
                self.get_logger().info(f'remaining goals: {feedback.number_of_poses_remaining}')
                self.get_logger().info(f'remaining distance[m]: {feedback.distance_remaining}')
                if self.i_cur < self.i_search - feedback.number_of_poses_remaining:
                    self.i_cur = self.i_search - feedback.number_of_poses_remaining
                    self.publish_proc_and_set_param()

            if self.navigator.isTaskComplete():
                self.pubSoundRequest.publish(self.sound_request_msg_stop)  # sound stop
                self.flg_gothrouph = False  # goThrouphPoses完了
                self.i_cur = self.i_search  # goThrouphPoses完了時は現在のインデックスはサーチインデックスになる
                if self.i_search == len(self.goal_poses):  # 経路が最終行まで実行されたら
                    if self.loop > 1:  # self.loopの値を１づつ引いていき、1になったら終了
#                        self.navigator = BasicNavigator()
                        self.i_cur = 0
                        self.i_search = 0
                        self.loop = self.loop - 1
                    else:
                        self.end_proc()
                # Do something depending on the return code
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('Goal succeeded!')
                elif result == TaskResult.CANCELED:
                    self.get_logger().info('Goal was canceled!')
                elif result == TaskResult.FAILED:
                    self.get_logger().info('Goal failed!')
                else:
                    self.get_logger().info('Goal has an invalid return status!')
                
                self.navigator = BasicNavigator()

    def publish_proc_and_set_param(self):
        proc = self.procs[self.i_cur]
        for key, value in proc.items():  # 設定されたトピックにStringをパブリッシュ
            publisher = self.create_publisher(String, key, 10)
            publisher.publish(String(data=value))

        vel = self.cmd_vels[self.i_cur]  # 走行速度設定反映
        self.parameter_setter.set_remote_parameter(
            'controller_server', ['FollowPath.desired_linear_vel'], [vel])

        obst = self.obsts[self.i_cur]
        if (obst == 'avoid'):  # 障害物回避設定時
            self.parameter_setter.set_remote_parameter(
                'global_costmap/global_costmap', ['obstacle_layer.enabled'], [True])
        elif (obst == 'stop'):  # 障害物停止設定時
            self.parameter_setter.set_remote_parameter(
                'global_costmap/global_costmap', ['obstacle_layer.enabled'], [False])
            self.client.call_async(
                ClearEntireCostmap.Request())  # グローバルコストマップをクリア
        self.get_logger().info(f'current index: {self.i_cur}, obst setting: {obst}, max_vel: {vel}')

    def route_initialpose_callback(self, msg):  # 初期位置・角度設定
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

    def route_start_callback(self, msg=""):  # 自律走行開始処理
        self.init_proc()
        self.goal_poses = self.goal_poses_read
        self.cmd_vels = self.cmd_vels_read
        self.obsts = self.obsts_read
        self.procs = self.procs_read
        self.flg_start = True

        if msg.data == "":  # ループ回数の設定
            self.loop = 1  # ループ走行のチェックがない場合は１回走行
        elif msg.data == "0":
            self.loop = float("inf")  # 無限に走行
        else:
            self.loop = int(msg.data)  # ループ回数に設定した回数だけ走行
        self.get_logger().info(f'loop: {self.loop}')

    def route_restart_callback(self, msg):  # 一時停止解除
        if self.flg_stop == True:
            self.flg_stop = False
            self.i_search += 1
            self.i_cur = self.i_search
            self.get_logger().info('restart')

    def route_cancel_callback(self, msg):  # 自律走行キャンセル
        self.navigator.cancelTask()
        self.end_proc()
        self.get_logger().info('cancel callback')

    def route_load_callback(self, msg):
        self.goal_poses_read = []
        self.cmd_vels_read = []
        self.obsts_read = []
        self.procs_read = []
        filename = msg.data
        csvfile = open(filename, 'r')
        gotdata = csv.reader(csvfile)
        for row in gotdata:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            proc = {}
            for r in row:
                self.get_logger().info(f'row: {r}')
                if "=" in r:
                    key, value = r.split("=")
                    if key == "x":
                        goal_pose.pose.position.x = float(value)
                    elif key == "y":
                        goal_pose.pose.position.y = float(value)
                    elif key == "z":
                        goal_pose.pose.position.z = float(value)
                    elif key == "qx":
                        goal_pose.pose.orientation.x = float(value)
                    elif key == "qy":
                        goal_pose.pose.orientation.y = float(value)
                    elif key == "qz":
                        goal_pose.pose.orientation.z = float(value)
                    elif key == "qw":
                        goal_pose.pose.orientation.w = float(value)
                    elif key == "v":
                        cmd_vel = float(value)
                    elif key == "obst":
                        obst = value
                    else:
                        proc[key] = value
            self.goal_poses_read.append(goal_pose)
            self.cmd_vels_read.append(cmd_vel)
            self.obsts_read.append(obst)
            self.procs_read.append(proc)
        self.get_logger().info(f'goal_poses: {self.goal_poses_read}')
        self.get_logger().info(f'cmd_vels: {self.cmd_vels_read}')
        self.get_logger().info(f'obsts: {self.obsts_read}')
        self.get_logger().info(f'procs: {self.procs_read}')
        self.get_logger().info('csv file read')

        csvfile.close()


def main(args=None):
    rclpy.init(args=args)
    node = csv2Waypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
