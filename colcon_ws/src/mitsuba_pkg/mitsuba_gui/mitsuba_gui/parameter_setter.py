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

import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter


class ParameterSetter(Node):

    def __init__(self, node_name=''):
        super().__init__(node_name + '_parameter_setter')

    def set_remote_parameter(self, node_name, parameter_names, parameter_values):
        self.client = self.create_client(SetParameters, f'{node_name}/set_parameters')
        if not self.client.wait_for_service(timeout_sec=0.01):  # パラメータサーバが起動していなかったら諦める場合
            self.get_logger().info(f'Service not available: {node_name}/set_parameters')
            return False
        #        while not self.client.wait_for_service(timeout_sec=1.0):   #パラメータサーバが起動するまで待つ場合
        #            self.get_logger().info(f'Service not available: {node_name}/set_parameters')

        self.client.wait_for_service(timeout_sec=1.0)

        parameters = []
        for name, value in zip(parameter_names, parameter_values):
            if (
                isinstance(value, (int, float, str, bool))
                or (isinstance(value, list) and all(isinstance(x, bytes) for x in value))
                or (isinstance(value, list) and all(isinstance(x, bool) for x in value))
                or (isinstance(value, list) and all(isinstance(x, int) for x in value))
                or (isinstance(value, list) and all(isinstance(x, float) for x in value))
                or (isinstance(value, list) and all(isinstance(x, str) for x in value))
            ):
                parameter = Parameter(name=name, value=value).to_parameter_msg()
            elif isinstance(value, list) and all(isinstance(x, list) for x in value):
                parameter = Parameter(
                    name=name, value=str(value)
                ).to_parameter_msg()  # ネストされたリストの時はvalueをstrに変換する
                print(parameter)
            else:  # 無効な値が設定された場合の処理
                print(f'Invalid value:{name}:{value}')
                return False
            parameters.append(parameter)

        request = SetParameters.Request(parameters=parameters)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            for result, name, value in zip(response.results, parameter_names, parameter_values):
                if result.successful:
                    self.get_logger().info(
                        f'Set parameter {node_name}/{name}: {value} successfully'
                    )
                else:
                    self.get_logger().warn(
                        f'Failed to set parameter {node_name}/{name}: {result.reason}'
                    )
                    return False
        return True


def main(args=None):
    rclpy.init(args=args)
    parameter_setter = ParameterSetter()
    # parameter_setter.set_remote_parameter('/can_to_odo', ['dia'], [0.5])
    # parameter_setter.set_remote_parameter('/can_to_odo', ['dia','tread'], [0.5,0.2])
    # parameter_setter.set_remote_parameter(
    #     '/velocity_smoother', ['max_velocity'], [[1.0, 0.0, 3.14]])
    parameter_setter.set_remote_parameter(
        '/controller_server', ['FollowPath.max_vel_x', 'FollowPath.max_speed_xy'], [1.0, 1.0]
    )

    rclpy.shutdown()


if __name__ == '__main__':
    main()
