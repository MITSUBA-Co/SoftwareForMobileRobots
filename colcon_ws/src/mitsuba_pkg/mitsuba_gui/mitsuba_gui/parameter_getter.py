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
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node


class ParameterGetter(Node):

    def __init__(self, node_name=''):
        super().__init__(node_name + '_parameter_getter')

    def get_remote_parameter(self, node_name, parameter_names):
        self.client = self.create_client(GetParameters, f'{node_name}/get_parameters')
        if not self.client.wait_for_service(timeout_sec=0.0):  # パラメータサーバが起動していなかったら諦める場合
            self.get_logger().info(f'Service not available: {node_name}/get_parameters')
            return False
        #        while not self.client.wait_for_service(timeout_sec=1.0):   #パラメータサーバが起動するまで待つ場合
        #            self.get_logger().info(f'Service not available: {node_name}/get_parameters')

        request = GetParameters.Request()
        request.names = parameter_names
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.values != []:
            self.get_logger().info(f'Get parameter {node_name}:{parameter_names} successfully')
        else:
            self.get_logger().warn(f'Failed to get parameter {node_name}:{parameter_names}')

        #        print(response)
        return_values = []
        for pvalue in response.values:
            if pvalue.type == ParameterType.PARAMETER_BOOL:
                value = pvalue.bool_value
            elif pvalue.type == ParameterType.PARAMETER_INTEGER:
                value = pvalue.integer_value
            elif pvalue.type == ParameterType.PARAMETER_DOUBLE:
                value = pvalue.double_value
            elif pvalue.type == ParameterType.PARAMETER_STRING:
                value = str(pvalue.string_value)  # Stringの場合はstr()でString型にして返す
            elif pvalue.type == ParameterType.PARAMETER_BYTE_ARRAY:
                value = pvalue.byte_array_value.tolist()  # tolist()でリストにする
            elif pvalue.type == ParameterType.PARAMETER_BOOL_ARRAY:
                value = pvalue.bool_array_value.tolist()  # tolist()でリストにする
            elif pvalue.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                value = pvalue.integer_array_value.tolist()  # tolist()でリストにする
            elif pvalue.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                value = pvalue.double_array_value.tolist()  # tolist()でリストにする（array('d'を消す）
            elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
                value = str(
                    pvalue.string_array_value.tolist()
                )  # tolist()でリストにする（string_arrayの場合str()をつけるだけで良いのか未検証
            elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
                value = None
            else:
                value = None
                self.get_logger().warn(f'invalid parameter value: {value}')
            return_values.append(value)

        return return_values


def main(args=None):
    rclpy.init(args=args)
    parameter_getter = ParameterGetter()
    # value = parameter_getter.get_remote_parameter('/can_to_odo', ['dia', 'tread'])
    # value = parameter_getter.get_remote_parameter('/joy_linux_node', ['autorepeat_rate'])
    # value = parameter_getter.get_remote_parameter('/velocity_smoother', ['max_velocity'])
    # value = parameter_getter.get_remote_parameter('controller_server', ['FollowPath.max_vel_x'])
    # value = parameter_getter.get_remote_parameter('collision_monitor', ['PolygonStop.points'])
    value = parameter_getter.get_remote_parameter('/global_costmap/global_costmap', ['footprint'])
    print(value)
    # for val in value:
    #     if isinstance(val, array.array): print(val.tolist())
    #     else: print(val)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
