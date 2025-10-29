#!/usr/bin/env python3
"""
测试传感器参数是否可以正常工作
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from openpilot.common.params import Params

def test_sensor_params():
    """
    测试传感器参数是否可以正常工作
    """
    print("开始测试传感器参数...")
    
    # 创建参数对象
    params = Params()
    
    # 测试写入参数
    print("测试写入传感器参数...")
    params.put_bool("GyroSensorConnected", True)
    params.put_bool("AccelSensorConnected", True)
    
    # 测试读取参数
    print("测试读取传感器参数...")
    gyro_connected = params.get_bool("GyroSensorConnected")
    accel_connected = params.get_bool("AccelSensorConnected")
    
    print(f"陀螺仪连接状态: {gyro_connected}")
    print(f"加速度计连接状态: {accel_connected}")
    
    # 测试设置为False
    print("测试设置传感器参数为False...")
    params.put_bool("GyroSensorConnected", False)
    params.put_bool("AccelSensorConnected", False)
    
    # 再次读取
    gyro_connected = params.get_bool("GyroSensorConnected")
    accel_connected = params.get_bool("AccelSensorConnected")
    
    print(f"陀螺仪连接状态: {gyro_connected}")
    print(f"加速度计连接状态: {accel_connected}")
    
    print("传感器参数测试完成!")

if __name__ == "__main__":
    try:
        test_sensor_params()
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()