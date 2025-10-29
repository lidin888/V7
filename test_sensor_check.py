#!/usr/bin/env python3
"""
测试传感器连接检查功能
"""

import time
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

def test_sensor_check():
    """
    测试传感器连接检查功能
    """
    print("开始测试传感器连接检查功能...")
    
    # 创建参数对象
    params = Params()
    
    # 记录检查前的状态
    gyro_before = params.get_bool("GyroSensorConnected")
    accel_before = params.get_bool("AccelSensorConnected")
    print(f"检查前状态 - 陀螺仪: {gyro_before}, 加速度计: {accel_before}")
    
    # 订阅传感器数据
    sm = messaging.SubMaster(['gyroscope', 'accelerometer'])
    
    # 等待一段时间收集传感器数据
    gyro_connected = False
    accel_connected = False
    
    print("开始5秒传感器数据监听...")
    start_time = time.time()
    
    while time.time() - start_time < 5.0:
        sm.update(100)  # 100ms 超时
        
        # 检查陀螺仪数据
        if sm.updated['gyroscope']:
            print(f"收到陀螺仪数据更新，状态: {sm['gyroscope'].gyroUncalibrated.status}")
            if sm['gyroscope'].gyroUncalibrated.status:
                gyro_connected = True
                print("检测到陀螺仪数据，陀螺仪已连接")
        
        # 检查加速度计数据
        if sm.updated['accelerometer']:
            print(f"收到加速度计数据更新，状态: {sm['accelerometer'].acceleration.status}")
            if sm['accelerometer'].acceleration.status:
                accel_connected = True
                print("检测到加速度计数据，加速度计已连接")
        
        # 如果两个传感器都连接上了，可以提前退出
        if gyro_connected and accel_connected:
            print("两个传感器都已连接，提前结束监听")
            break
    
    # 更新参数
    params.put_bool("GyroSensorConnected", gyro_connected)
    params.put_bool("AccelSensorConnected", accel_connected)
    
    # 记录日志
    print(f"传感器连接检查完成 - 陀螺仪: {gyro_connected}, 加速度计: {accel_connected}")
    
    # 读取更新后的参数
    gyro_after = params.get_bool("GyroSensorConnected")
    accel_after = params.get_bool("AccelSensorConnected")
    print(f"检查后状态 - 陀螺仪: {gyro_after}, 加速度计: {accel_after}")
    
    return gyro_connected, accel_connected

if __name__ == "__main__":
    try:
        gyro_status, accel_status = test_sensor_check()
        print(f"\n最终结果:")
        print(f"陀螺仪连接状态: {gyro_status}")
        print(f"加速度计连接状态: {accel_status}")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()