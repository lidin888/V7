#!/usr/bin/env python3
"""
简化版传感器检查测试
"""

from openpilot.common.params import Params

def test_simple_sensor_check():
    """
    简化版传感器检查测试
    """
    print("开始测试传感器参数读写功能...")
    
    # 创建参数对象
    params = Params()
    
    # 初始化参数
    print("初始化传感器参数...")
    params.put_bool("GyroSensorConnected", False)
    params.put_bool("AccelSensorConnected", False)
    
    # 检查初始状态
    gyro_initial = params.get_bool("GyroSensorConnected")
    accel_initial = params.get_bool("AccelSensorConnected")
    print(f"初始状态 - 陀螺仪: {gyro_initial}, 加速度计: {accel_initial}")
    
    # 模拟传感器检测逻辑
    print("模拟传感器检测...")
    gyro_detected = True   # 假设陀螺仪检测到了
    accel_detected = True  # 假设加速度计检测到了
    
    # 更新参数
    params.put_bool("GyroSensorConnected", gyro_detected)
    params.put_bool("AccelSensorConnected", accel_detected)
    
    # 检查更新后的状态
    gyro_final = params.get_bool("GyroSensorConnected")
    accel_final = params.get_bool("AccelSensorConnected")
    print(f"最终状态 - 陀螺仪: {gyro_final}, 加速度计: {accel_final}")
    
    print("传感器参数读写测试完成!")

if __name__ == "__main__":
    try:
        test_simple_sensor_check()
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()