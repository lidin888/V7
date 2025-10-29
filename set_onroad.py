#!/usr/bin/env python3

import time
from openpilot.common.params import Params

def set_onroad():
    """设置车辆为onroad状态以启动传感器进程"""
    params = Params()
    
    # 获取当前状态
    is_onroad = params.get_bool("IsOnroad")
    gyro_connected = params.get_bool("GyroSensorConnected")
    
    print(f"当前状态:")
    print(f"  IsOnroad: {is_onroad}")
    print(f"  GyroSensorConnected: {gyro_connected}")
    
    # 设置为onroad状态
    params.put_bool("IsOnroad", True)
    print("\n已设置 IsOnroad = True")
    
    # 等待一段时间观察进程状态
    time.sleep(2)
    
    # 再次检查状态
    is_onroad = params.get_bool("IsOnroad")
    print(f"\n更新后的状态:")
    print(f"  IsOnroad: {is_onroad}")

if __name__ == "__main__":
    set_onroad()