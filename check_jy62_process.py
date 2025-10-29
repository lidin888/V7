#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from system.manager.process_config import managed_processes, jy62_sensor
from openpilot.common.params import Params

def check_jy62_process():
    """检查JY62传感器进程状态"""
    print("检查JY62传感器进程状态...")
    
    params = Params()
    is_onroad = params.get_bool("IsOnroad")
    gyro_connected = params.get_bool("GyroSensorConnected")
    
    print(f"IsOnroad: {is_onroad}")
    print(f"GyroSensorConnected: {gyro_connected}")
    
    if 'sensord_jy62' in managed_processes:
        proc = managed_processes['sensord_jy62']
        print(f"进程是否运行: {proc.proc is not None}")
        
        # 直接调用启用条件函数
        should_run = jy62_sensor(is_onroad, params, None)
        print(f"是否应该运行: {should_run}")
        
        print("进程状态分析:")
        print(f"  - IsOnroad: {is_onroad}")
        print(f"  - GyroSensorConnected: {gyro_connected}")
    else:
        print("错误: 在managed_processes中未找到sensord_jy62进程")

if __name__ == "__main__":
    check_jy62_process()