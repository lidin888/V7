#!/usr/bin/env python3
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from openpilot.common.params import Params
from system.manager.process_config import managed_processes
from system.hardware import PC

def test_jy62_python():
    """测试Python版本的JY62传感器进程"""
    print("测试Python版本的JY62传感器进程配置...")
    
    # 检查进程是否在managed_processes中
    if 'sensord_jy62' in managed_processes:
        process = managed_processes['sensord_jy62']
        print(f"进程名称: {process.name}")
        print(f"进程类型: {type(process)}")
        print(f"启用条件函数: {process.enabled}")
        
        # 测试启用条件
        params = Params()
        # 设置传感器为已连接
        params.put_bool("GyroSensorConnected", True)
        
        # 测试在PC模式下是否应该运行
        should_run = process.should_run(True, params, None)  # started=True, PC mode
        print(f"在PC模式下应该运行: {should_run}")
        
        # 测试在非PC模式下是否应该运行
        if not PC:
            should_run = process.should_run(True, params, None)  # started=True
            print(f"在非PC模式下启动时应该运行: {should_run}")
            
            should_run = process.should_run(False, params, None)  # started=False
            print(f"在非PC模式下未启动时应该运行: {should_run}")
    else:
        print("错误: sensord_jy62进程未在managed_processes中找到")

if __name__ == "__main__":
    test_jy62_python()