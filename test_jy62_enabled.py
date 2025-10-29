#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from system.manager.process_config import jy62_sensor
from openpilot.common.params import Params

def test_jy62_enabled():
    """测试JY62传感器进程启用条件"""
    print("测试JY62传感器进程启用条件...")
    
    params = Params()
    
    # 测试不同条件组合
    test_cases = [
        (False, "车辆未启动"),
        (True, "车辆已启动")
    ]
    
    for started, description in test_cases:
        print(f"\n{description}:")
        print("-" * 30)
        result = jy62_sensor(started, params, None)
        print(f"结果: {'启用' if result else '不启用'}")

if __name__ == "__main__":
    test_jy62_enabled()