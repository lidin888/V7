#!/usr/bin/env python3
"""
最终测试脚本，验证所有功能是否正常工作
"""

def test_params():
    """测试参数系统"""
    print("测试参数系统...")
    try:
        from openpilot.common.params import Params
        p = Params()
        # 测试写入
        p.put_bool("GyroSensorConnected", True)
        p.put_bool("AccelSensorConnected", True)
        # 测试读取
        gyro = p.get_bool("GyroSensorConnected")
        accel = p.get_bool("AccelSensorConnected")
        print(f"参数测试成功 - Gyro: {gyro}, Accel: {accel}")
        return True
    except Exception as e:
        print(f"参数测试失败: {e}")
        return False

def test_imports():
    """测试关键模块导入"""
    print("测试模块导入...")
    modules = [
        "system.manager.manager",
        "openpilot.common.params",
        "cereal.messaging",
        "openpilot.common.transformations.transformations",
        "openpilot.selfdrive.modeld.models.commonmodel_pyx",
        "opendbc.can.packer_pyx",
        "opendbc.can.parser_pyx"
    ]
    
    results = []
    for module in modules:
        try:
            __import__(module)
            print(f"  {module} 导入成功")
            results.append(True)
        except Exception as e:
            print(f"  {module} 导入失败: {e}")
            results.append(False)
    
    return all(results)

def test_sensor_functions():
    """测试传感器相关功能"""
    print("测试传感器功能...")
    try:
        from openpilot.common.params import Params
        from system.manager.manager import check_sensor_connection
        import time
        
        # 创建参数对象
        params = Params()
        
        # 调用传感器检查函数
        print("调用传感器检查函数...")
        check_sensor_connection(params)
        
        # 检查结果
        gyro = params.get_bool("GyroSensorConnected")
        accel = params.get_bool("AccelSensorConnected")
        print(f"传感器检查结果 - Gyro: {gyro}, Accel: {accel}")
        return True
    except Exception as e:
        print(f"传感器功能测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """主测试函数"""
    print("=" * 50)
    print("开始最终测试")
    print("=" * 50)
    
    tests = [
        test_imports,
        test_params,
        test_sensor_functions
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
            print()
        except Exception as e:
            print(f"测试 {test.__name__} 发生异常: {e}")
            results.append(False)
            print()
    
    print("=" * 50)
    print("测试结果汇总:")
    test_names = ["模块导入", "参数系统", "传感器功能"]
    for i, (name, result) in enumerate(zip(test_names, results)):
        status = "通过" if result else "失败"
        print(f"  {name}: {status}")
    
    all_passed = all(results)
    print(f"\n总体结果: {'所有测试通过' if all_passed else '部分测试失败'}")
    print("=" * 50)
    
    return all_passed

if __name__ == "__main__":
    main()