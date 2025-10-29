#!/usr/bin/env python3
import os
import time
from openpilot.common.params import Params
import serial

def test_comprehensive():
    """综合测试传感器功能"""
    print("=== 综合传感器测试 ===")
    
    # 1. 检查参数系统
    print("\n1. 检查参数系统...")
    params = Params()
    try:
        gyro_connected = params.get_bool("GyroSensorConnected")
        accel_connected = params.get_bool("AccelSensorConnected")
        print(f"  GyroSensorConnected: {gyro_connected}")
        print(f"  AccelSensorConnected: {accel_connected}")
        print("  参数系统工作正常")
    except Exception as e:
        print(f"  参数系统错误: {e}")
        return
    
    # 2. 检查串口设备
    print("\n2. 检查串口设备...")
    serial_ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyS0"]
    serial_port = None
    for port in serial_ports:
        if os.path.exists(port):
            serial_port = port
            break
    
    if serial_port:
        print(f"  找到串口设备: {serial_port}")
        # 检查权限
        try:
            st = os.stat(serial_port)
            perms = oct(st.st_mode)[-3:]
            print(f"  设备权限: {perms}")
        except Exception as e:
            print(f"  权限检查失败: {e}")
    else:
        print("  未找到串口设备")
        return
    
    # 3. 尝试串口通信
    print("\n3. 测试串口通信...")
    try:
        ser = serial.Serial(serial_port, 115200, timeout=2)
        print("  串口打开成功")
        
        # 尝试读取数据
        print("  尝试读取传感器数据（最多5秒）...")
        start_time = time.time()
        data_found = False
        
        while time.time() - start_time < 5.0:
            if ser.in_waiting > 0:
                data = ser.readline()
                if data:
                    try:
                        line = data.decode('utf-8', errors='ignore').strip()
                        if line:
                            print(f"    收到数据: {line}")
                            if 'acc:' in line.lower():
                                data_found = True
                                break
                    except Exception as e:
                        print(f"    解码错误: {e}")
            
            time.sleep(0.1)
        
        ser.close()
        
        if data_found:
            print("  成功读取到传感器数据")
        else:
            print("  串口通信正常，但未收到传感器数据格式")
            
    except Exception as e:
        print(f"  串口通信失败: {e}")
    
    print("\n=== 测试完成 ===")

if __name__ == "__main__":
    test_comprehensive()