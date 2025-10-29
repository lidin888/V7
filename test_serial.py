#!/usr/bin/env python3
import serial
import time

def test_serial():
    """测试串口通信"""
    try:
        print("尝试打开串口 /dev/ttyUSB0...")
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
        print("串口打开成功")
        
        print("读取数据（最多10秒）...")
        start_time = time.time()
        data_count = 0
        
        while time.time() - start_time < 10.0:
            if ser.in_waiting > 0:
                data = ser.readline()
                try:
                    line = data.decode('utf-8').strip()
                    print(f"收到数据: {line}")
                    data_count += 1
                    
                    if data_count >= 5:  # 读取5条数据后退出
                        break
                except Exception as e:
                    print(f"解码数据时出错: {e}")
            
            time.sleep(0.1)
        
        ser.close()
        print(f"测试完成，共收到 {data_count} 条数据")
        
    except Exception as e:
        print(f"串口测试失败: {e}")

if __name__ == "__main__":
    test_serial()