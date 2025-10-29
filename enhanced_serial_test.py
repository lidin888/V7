#!/usr/bin/env python3
import serial
import time

def enhanced_serial_test():
    """增强版串口测试"""
    print("=== 增强版串口测试 ===")
    
    port = "/dev/ttyUSB0"
    baudrate = 115200
    
    print(f"测试串口: {port}")
    print(f"波特率: {baudrate}")
    
    try:
        # 尝试不同的串口配置
        configs = [
            {"baudrate": 115200, "bytesize": serial.EIGHTBITS, "parity": serial.PARITY_NONE, "stopbits": serial.STOPBITS_ONE},
            {"baudrate": 9600, "bytesize": serial.EIGHTBITS, "parity": serial.PARITY_NONE, "stopbits": serial.STOPBITS_ONE},
            {"baudrate": 115200, "bytesize": serial.EIGHTBITS, "parity": serial.PARITY_EVEN, "stopbits": serial.STOPBITS_ONE},
        ]
        
        for i, config in enumerate(configs):
            print(f"\n尝试配置 {i+1}: {config}")
            
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = config["baudrate"]
            ser.bytesize = config["bytesize"]
            ser.parity = config["parity"]
            ser.stopbits = config["stopbits"]
            ser.timeout = 2
            ser.xonxoff = False
            ser.rtscts = False
            ser.dsrdtr = False
            
            try:
                ser.open()
                print(f"  串口打开成功")
                
                # 清空输入缓冲区
                ser.flushInput()
                
                # 等待一段时间接收数据
                print(f"  等待数据（最多5秒）...")
                start_time = time.time()
                data_count = 0
                
                while time.time() - start_time < 5.0:
                    if ser.in_waiting > 0:
                        data = ser.read(ser.in_waiting)
                        if data:
                            print(f"    收到原始数据: {data}")
                            try:
                                line = data.decode('utf-8', errors='ignore')
                                if line.strip():
                                    print(f"    解码后数据: {repr(line)}")
                                    data_count += 1
                            except Exception as e:
                                print(f"    解码错误: {e}")
                                
                    time.sleep(0.1)
                
                ser.close()
                print(f"  收到 {data_count} 条数据")
                
                if data_count > 0:
                    print(f"  配置 {i+1} 成功!")
                    return config  # 返回成功的配置
                    
            except Exception as e:
                print(f"  错误: {e}")
                if ser.is_open:
                    ser.close()
                    
        print("\n所有配置都失败了")
        return None
        
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        return None

if __name__ == "__main__":
    enhanced_serial_test()