#!/usr/bin/env python3
import time
import cereal.messaging as messaging

def check_published_data():
    """检查传感器数据是否正确发布到消息总线"""
    print("检查传感器数据是否正确发布到消息总线...")
    
    # 订阅传感器数据
    sm = messaging.SubMaster(['accelerometer', 'gyroscope'])
    
    print("等待传感器数据（最多10秒）...")
    start_time = time.time()
    accelerometer_count = 0
    gyroscope_count = 0
    
    while time.time() - start_time < 10.0:
        sm.update(100)  # 100ms 超时
        
        if sm.updated['accelerometer']:
            accelerometer_count += 1
            acc = sm['accelerometer'].acceleration.v
            print(f"加速度计数据 #{accelerometer_count}: x={acc[0]:.3f}, y={acc[1]:.3f}, z={acc[2]:.3f}")
            
        if sm.updated['gyroscope']:
            gyroscope_count += 1
            gyro = sm['gyroscope'].gyroUncalibrated.v
            print(f"陀螺仪数据 #{gyroscope_count}: x={gyro[0]:.3f}, y={gyro[1]:.3f}, z={gyro[2]:.3f}")
            
        time.sleep(0.1)
    
    print(f"\n检查结果:")
    print(f"  收到加速度计数据: {accelerometer_count} 条")
    print(f"  收到陀螺仪数据: {gyroscope_count} 条")
    
    if accelerometer_count > 0 or gyroscope_count > 0:
        print("传感器数据正常发布到消息总线")
    else:
        print("未收到传感器数据")

if __name__ == "__main__":
    check_published_data()