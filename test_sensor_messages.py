#!/usr/bin/env python3

import cereal.messaging as messaging
import time

def test_sensor_messages():
    """测试传感器消息是否正确发布"""
    # 创建订阅者来检查消息
    sm = messaging.SubMaster(['accelerometer', 'gyroscope'])
    print('等待传感器消息...')

    for i in range(20):
        sm.update(1000)  # 等待最多1秒
        updated = False
        
        if sm.updated['accelerometer']:
            acc = sm['accelerometer'].acceleration.v
            print(f'收到accelerometer数据: x={acc[0]:.3f}, y={acc[1]:.3f}, z={acc[2]:.3f}')
            updated = True
            
        if sm.updated['gyroscope']:
            gyro = sm['gyroscope'].gyroUncalibrated.v
            print(f'收到gyroscope数据: x={gyro[0]:.3f}, y={gyro[1]:.3f}, z={gyro[2]:.3f}')
            updated = True
            
        if not updated:
            print(f'第{i+1}次检查: 未收到传感器数据')
            
        time.sleep(0.1)

    print('检查完成')

if __name__ == '__main__':
    test_sensor_messages()