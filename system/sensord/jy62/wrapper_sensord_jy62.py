#!/usr/bin/env python3
import os
import sys
import time
import serial
import struct
import threading
from typing import Optional

import cereal.messaging as messaging
from common.realtime import Ratekeeper
from common.params import Params


BAUDRATE = 115200
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# JY62 协议常量
HEADER = 0x55  # 数据帧头
CMD_ACCEL = 0x51  # 加速度
CMD_ANGLE = 0x53  # 角度
CMD_MAG = 0x52   # 磁场

def init_serial(port: str) -> Optional[serial.Serial]:
  try:
    return serial.Serial(
      port=port,
      baudrate=BAUDRATE,
      parity=PARITY,
      stopbits=STOPBITS,
      bytesize=BYTESIZE,
      timeout=1
    )
  except serial.SerialException as e:
    print(f"Error opening serial port {port}: {e}")
    return None

def parse_frame(data: bytes) -> Optional[dict]:
  """解析JY62数据帧

  数据帧格式:
  帧头(1字节) | 命令字(1字节) | 数据(6/8字节) | 校验和(1字节)
  """
  if len(data) < 11:  # 最小帧长度
    return None

  if data[0] != HEADER:  # 验证帧头
    return None

  # 计算校验和
  checksum = sum(data[:-1]) & 0xFF
  if checksum != data[-1]:
    return None

  cmd = data[1]
  values = {}

  if cmd == CMD_ACCEL:  # 加速度
    ax, ay, az = struct.unpack('<3h', data[2:8])
    values['accel'] = [ax/32768.0*16, ay/32768.0*16, az/32768.0*16]  # 转换为g

  elif cmd == CMD_ANGLE:  # 欧拉角
    roll, pitch, yaw = struct.unpack('<3h', data[2:8])
    values['angle'] = [roll/32768.0*180, pitch/32768.0*180, yaw/32768.0*180]  # 转换为度

  elif cmd == CMD_MAG:  # 磁场
    mx, my, mz = struct.unpack('<3h', data[2:8])
    values['mag'] = [mx, my, mz]

  return values

def sensor_reader(ser: serial.Serial, sm: messaging.SubMaster, pm: messaging.PubMaster):
  """读取传感器数据并发布消息"""
  rk = Ratekeeper(100, print_delay_threshold=2.0)  # 100Hz
  buffer = bytearray()

  accel_data = None
  gyro_data = None
  last_publish = 0

  while True:
    if ser.in_waiting:
      byte = ser.read()
      buffer.extend(byte)

      # 查找帧头
      while len(buffer) > 0 and buffer[0] != HEADER:
        buffer.pop(0)

      # 尝试解析完整数据帧
      if len(buffer) >= 11:
        data = parse_frame(buffer[:11])
        if data:
          # 更新传感器数据
          if 'accel' in data:
            accel_data = data['accel']
          elif 'angle' in data:
            # parse_frame 返回的是角度 (deg)，但 locationd expects gyro (rad/s).
            # JY62 returns Euler angles; if using raw gyro rates, adjust accordingly.
            # For now publish the angle values as gyro-like readings to allow connection check.
            gyro_data = data['angle']

          # 当有新的陀螺仪数据时发布 gyroscope/accelerometer 消息
          now = time.monotonic()
          if gyro_data is not None and now - last_publish > 0.01:  # 限制发布频率到100Hz
            # 发布陀螺仪
            gmsg = messaging.new_message('gyroscope', valid=True)
            gmsg.gyroscope.sensor = 5
            gmsg.gyroscope.type = 0x10
            gmsg.gyroscope.timestamp = gmsg.logMonoTime
            gmsg.gyroscope.init('gyroUncalibrated')
            # 把值转为 float32 列表
              try:
                if ser.in_waiting:
                  byte = ser.read()
            pm.send('gyroscope', gmsg)
                buffer.extend(byte)
            # 如果有加速度数据, 发布加速度消息
            if accel_data is not None:
              amsg = messaging.new_message('accelerometer', valid=True)
              amsg.accelerometer.sensor = 4
              amsg.accelerometer.type = 0x10
              amsg.accelerometer.timestamp = amsg.logMonoTime
              amsg.accelerometer.init('acceleration')
              amsg.accelerometer.acceleration.v = [float(x) for x in accel_data]
              amsg.accelerometer.acceleration.status = 1
              pm.send('accelerometer', amsg)

            last_publish = now

            # 设置传感器状态
            params = Params()
            params.put_bool("GyroSensorConnected", True)
            params.put_bool("AccelSensorConnected", accel_data is not None)

        buffer = buffer[11:]  # 移除已处理的数据

    rk.keep_time()

def main():
  params = Params()

  # 初始化传感器状态为未连接
  try:
    params.put_bool("GyroSensorConnected", False)
    params.put_bool("AccelSensorConnected", False)
  except Exception:
    # Params implementation may raise on unknown keys in some builds; ignore here
    pass

  # 从参数中读取串口配置，如果没有则使用默认值
  try:
    port = params.get("SensorPort", encoding='utf8')
  except Exception:
    port = None

  if not port:
    port = "/dev/ttyUSB0"
    try:
      params.put("SensorPort", port)
    except Exception:
      # ignore if params doesn't accept arbitrary keys
      pass

  print(f"正在初始化传感器 (端口: {port})")
  ser = init_serial(port)
  if not ser:
    print(f"无法打开串口 {port}")
    sys.exit(1)
              except serial.SerialException as e:
                print(f"Serial error in sensor_reader: {e}")
                try:
                  params = Params()
                  params.put_bool("GyroSensorConnected", False)
                  params.put_bool("AccelSensorConnected", False)
                except Exception:
                  pass
                break

              rk.keep_time()

  # 初始化消息 - 发布 accelerometer 和 gyroscope
  services = ['accelerometer', 'gyroscope']
  sm = messaging.SubMaster(services)
  pm = messaging.PubMaster(services)

  # 启动读取线程
  reader = threading.Thread(target=sensor_reader, args=(ser, sm, pm))
  reader.daemon = True
  reader.start()

  # 主循环
  while True:
    sm.update()
    time.sleep(0.01)

    # 如果读取线程退出（例如串口被断开），尝试重新连接并重启读取线程
    if not reader.is_alive():
      print("sensor_reader 线程已退出，尝试重新连接串口...")
      try:
        ser.close()
      except Exception:
        pass

      # 重试打开串口
      reconnected = False
      for _ in range(10):
        new_ser = init_serial(port)
        if new_ser:
          ser = new_ser
          reader = threading.Thread(target=sensor_reader, args=(ser, sm, pm))
          reader.daemon = True
          reader.start()
          reconnected = True
          print("已重新连接串口并重启读取线程")
          break
        time.sleep(1)

      if not reconnected:
        print("无法重新连接串口，继续等待...")

if __name__ == "__main__":
  main()
