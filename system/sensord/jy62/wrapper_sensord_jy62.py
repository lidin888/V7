#!/usr/bin/env python3
"""JY62 serial wrapper: parse frames and publish gyroscope/accelerometer.

This version logs additional diagnostics and supports robust error handling.
"""
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
from common.swaglog import cloudlog

BAUDRATE = 115200
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# JY62 协议常量
HEADER = 0x55
CMD_ACCEL = 0x51
CMD_ANGLE = 0x53
CMD_MAG = 0x52

def _get_behavior(params: Params) -> str:
  """获取传感器读取行为配置"""
  try:
    behavior = params.get('SensorReadBehavior', encoding='utf8')
    if behavior in ['continue', 'retry', 'exit']:
      return behavior
  except Exception:
    pass
  
  # 从环境变量获取
  behavior = os.environ.get('SENSOR_READ_BEHAVIOR', 'continue')
  if behavior not in ['continue', 'retry', 'exit']:
    behavior = 'continue'
  
  return behavior

def init_serial(port: str) -> Optional[serial.Serial]:
  """初始化串口连接"""
  try:
    print(f"正在尝试打开串口: {port}")
    ser = serial.Serial(
      port=port,
      baudrate=BAUDRATE,
      parity=PARITY,
      stopbits=STOPBITS,
      bytesize=BYTESIZE,
      timeout=1.0
    )
    
    if ser.is_open:
      print(f"串口 {port} 打开成功")
      cloudlog.info(f"JY62传感器串口 {port} 连接成功")
      return ser
    else:
      print(f"串口 {port} 打开失败")
      return None
      
  except Exception as e:
    print(f"无法打开串口 {port}: {e}")
    cloudlog.warning(f"JY62传感器串口 {port} 连接失败: {e}")
    return None

def parse_jy62_frame(data: bytes) -> tuple:
  """解析JY62数据帧
  
  Returns:
    tuple: (accel_data, gyro_data, angle_data) 或 (None, None, None)
  """
  if len(data) < 11:
    return None, None, None
    
  if data[0] != HEADER:
    return None, None, None
    
  cmd = data[1]
  payload = data[2:10]
  checksum = data[10]
  
  # 验证校验和
  calc_checksum = (HEADER + cmd + sum(payload)) & 0xFF
  if calc_checksum != checksum:
    return None, None, None
  
  # 解析数据
  values = struct.unpack('<4h', payload)
  
  if cmd == CMD_ACCEL:
    # 加速度数据 (单位: g)
    ax = values[0] / 32768.0 * 16.0
    ay = values[1] / 32768.0 * 16.0
    az = values[2] / 32768.0 * 16.0
    return [ax, ay, az], None, None
    
  elif cmd == CMD_ANGLE:
    # 角速度数据 (单位: rad/s)
    gx = values[0] / 32768.0 * 2000.0 * 3.14159 / 180.0
    gy = values[1] / 32768.0 * 2000.0 * 3.14159 / 180.0
    gz = values[2] / 32768.0 * 2000.0 * 3.14159 / 180.0
    return None, [gx, gy, gz], None
    
  elif cmd == CMD_MAG:
    # 磁力计数据 (暂不使用)
    return None, None, values[:3]
    
  return None, None, None

def sensor_reader(ser: serial.Serial, pm: messaging.PubMaster, behavior: str):
  """传感器数据读取线程"""
  print("JY62传感器读取线程已启动")
  cloudlog.info("JY62传感器读取线程启动")
  
  buf = bytearray()
  rk = Ratekeeper(100)  # 100Hz
  last_pub = 0
  
  try:
    params = Params()
    
    while True:
      try:
        # 读取串口数据
        if ser.in_waiting > 0:
          data = ser.read(ser.in_waiting)
          if data:
            buf.extend(data)
          else:
            # 空读取处理
            if behavior == 'exit':
              print("收到空数据，退出读取线程")
              break
            elif behavior == 'retry':
              print("收到空数据，等待重试...")
              time.sleep(0.1)
              continue
            # 'continue' 行为：继续正常处理
        
        # 处理缓冲区中的数据
        while len(buf) >= 11:
          # 查找帧头
          header_idx = buf.find(HEADER)
          if header_idx == -1:
            buf.clear()
            cloudlog.debug("JY62数据帧头未找到，清空缓冲区")
            break
            
          if header_idx > 0:
            buf = buf[header_idx:]
            cloudlog.debug(f"JY62数据帧头位置: {header_idx}, 丢弃前{header_idx}字节")
            
          if len(buf) < 11:
            cloudlog.debug(f"JY62缓冲区数据不足: {len(buf)} < 11")
            break
            
          # 解析数据帧
          frame = buf[:11]
          accel, gyro, angle = parse_jy62_frame(frame)
          
          now = time.time()
          if now - last_pub > 0.01:  # 限制发布频率到100Hz
            
            # 发布陀螺仪数据
            if gyro is not None:
              g = messaging.new_message('gyroscope', valid=True)
              g.gyroscope.sensor = 4
              g.gyroscope.type = 0x10
              g.gyroscope.timestamp = g.logMonoTime
              g.gyroscope.init('gyroUncalibrated')
              g.gyroscope.gyroUncalibrated.v = [float(x) for x in gyro]
              g.gyroscope.gyroUncalibrated.status = 1
              pm.send('gyroscope', g)
              cloudlog.debug(f"发布陀螺仪数据: {gyro}")
            
            # 发布加速度计数据
            if accel is not None:
              a = messaging.new_message('accelerometer', valid=True)
              a.accelerometer.sensor = 4
              a.accelerometer.type = 0x10
              a.accelerometer.timestamp = a.logMonoTime
              a.accelerometer.init('acceleration')
              a.accelerometer.acceleration.v = [float(x) for x in accel]
              a.accelerometer.acceleration.status = 1
              pm.send('accelerometer', a)
              cloudlog.debug(f"发布加速度计数据: {accel}")
            
            last_pub = now
            
            # 更新连接状态
            try:
              params.put_bool('GyroSensorConnected', True)
              params.put_bool('AccelSensorConnected', accel is not None)
              cloudlog.debug(f"更新传感器连接状态: 陀螺仪=已连接, 加速度计={'已连接' if accel is not None else '未连接'}")
            except Exception as e:
              cloudlog.error(f"更新传感器连接状态失败: {e}")
          
          # 移除已处理的帧
          buf = buf[11:]
        
      except serial.SerialException as e:
        print(f"串口读取错误: {e}")
        cloudlog.error(f"JY62传感器串口读取错误: {e}")
        try:
          params.put_bool('GyroSensorConnected', False)
          params.put_bool('AccelSensorConnected', False)
          cloudlog.warning("JY62传感器连接状态已更新为未连接")
        except Exception as e:
          cloudlog.error(f"更新传感器连接状态失败: {e}")
        break
      except Exception as e:
        print(f"传感器读取异常: {e}")
        cloudlog.error(f"JY62传感器读取异常: {e}")
        if behavior == 'exit':
          cloudlog.warning("JY62传感器读取线程因异常退出")
          break
        time.sleep(0.1)
        cloudlog.debug("JY62传感器读取线程等待重试...")
      
      rk.keep_time()
      
  except Exception as e:
    print(f"传感器读取线程异常退出: {e}")
    cloudlog.error(f"JY62传感器读取线程异常退出: {e}")
  finally:
    print("JY62传感器读取线程已退出")
    cloudlog.info("JY62传感器读取线程退出")

def main():
  """主函数"""
  print("============ JY62传感器启动 ============")
  cloudlog.info("JY62传感器进程启动")
  
  params = Params()
  
  # 初始化传感器状态
  try:
    params.put_bool('GyroSensorConnected', False)
    params.put_bool('AccelSensorConnected', False)
  except Exception:
    pass
  
  # 获取串口配置
  try:
    port = params.get('SensorPort')
    if isinstance(port, bytes):
      port = port.decode('utf-8')
  except Exception:
    port = None
  
  if not port:
    # 尝试常见的串口设备
    for test_port in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyS0']:
      if os.path.exists(test_port):
        port = test_port
        break
    
    if not port:
      port = '/dev/ttyUSB0'  # 默认端口
    
    try:
      params.put('SensorPort', port.encode('utf-8'))
      print(f"已保存串口配置: {port}")
    except Exception as e:
      print(f"保存串口配置失败: {e}")
      # 尝试直接写入参数文件
      try:
        with open('/data/params/d/SensorPort', 'w') as f:
          f.write(port)
        print(f"直接写入参数文件成功: {port}")
      except Exception as e:
        print(f"直接写入参数文件失败: {e}")
  
  print(f"使用串口设备: {port}")
  
  # 获取传感器读取行为配置
  behavior = _get_behavior(params)
  print(f"传感器读取行为: {behavior}")
  
  # 初始化串口
  ser = init_serial(port)
  if not ser:
    print(f"无法打开串口 {port}，进程退出")
    cloudlog.error(f"JY62传感器无法打开串口 {port}")
    sys.exit(1)
  
  # 初始化消息发布器
  services = ['accelerometer', 'gyroscope']
  pm = messaging.PubMaster(services)
  
  print("启动传感器读取线程...")
  
  # 启动传感器读取线程
  reader = threading.Thread(target=sensor_reader, args=(ser, pm, behavior))
  reader.daemon = True
  reader.start()
  
  print("JY62传感器进程运行中...")
  print("=====================================")
  
  # 主循环：监控读取线程状态
  try:
    while True:
      time.sleep(1.0)
      
      if not reader.is_alive():
        print('传感器读取线程已退出，尝试重新连接...')
        cloudlog.warning('JY62传感器读取线程退出，尝试重新连接')
        
        try:
          ser.close()
        except Exception:
          pass
        
        # 尝试重新连接
        reconnected = False
        for attempt in range(10):
          print(f"重连尝试 {attempt + 1}/10...")
          new_ser = init_serial(port)
          if new_ser:
            ser = new_ser
            reader = threading.Thread(target=sensor_reader, args=(ser, pm, behavior))
            reader.daemon = True
            reader.start()
            reconnected = True
            print('串口重连成功，读取线程已重启')
            cloudlog.info('JY62传感器串口重连成功')
            break
          time.sleep(2)
        
        if not reconnected:
          print('无法重新连接串口，进程退出')
          cloudlog.error('JY62传感器无法重新连接串口')
          break
          
  except KeyboardInterrupt:
    print("\n收到中断信号，正在关闭JY62传感器进程...")
    cloudlog.info("JY62传感器进程收到中断信号")
  except Exception as e:
    print(f"主循环异常: {e}")
    cloudlog.error(f"JY62传感器主循环异常: {e}")
  finally:
    try:
      ser.close()
    except Exception:
      pass
    print("JY62传感器进程已退出")
    cloudlog.info("JY62传感器进程退出")

if __name__ == '__main__':
  main()