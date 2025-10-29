
#!/usr/bin/env python3
import cereal.messaging as messaging
import time

print("测试消息总线连接...")
try:
    pm = messaging.PubMaster(["accelerometer"])
    print("PubMaster创建成功")
    
    # 创建一条测试消息
    dat = messaging.new_message("accelerometer", valid=True)
    dat.accelerometer.sensor = 4
    dat.accelerometer.type = 0x10
    dat.accelerometer.timestamp = dat.logMonoTime
    dat.accelerometer.init("acceleration")
    dat.accelerometer.acceleration.v = [1.0, 2.0, 3.0]
    dat.accelerometer.acceleration.status = True
    
    # 发送消息
    pm.send("accelerometer", dat)
    print("测试消息发送成功")
    
except Exception as e:
    print(f"错误: {e}")

print("测试完成")

