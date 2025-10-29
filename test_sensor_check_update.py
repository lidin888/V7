
#!/usr/bin/env python3
from openpilot.common.params import Params
from system.manager.manager import check_sensor_connection

params = Params()
print("运行传感器检查测试...")
check_sensor_connection(params)
print("测试完成")

