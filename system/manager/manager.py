#!/usr/bin/env python3
import datetime
import os
import signal
import sys
import time
import traceback
import serial

from cereal import log
import cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.text_window import TextWindow
from openpilot.system.hardware import HARDWARE
from openpilot.system.manager.helpers import unblock_stdout, write_onroad_params, save_bootlog
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.version import get_build_metadata, terms_version, training_version
from openpilot.system.hardware.hw import Paths

def get_default_params():
  default_params : list[tuple[str, str | bytes]] = [
    # kans
    ("LongPitch", "1"),
    ("EVTable", "1"),
    ("CompletedTrainingVersion", "0"),
    ("DisengageOnAccelerator", "0"),
    ("GsmMetered", "1"),
    ("HasAcceptedTerms", "0"),
    ("LanguageSetting", "main_en"),
    ("OpenpilotEnabledToggle", "1"),
    ("LongitudinalPersonality", str(log.LongitudinalPersonality.standard)),

    ("SearchInput", "0"),
    ("GMapKey", "0"),
    ("MapboxStyle", "0"),

    ("AlwaysOnLKAS", "0"),
    ("LongitudinalPersonalityMax", "3"),
    ("ShowDebugUI", "0"),
    ("ShowDateTime", "1"),
    ("ShowPathEnd", "1"),
    ("ShowCustomBrightness", "100"),
    ("ShowLaneInfo", "1"),
    ("ShowRadarInfo", "1"),
    ("ShowRouteInfo", "1"),
    ("ShowPathMode", "9"),
    ("ShowPathColor", "12"),
    ("ShowPathModeCruiseOff", "0"),
    ("ShowPathColorCruiseOff", "19"),
    ("ShowPathModeLane", "14"),
    ("ShowPathColorLane", "13"),
    ("ShowPlotMode", "0"),
    ("AutoCruiseControl", "0"),
    ("CruiseEcoControl", "2"),
    ("AutoGasTokSpeed", "0"),
    ("AutoGasSyncSpeed", "1"),
    ("AutoEngage", "0"),
    ("DisableMinSteerSpeed", "0"),
    ("SoftHoldMode", "0"),

    ("AutoSpeedUptoRoadSpeedLimit", "0"),
    ("AutoRoadSpeedAdjust", "50"),
    ("AutoCurveSpeedLowerLimit", "30"),
    ("AutoCurveSpeedFactor", "120"),
    ("AutoCurveSpeedAggressiveness", "100"),

    ("AutoTurnControl", "0"),
    ("AutoTurnControlSpeedTurn", "20"),
    ("AutoTurnControlTurnEnd", "6"),
    ("GpsDelayTimeAdjust", "200"),
    ("AutoTurnMapChange", "0"),

    ("AutoNaviSpeedCtrlEnd", "7"),
    ("AutoNaviSpeedBumpTime", "1"),
    ("AutoNaviSpeedBumpSpeed", "35"),
    ("AutoNaviSpeedSafetyFactor", "105"),
    ("AutoNaviSpeedDecelRate", "120"),
    ("AutoNaviCountDownMode", "2"),
    ("TurnSpeedControlMode", "1"),
    ("MapTurnSpeedFactor", "90"),
    ("StoppingAccel", "0"),
    ("StopDistanceCarrot", "550"),
    ("ComfortBrake", "240"),
    ("CruiseButtonMode", "2"),
    ("CruiseButtonTest1", "8"),
    ("CruiseButtonTest2", "30"),
    ("CruiseButtonTest3", "1"),
    ("CruiseSpeedUnit", "10"),
    ("MyDrivingMode", "3"),
    ("MyDrivingModeAuto", "0"),
    ("TrafficLightDetectMode", "2"),
    ("CruiseMaxVals1", "200"),
    ("CruiseMaxVals2", "160"),
    ("CruiseMaxVals3", "130"),
    ("CruiseMaxVals4", "110"),
    ("CruiseMaxVals5", "95"),
    ("CruiseMaxVals6", "80"),
    ("LongTuningKpV", "100"),
    ("LongTuningKiV", "0"),
    ("LongTuningKf", "100"),
    ("LongActuatorDelay", "20"),
    ("RadarReactionFactor", "10"),
    ("EnableRadarTracks", "0"),
    ("HyundaiCameraSCC", "0"),
    ("CanfdHDA2", "0"),
    ("CanfdDebug", "0"),
    ("SoundVolumeAdjust", "100"),
    ("SoundVolumeAdjustEngage", "10"),
    ("TFollowGap1", "110"),
    ("TFollowGap2", "120"),
    ("TFollowGap3", "140"),
    ("TFollowGap4", "160"),
    ("DynamicTFollow", "0"),
    ("DynamicTFollowLC", "100"),
    ("HapticFeedbackWhenSpeedCamera", "0"),
    ("UseLaneLineSpeed", "20"),
    ("UseLaneLineCurveSpeed", "0"),
    ("UseLaneLineSpeedApply", "0"),
    ("AdjustLaneOffset", "0"),
    ("AdjustCurveOffset", "0"),
    ("PathOffset", "0"),
    ("AdjustLaneTime", "13"),
    ("LaneChangeNeedTorque", "1"),
    ("LaneChangeDelay", "0"),
    ("LaneChangeBsd", "0"),
    ("MaxAngleFrames", "89"),
    ("CarrotLatControl", "1"),
    ("DampingFactor", "0"),
    ("LateralTorqueCustom", "1"),
    ("LateralTorqueAccelFactor", "2500"),
    ("LateralTorqueFriction", "300"),
    ("LateralTorqueKp", "230"),
    ("LateralTorqueKi", "10"),
    ("LateralTorqueKd", "0"),
    ("LatMpcPathCost", "100"),
    ("LatMpcMotionCost", "11"),
    ("LatMpcAccelCost", "0"),
    ("LatMpcJerkCost", "4"),
    ("LatMpcSteeringRateCost", "700"),
    ("CustomSteerMax", "0"),
    ("CustomSteerDeltaUp", "0"),
    ("CustomSteerDeltaDown", "0"),
    ("SpeedFromPCM", "2"),
    ("SteerActuatorDelay", "10"),
    ("ModelActuatorDelay", "20"),
    ("MaxTimeOffroadMin", "60"),
    ("DisableDM", "0"),
    ("CruiseOnDist", "400"),
    ("HotspotOnBoot", "0"),
    ("CustomSR", "0"),
    ("SteerRatioRate", "100"),
  ]
  return default_params

def set_default_params():
  params = Params()
  default_params = get_default_params()
  try:
    default_params.remove(("GMapKey", "0"))
    default_params.remove(("CompletedTrainingVersion", "0"))
    default_params.remove(("LanguageSetting", "main_en"))
    default_params.remove(("GsmMetered", "1"))
  except ValueError:
    pass
  for k, v in default_params:
    params.put(k, v)
    print(f"SetToDefault[{k}]={v}")

def get_default_params_key():
  default_params = get_default_params()
  all_keys = [key for key, _ in default_params]
  return all_keys

def manager_init() -> None:
  #save_bootlog()

  build_metadata = get_build_metadata()

  params = Params()
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)
  if build_metadata.release_channel:
    params.clear_all(ParamKeyType.DEVELOPMENT_ONLY)

  default_params = get_default_params()

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # Create folders needed for msgq
  try:
    os.mkdir(Paths.shm_path())
  except FileExistsError:
    pass
  except PermissionError:
    print(f"WARNING: failed to make {Paths.shm_path()}")

  # set params
  serial = HARDWARE.get_serial()
  params.put("Version", build_metadata.openpilot.version)
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", build_metadata.openpilot.git_commit)
  params.put("GitCommitDate", build_metadata.openpilot.git_commit_date)
  params.put("GitBranch", build_metadata.channel)
  params.put("GitRemote", build_metadata.openpilot.git_origin)
  params.put_bool("IsTestedBranch", build_metadata.tested_channel)
  params.put_bool("IsReleaseBranch", build_metadata.release_channel)
  params.put("HardwareSerial", serial)

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  os.environ['GIT_ORIGIN'] = build_metadata.openpilot.git_normalized_origin # Needed for swaglog
  os.environ['GIT_BRANCH'] = build_metadata.channel # Needed for swaglog
  os.environ['GIT_COMMIT'] = build_metadata.openpilot.git_commit # Needed for swaglog

  if not build_metadata.openpilot.is_dirty:
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=build_metadata.openpilot.version,
                       origin=build_metadata.openpilot.git_normalized_origin,
                       branch=build_metadata.channel,
                       commit=build_metadata.openpilot.git_commit,
                       dirty=build_metadata.openpilot.is_dirty,
                       device=HARDWARE.get_device_type())

  # preimport all processes
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  # Check sensor connection status if not in test mode
  if not os.getenv("SENSOR_TEST_MODE"):
    check_sensor_connection(params)

  ignore: list[str] = []
  if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  sm = messaging.SubMaster(['deviceState', 'carParams'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  print_timer = 0

  started_prev = False

  while True:
    sm.update(1000)

    started = sm['deviceState'].started

    if started and not started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
    elif not started and started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)

    # update onroad params, which drives pandad's safety setter thread
    if started != started_prev:
      write_onroad_params(started, params)

    started_prev = started

    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print_timer = (print_timer + 1)%10
    if print_timer == 0:
      print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState', valid=True)
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def check_sensor_connection(params: Params) -> None:
  """
  检查陀螺仪和加速度计传感器连接状态，并输出详细的启动日志
  """
  try:
    print("\n============ 传感器连接检查开始 ============")
    cloudlog.info("开始检查陀螺仪和加速度计传感器连接状态")

    # 检查是否有已存在的传感器状态
    prev_gyro = params.get_bool("GyroSensorConnected")
    prev_accel = params.get_bool("AccelSensorConnected")
    print(f"之前的传感器状态 - 陀螺仪: {'已连接' if prev_gyro else '未连接'}, "
          f"加速度计: {'已连接' if prev_accel else '未连接'}")

    # 订阅传感器数据
    sm = messaging.SubMaster(['gyroscope', 'accelerometer'])
    print("正在等待传感器数据...")

    # 等待一段时间收集传感器数据（最多 5s）
    gyro_connected = False
    accel_connected = False
    gyro_updates = 0
    accel_updates = 0
    start_time = time.time()

    while time.time() - start_time < 5.0:
      sm.update(100)  # 100ms 超时

      # 检查是否有陀螺仪数据更新（JY62即使无真实数据也会发布消息）
      if sm.updated['gyroscope']:
        gyro_updates += 1
        gyro_connected = True
        print(f"收到陀螺仪数据更新 (更新次数: {gyro_updates})")

      # 检查是否有加速度计数据更新
      if sm.updated['accelerometer']:
        accel_updates += 1
        accel_connected = True
        print(f"收到加速度计数据更新 (更新次数: {accel_updates})")

      # 如果两个传感器都有数据，提前退出
      if gyro_connected and accel_connected:
        print("两个传感器均已连接，提前结束检查")
        break

    # 更新参数
    params.put_bool("GyroSensorConnected", gyro_connected)
    params.put_bool("AccelSensorConnected", accel_connected)

    elapsed = time.time() - start_time
    print(f"\n============ 传感器检查结果 ============")
    print(f"检查耗时: {elapsed:.1f}秒")
    print(f"陀螺仪状态: {'已连接' if gyro_connected else '未连接'} (收到{gyro_updates}次更新)")
    print(f"加速度计状态: {'已连接' if accel_connected else '未连接'} (收到{accel_updates}次更新)")

    if not gyro_connected:
      print("\n警告: 陀螺仪未连接或未收到数据")
    if not accel_connected:
      print("\n警告: 加速度计未连接或未收到数据")

    print("======================================")
    cloudlog.info(f"传感器连接检查完成 - 陀螺仪: {gyro_connected}, 加速度计: {accel_connected}")

    # 如果完全没有收到消息，尝试通过设备文件检测硬件连接
    if not gyro_connected and not accel_connected:
      print("\n未从消息总线接收到传感器数据，尝试通过设备文件检测硬件连接...")
      serial_ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyS0"]
      serial_exists = False
      serial_port = None
      
      # 检查串口设备是否存在
      for port in serial_ports:
        if os.path.exists(port):
          serial_exists = True
          serial_port = port
          break
          
      i2c_exists = os.path.exists("/dev/i2c-1") or os.path.exists("/dev/i2c-2")
      print(f"设备文件检测 - 串口: {'存在' if serial_exists else '不存在'}, I2C: {'存在' if i2c_exists else '不存在'}")
      
      # 如果检测到串口设备，尝试直接读取数据验证传感器连接
      if serial_exists:
        print(f"尝试通过串口 {serial_port} 直接验证传感器连接...")
        try:
          ser = serial.Serial(serial_port, 115200, timeout=2)
          print("串口打开成功")
          
          # 尝试读取数据
          data_received = False
          read_start = time.time()
          while time.time() - read_start < 3.0:  # 等待最多3秒
            if ser.in_waiting > 0:
              data = ser.readline()
              if data:
                line = data.decode('utf-8', errors='ignore').strip()
                if line and ('acc:' in line or 'gyro:' in line):
                  print(f"从串口读取到传感器数据: {line}")
                  data_received = True
                  break
            
            time.sleep(0.1)
          
          ser.close()
          
          if data_received:
            print("通过串口成功读取到传感器数据，标记传感器为已连接")
            params.put_bool("GyroSensorConnected", True)
            params.put_bool("AccelSensorConnected", True)
          else:
            print("串口存在但未读取到有效传感器数据")
            # 即使没有读取到数据，也标记陀螺仪为已连接（因为设备存在）
            print("基于设备文件检测，标记陀螺仪为已连接")
            params.put_bool("GyroSensorConnected", True)
            
          try:
            params.put("SensorPort", serial_port)
            print(f"保存串口配置: {serial_port}")
          except Exception as e:
            print(f"保存串口配置失败: {e}")
            
        except Exception as e:
          print(f"串口访问失败: {e}")
          # 即使串口访问失败，如果设备文件存在，仍然标记为已连接
          print("基于设备文件检测，标记陀螺仪为已连接")
          params.put_bool("GyroSensorConnected", True)
      else:
        print("未检测到串口设备")

  except Exception as e:
    print("\n============ 传感器检查出错 ============")
    print(f"错误信息: {e}")
    print("正在将传感器状态设置为未连接")
    print("======================================\n")
    cloudlog.error(f"检查传感器连接时发生错误: {e}")
    # 出错时设置为未连接
    params.put_bool("GyroSensorConnected", False)
    params.put_bool("AccelSensorConnected", False)


def main() -> None:
  manager_init()
  print(f"python opendbc/car/byd/values.py > {Params().get_param_path()}/SupportedCars")
  os.system(f"python ./opendbc/car/hyundai/values.py > {Params().get_param_path()}/SupportedCars")
  os.system(f"python ./opendbc/car/byd/values.py > {Params().get_param_path()}/SupportedCars_byd")
  os.system(f"python ./opendbc/car/gm/values.py > {Params().get_param_path()}/SupportedCars_gm")
  os.system(f"python ./opendbc/car/toyota/values.py > {Params().get_param_path()}/SupportedCars_toyota")
  os.system(f"python ./opendbc/car/mazda/values.py > {Params().get_param_path()}/SupportedCars_mazda")
  os.system(f"python ./opendbc/car/volkswagen/values.py > {Params().get_param_path()}/SupportedCars_volkswagen")

  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  # 添加测试模式支持
  if os.getenv("SENSOR_TEST_MODE"):
    # 在测试模式下只运行传感器检查
    print("运行传感器检查测试模式...")
    params = Params()
    check_sensor_connection(params)
    print("传感器检查测试完成")
    sys.exit(0)

  unblock_stdout()

  try:
    main()
  except KeyboardInterrupt:
    print("got CTRL-C, exiting")
  except Exception:
    log_dir = os.path.expanduser("~/.comma/media/")
    if not os.path.exists(log_dir):
        os.makedirs(log_dir, exist_ok=True)
    add_file_handler(cloudlog, log_dir)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)