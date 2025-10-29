#!/usr/bin/env python3
import subprocess
import time
import sys
import os
from cereal import log
import cereal.messaging as messaging

# Wrapper for sensord_jy62: spawn the binary, parse stdout lines like:
# "acc:9.800000 -0.038281 -0.330176"
# and publish to messaging as 'accelerometer' (and optionally 'gyroscope').

BIN = os.path.join(os.path.dirname(__file__), "bin/sensord_jy62")

PM_TOPICS = ['accelerometer', 'gyroscope']


def parse_acc_line(line: str):
  # Expect: acc:<ax> <ay> <az>
  try:
    line = line.strip()
    if not line.startswith('acc:'):
      return None
    payload = line.split(':', 1)[1].strip()
    parts = payload.split()
    if len(parts) < 3:
      return None
    ax, ay, az = float(parts[0]), float(parts[1]), float(parts[2])
    return (ax, ay, az)
  except Exception:
    return None


def main():
  # Try to create PubMaster with retries in case of transient binding issues
  pm = None
  for _ in range(10):
    try:
      pm = messaging.PubMaster(PM_TOPICS)
      break
    except Exception as e:
      print(f"Warning: failed to create PubMaster: {e}, retrying...")
      time.sleep(0.1)
  if pm is None:
    print("ERROR: could not create PubMaster, exiting")
    sys.exit(1)

  if not os.path.exists(BIN):
    print(f"ERROR: binary not found: {BIN}")
    sys.exit(2)

  print(f"Starting wrapper, running: {BIN}")
  # Try to open serial directly first (avoid spawning binary which may also try to publish)
  serial_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyS0']
  ser = None
  serial_module = None
  try:
    import serial as _serial
    serial_module = _serial
  except Exception:
    serial_module = None

  if serial_module is not None:
    for port in serial_ports:
      try:
        print(f"Trying serial port {port} @115200")
        ser = serial_module.Serial(port, 115200, timeout=1)
        print(f"Opened serial {port}")
        break
      except Exception as e:
        # can't open this port, continue
        print(f"Could not open {port}: {e}")

  if ser is not None:
    try:
      for raw in iter(ser.readline, b''):
        try:
          line = raw.decode('utf-8', errors='ignore').strip()
        except Exception:
          continue
        if not line:
          continue
        print(f"[serial] {line}")
        acc = parse_acc_line(line)
        if acc is not None:
          ax, ay, az = acc
          dat = messaging.new_message('accelerometer', valid=True)
          dat.accelerometer.sensor = 4
          dat.accelerometer.type = 0x10
          dat.accelerometer.timestamp = dat.logMonoTime
          dat.accelerometer.init('acceleration')
          dat.accelerometer.acceleration.v = [ax, ay, az]
          try:
            pm.send('accelerometer', dat)
          except Exception as e:
            print(f"Warning: failed to send accelerometer msg: {e}")

          datg = messaging.new_message('gyroscope', valid=True)
          datg.gyroscope.sensor = 5
          datg.gyroscope.type = 0x10
          datg.gyroscope.timestamp = datg.logMonoTime
          datg.gyroscope.init('gyroUncalibrated')
          datg.gyroscope.gyroUncalibrated.v = [0.0, 0.0, 0.0]
          try:
            pm.send('gyroscope', datg)
          except Exception as e:
            print(f"Warning: failed to send gyroscope msg: {e}")
    except KeyboardInterrupt:
      print('Wrapper serial interrupted')
    finally:
      try:
        ser.close()
      except Exception:
        pass
  else:
    # Fallback: try to open serial via stty+cat to avoid spawning the binary
    cat_proc = None
    cat_port = None
    for port in serial_ports:
      if os.path.exists(port):
        cat_port = port
        break
    if cat_port is not None:
      try:
        cmd = f"stty -F {cat_port} 115200 raw -echo; exec cat {cat_port}"
        print(f"Falling back to cat on {cat_port}")
        cat_proc = subprocess.Popen(["/bin/sh", "-c", cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
      except Exception as e:
        print(f"Could not spawn cat for {cat_port}: {e}")

    # If cat failed, as last resort run the binary (may conflict with messaging)
    if cat_proc is None:
      print("Falling back to running binary (may conflict) :", BIN)
      p = subprocess.Popen([BIN], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    else:
      p = cat_proc
    try:
      for raw in p.stdout:
        line = raw.strip()
        if not line:
          continue
        # print to wrapper stdout for debug
        print(f"[sensord_jy62] {line}")

        acc = parse_acc_line(line)
        if acc is not None:
          ax, ay, az = acc
          # publish accelerometer
          dat = messaging.new_message('accelerometer', valid=True)
          dat.accelerometer.sensor = 4
          dat.accelerometer.type = 0x10
          dat.accelerometer.timestamp = dat.logMonoTime
          dat.accelerometer.init('acceleration')
          dat.accelerometer.acceleration.v = [ax, ay, az]
          try:
            pm.send('accelerometer', dat)
          except Exception as e:
            print(f"Warning: failed to send accelerometer msg: {e}")

          # also publish a simple gyroscope message with zeros if no gyro reading available
          datg = messaging.new_message('gyroscope', valid=True)
          datg.gyroscope.sensor = 5
          datg.gyroscope.type = 0x10
          datg.gyroscope.timestamp = datg.logMonoTime
          datg.gyroscope.init('gyroUncalibrated')
          # no gyro in printed output; set zeros
          datg.gyroscope.gyroUncalibrated.v = [0.0, 0.0, 0.0]
          try:
            pm.send('gyroscope', datg)
          except Exception as e:
            print(f"Warning: failed to send gyroscope msg: {e}")

    except KeyboardInterrupt:
      print('Wrapper interrupted, killing child')
      p.terminate()
      try:
        p.wait(timeout=1)
      except Exception:
        p.kill()
    except Exception as e:
      print('Wrapper error:', e)
      p.terminate()
    finally:
      if p.poll() is None:
        p.terminate()


if __name__ == '__main__':
  main()
