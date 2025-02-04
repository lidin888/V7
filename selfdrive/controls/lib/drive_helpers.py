import numpy as np
from cereal import log
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
import numpy as np

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2

# EU guidelines
MAX_LATERAL_JERK = 5.0
MAX_VEL_ERR = 5.0

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, desired_curvature_last, model_delay, steer_actuator_delay, t_since_plan, lat_filter):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N

  v_ego = max(MIN_SPEED, v_ego)

  delay = max(0.01, steer_actuator_delay)

  current_curvature_desired = desired_curvature_last

  window_size =  lat_filter
  if window_size > 0:
    #def moving_average(data, window_size):
    #    kernel = np.ones(window_size) / window_size
    #    return np.convolve(data, kernel, mode='same')
    def moving_average(data, window_size):
      data = np.array(data, dtype=float)

      kernel = np.ones(window_size) / window_size
      smoothed = np.convolve(data, kernel, mode='same')

      half_window = window_size // 2
      smoothed[:half_window] = (
          np.cumsum(data[:window_size])[:half_window] / np.arange(1, half_window + 1)
      )
      smoothed[-half_window:] = (
          np.cumsum(data[-window_size:][::-1])[:half_window][::-1] / np.arange(1, half_window + 1)
      )
      return smoothed.tolist()

    curvatures = moving_average(curvatures, window_size)
  
  desired_curvature = np.interp(model_delay + t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], curvatures)
  desired_curvature_ff = np.interp(model_delay + steer_actuator_delay + t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], curvatures)

  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature = np.clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)
  #safe_desired_curvature_ff = clip(desired_curvature_ff,
  #                              current_curvature_desired - max_curvature_rate * DT_MDL,
  #                              current_curvature_desired + max_curvature_rate * DT_MDL)
  return safe_desired_curvature, desired_curvature_ff

def get_lag_adjusted_curvature1(CP, v_ego, psis, curvatures, steer_actuator_delay):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = max(0.01, steer_actuator_delay)

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  psi = np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], psis)
  average_curvature_desired = psi / (v_ego * delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # This is the "desired rate of the setpoint" not an actual desired rate
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature = np.clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)
  return safe_desired_curvature

def clip_curvature(v_ego, prev_curvature, new_curvature):
  new_curvature = np.clip(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  v_ego = max(MIN_SPEED, v_ego)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature = np.clip(new_curvature,
                                prev_curvature - max_curvature_rate * DT_CTRL,
                                prev_curvature + max_curvature_rate * DT_CTRL)

  return safe_desired_curvature


def get_speed_error(modelV2: log.ModelDataV2, v_ego: float) -> float:
  # ToDo: Try relative error, and absolute speed
  if len(modelV2.temporalPose.trans):
    vel_err = np.clip(modelV2.temporalPose.trans[0] - v_ego, -MAX_VEL_ERR, MAX_VEL_ERR)
    return float(vel_err)
  return 0.0
