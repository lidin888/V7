import numpy as np
from typing import Any

from cereal import log
from openpilot.common.transformations.orientation import rot_from_euler, euler_from_rot

# Global candidate name pairs for measurements. Tests or runtime debug can
# register new candidates via `register_measurement_candidate` so the parser
# remains extensible for different capnp/schema variants.
MEASUREMENT_CANDIDATES: list[tuple[tuple[str, ...], tuple[str, ...]]] = [
  (("x", "y", "z"), ("xStd", "yStd", "zStd")),
  (("x", "y", "z"), ("x_std", "y_std", "z_std")),
  (("n", "e", "d"), ("nStd", "eStd", "dStd")),
  (("north", "east", "down"), ("northStd", "eastStd", "downStd")),
  (("north", "east", "down"), ("north_std", "east_std", "down_std")),
]


def register_measurement_candidate(coord_names: tuple[str, ...], std_names: tuple[str, ...]) -> None:
  """Register a new candidate name pair (coords, stds).

  New candidates are inserted at the front so tests/runtime-registered
  names take precedence over built-in defaults.
  """
  MEASUREMENT_CANDIDATES.insert(0, (tuple(coord_names), tuple(std_names)))


def rotate_cov(rot_matrix, cov_in):
  return rot_matrix @ cov_in @ rot_matrix.T


def rotate_std(rot_matrix, std_in):
  return np.sqrt(np.diag(rotate_cov(rot_matrix, np.diag(std_in**2))))


class NPQueue:
  def __init__(self, maxlen: int, rowsize: int) -> None:
    self.maxlen = maxlen
    self.arr = np.empty((0, rowsize))

  def __len__(self) -> int:
    return len(self.arr)

  def append(self, pt: list[float]) -> None:
    if len(self.arr) < self.maxlen:
      self.arr = np.append(self.arr, [pt], axis=0)
    else:
      self.arr[:-1] = self.arr[1:]
      self.arr[-1] = pt


class PointBuckets:
  def __init__(self, x_bounds: list[tuple[float, float]], min_points: list[float], min_points_total: int, points_per_bucket: int, rowsize: int) -> None:
    self.x_bounds = x_bounds
    self.buckets = {bounds: NPQueue(maxlen=points_per_bucket, rowsize=rowsize) for bounds in x_bounds}
    self.buckets_min_points = dict(zip(x_bounds, min_points, strict=True))
    self.min_points_total = min_points_total

  def __len__(self) -> int:
    return sum([len(v) for v in self.buckets.values()])

  def is_valid(self) -> bool:
    individual_buckets_valid = all(len(v) >= min_pts for v, min_pts in zip(self.buckets.values(), self.buckets_min_points.values(), strict=True))
    total_points_valid = self.__len__() >= self.min_points_total
    return individual_buckets_valid and total_points_valid

  def is_calculable(self) -> bool:
    return all(len(v) > 0 for v in self.buckets.values())

  def add_point(self, x: float, y: float) -> None:
    raise NotImplementedError

  def get_points(self, num_points: int = None) -> Any:
    points = np.vstack([x.arr for x in self.buckets.values()])
    if num_points is None:
      return points
    return points[np.random.choice(np.arange(len(points)), min(len(points), num_points), replace=False)]

  def load_points(self, points: list[list[float]]) -> None:
    for point in points:
      self.add_point(*point)


class ParameterEstimator:
  """ Base class for parameter estimators """
  def reset(self) -> None:
    raise NotImplementedError

  def handle_log(self, t: int, which: str, msg: log.Event) -> None:
    raise NotImplementedError

  def get_msg(self, valid: bool, with_points: bool) -> log.Event:
    raise NotImplementedError


class Measurement:
  x, y, z = (property(lambda self: self.xyz[0]), property(lambda self: self.xyz[1]), property(lambda self: self.xyz[2]))
  x_std, y_std, z_std = (property(lambda self: self.xyz_std[0]), property(lambda self: self.xyz_std[1]), property(lambda self: self.xyz_std[2]))
  roll, pitch, yaw = x, y, z
  roll_std, pitch_std, yaw_std = x_std, y_std, z_std

  def __init__(self, xyz: np.ndarray, xyz_std: np.ndarray):
    self.xyz: np.ndarray = xyz
    self.xyz_std: np.ndarray = xyz_std

  @classmethod
  def from_measurement_xyz(cls, measurement: log.LivePose.XYZMeasurement) -> 'Measurement':
    """
    Robustly construct a Measurement from a capnp measurement that may use different
    field names depending on schema versions. Tries common name variants and
    iterable forms. If none match, raises AttributeError with available fields.
    """
    # helper to attempt attribute reads and return numpy arrays or None
    def _try_attrs(names):
      try:
        vals = [getattr(measurement, n) for n in names]
        return np.array(vals, dtype=float)
      except Exception:
        return None

    # Use the module-level candidate list so users/tests can register new names
    for coord_names, std_names in MEASUREMENT_CANDIDATES:
      coords = _try_attrs(coord_names)
      stds = _try_attrs(std_names)
      if coords is not None and stds is not None:
        return cls(xyz=coords, xyz_std=stds)

    # try iterable forms: 'xyz' and 'xyzStd' or 'values' and 'stds'
    try:
      if hasattr(measurement, "xyz"):
        coords = np.array(list(measurement.xyz), dtype=float)
        if hasattr(measurement, "xyzStd"):
          stds = np.array(list(measurement.xyzStd), dtype=float)
          return cls(xyz=coords, xyz_std=stds)
        if hasattr(measurement, "xyz_std"):
          stds = np.array(list(measurement.xyz_std), dtype=float)
          return cls(xyz=coords, xyz_std=stds)
    except Exception:
      # fall through to other attempts
      pass

    try:
      if hasattr(measurement, "values"):
        coords = np.array(list(measurement.values), dtype=float)
        if hasattr(measurement, "stds"):
          stds = np.array(list(measurement.stds), dtype=float)
          return cls(xyz=coords, xyz_std=stds)
    except Exception:
      pass

    # As a last-resort, try any three numeric attributes heuristically
    attrs = []
    try:
      attrs = [a for a in dir(measurement) if not a.startswith("_")]
    except Exception:
      attrs = []

    numeric_attrs = []
    for a in attrs:
      try:
        v = getattr(measurement, a)
        # consider scalars or single-item lists
        if isinstance(v, (int, float)):
          numeric_attrs.append((a, float(v)))
        else:
          # capnp lists are iterable; ignore multi-length iterables here
          pass
      except Exception:
        continue

    if len(numeric_attrs) >= 3:
      # pick first three found (best-effort)
      coords = np.array([numeric_attrs[i][1] for i in range(3)], dtype=float)
      stds = np.full(3, np.nan)
      return cls(xyz=coords, xyz_std=stds)

    # nothing matched: raise informative error
    try:
      available = [a for a in dir(measurement) if not a.startswith("_")]
    except Exception:
      available = ["<could not list attributes>"]

    raise AttributeError(f"measurement has no x/y/z fields; available fields: {available}")


class Pose:
  def __init__(self, orientation: Measurement, velocity: Measurement, acceleration: Measurement, angular_velocity: Measurement):
    self.orientation = orientation
    self.velocity = velocity
    self.acceleration = acceleration
    self.angular_velocity = angular_velocity

  @classmethod
  def from_live_pose(cls, live_pose: log.LivePose) -> 'Pose':
    return Pose(
      orientation=Measurement.from_measurement_xyz(live_pose.orientationNED),
      velocity=Measurement.from_measurement_xyz(live_pose.velocityDevice),
      acceleration=Measurement.from_measurement_xyz(live_pose.accelerationDevice),
      angular_velocity=Measurement.from_measurement_xyz(live_pose.angularVelocityDevice)
    )


class PoseCalibrator:
  def __init__(self):
    self.calib_valid = False
    self.calib_from_device = np.eye(3)

  def _transform_calib_from_device(self, meas: Measurement):
    new_xyz = self.calib_from_device @ meas.xyz
    new_xyz_std = rotate_std(self.calib_from_device, meas.xyz_std)
    return Measurement(new_xyz, new_xyz_std)

  def _ned_from_calib(self, orientation: Measurement):
    ned_from_device = rot_from_euler(orientation.xyz)
    ned_from_calib = ned_from_device @ self.calib_from_device.T
    ned_from_calib_euler_meas = Measurement(euler_from_rot(ned_from_calib), np.full(3, np.nan))
    return ned_from_calib_euler_meas

  def build_calibrated_pose(self, pose: Pose) -> Pose:
    ned_from_calib_euler = self._ned_from_calib(pose.orientation)
    angular_velocity_calib = self._transform_calib_from_device(pose.angular_velocity)
    acceleration_calib = self._transform_calib_from_device(pose.acceleration)
    velocity_calib = self._transform_calib_from_device(pose.angular_velocity)

    return Pose(ned_from_calib_euler, velocity_calib, acceleration_calib, angular_velocity_calib)

  def feed_live_calib(self, live_calib: log.LiveCalibrationData):
    calib_rpy = np.array(live_calib.rpyCalib)
    device_from_calib = rot_from_euler(calib_rpy)
    self.calib_from_device = device_from_calib.T
    self.calib_valid = live_calib.calStatus == log.LiveCalibrationData.Status.calibrated
