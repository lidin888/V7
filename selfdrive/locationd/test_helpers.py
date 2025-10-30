import numpy as np

from openpilot.selfdrive.locationd.helpers import Measurement, register_measurement_candidate


class Simple:
  def __init__(self, **kwargs):
    for k, v in kwargs.items():
      setattr(self, k, v)


def test_xyz():
  m = Simple(x=1.0, y=2.0, z=3.0, xStd=0.1, yStd=0.2, zStd=0.3)
  meas = Measurement.from_measurement_xyz(m)
  assert np.allclose(meas.xyz, np.array([1.0, 2.0, 3.0]))
  assert np.allclose(meas.xyz_std, np.array([0.1, 0.2, 0.3]))


def test_ned():
  m = Simple(n=4.0, e=5.0, d=6.0, nStd=0.4, eStd=0.5, dStd=0.6)
  meas = Measurement.from_measurement_xyz(m)
  assert np.allclose(meas.xyz, np.array([4.0, 5.0, 6.0]))
  assert np.allclose(meas.xyz_std, np.array([0.4, 0.5, 0.6]))


def test_iterable_xyz():
  m = Simple(xyz=[7.0, 8.0, 9.0], xyzStd=[0.7, 0.8, 0.9])
  meas = Measurement.from_measurement_xyz(m)
  assert np.allclose(meas.xyz, np.array([7.0, 8.0, 9.0]))
  assert np.allclose(meas.xyz_std, np.array([0.7, 0.8, 0.9]))


def test_register_custom():
  # simulate runtime-discovered field names and register them
  register_measurement_candidate(("alpha", "beta", "gamma"), ("alphaStd", "betaStd", "gammaStd"))
  m = Simple(alpha=10.0, beta=11.0, gamma=12.0, alphaStd=1.0, betaStd=1.1, gammaStd=1.2)
  meas = Measurement.from_measurement_xyz(m)
  assert np.allclose(meas.xyz, np.array([10.0, 11.0, 12.0]))
  assert np.allclose(meas.xyz_std, np.array([1.0, 1.1, 1.2]))
