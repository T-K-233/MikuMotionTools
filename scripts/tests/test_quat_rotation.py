import numpy as np

from mikumotion.math import quat_from_euler_xyz


rad_30 = np.deg2rad(30)
rad_60 = np.deg2rad(60)
rad_90 = np.deg2rad(90)


def test(name, golden, actual):
    if not np.allclose(golden, actual, atol=1e-3):
        print(f"Test <{name}> failed: {golden} != {actual}")
        # raise AssertionError(f"{golden} != {actual}")


test("X=  0,  Y=  0, Z=  0", quat_from_euler_xyz(0, 0, 0), np.array([1, 0, 0, 0]))

test("X= 30,  Y=  0, Z=  0", quat_from_euler_xyz(rad_30, 0, 0), np.array([0.966, 0.259, 0, 0]))
test("X=-30,  Y=  0, Z=  0", quat_from_euler_xyz(-rad_30, 0, 0), np.array([0.966, -0.259, 0, 0]))

test("X=  0,  Y= 60, Z=  0", quat_from_euler_xyz(0, rad_60, 0), np.array([0.866, 0, 0.5, 0]))
test("X=  0,  Y=-60, Z=  0", quat_from_euler_xyz(0, -rad_60, 0), np.array([0.866, 0, -0.5, 0]))

test("X=  0,  Y=  0, Z= 90", quat_from_euler_xyz(0, 0, rad_90), np.array([0.707, 0, 0, 0.707]))
test("X=  0,  Y=  0, Z=-90", quat_from_euler_xyz(0, 0, -rad_90), np.array([0.707, 0, 0, -0.707]))

test("X= 90,  Y= 90, Z=  0", quat_from_euler_xyz(rad_90, rad_90, 0), np.array([.5, .5, .5, .5]))
test("X= 90,  Y=-90, Z=  0", quat_from_euler_xyz(rad_90, -rad_90, 0), np.array([.5, .5, -.5, -.5]))
test("X= 90,  Y=  0, Z= 90", quat_from_euler_xyz(rad_90, 0, rad_90), np.array([.5, .5, -.5, .5]))
test("X= 90,  Y=  0, Z=-90", quat_from_euler_xyz(rad_90, 0, -rad_90), np.array([.5, .5, .5, -.5]))
