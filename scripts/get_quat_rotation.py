import numpy as np

from mikumotion.math import quat_from_euler_xyz, quat_mul


np.set_printoptions(precision=3, suppress=True)

# rot_a = quat_from_euler_xyz(np.deg2rad(90), 0, np.deg2rad(-90))
# rot_b = quat_from_euler_xyz(0, np.deg2rad(90+55), 0)
# rot_c = quat_mul(rot_a, rot_b)

# rot_a = quat_from_euler_xyz(np.deg2rad(90), 0, np.deg2rad(-90))
# rot_b = quat_from_euler_xyz(0, np.deg2rad(65), 0)
# rot_c = quat_mul(rot_a, rot_b)

# rot_a = quat_from_euler_xyz(np.deg2rad(90), 0, np.deg2rad(-90))
# rot_b = quat_from_euler_xyz(0, np.deg2rad(90), 0)
# rot_c = quat_mul(rot_a, rot_b)

print(quat_mul(
    np.array([-0.327, 0.627, 0.627, 0.327]),
    quat_from_euler_xyz(0, np.deg2rad(200), 0),
))
