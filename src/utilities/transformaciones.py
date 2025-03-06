import numpy as np

def xyz_rotation_matrix(thetax, thetay, thetaz, inverse):
    cx, sx = np.cos(thetax), np.sin(thetax)
    cy, sy = np.cos(thetay), np.sin(thetay)
    cz, sz = np.cos(thetaz), np.sin(thetaz)

    if inverse:
        # Rx * Ry * Rz
        M = np.array([
            [cy * cz, cz * sx * sy + cx * sz, -cx * cz * sy + sx * sz],
            [-cy * sz, cx * cz - sx * sy * sz, cz * sx + cx * sy * sz],
            [sy, -cy * sx, cx * cy]
        ])
    else:
        # Rz * Ry * Rx
        M = np.array([
            [cy * cz, -cz * sx + cx * sy * sz, cx * cz * sy + sx * sz],
            [cy * sz, cx * cz + sx * sy * sz, -cz * sx + cx * sy * sz],
            [-sy, cy * sx, cx * cy]
        ])
    return M

def new_coordinates(M, x, y, z, x0, y0, z0):
    point = np.array([x, y, z])
    offset = np.array([x0, y0, z0])
    transformed_point = M @ point + offset
    return transformed_point.tolist()

def foot_coordinate(x, y, z, thetax, thetay):
    cx, sx = np.cos(thetax), np.sin(thetax)
    cy, sy = np.cos(thetay), np.sin(thetay)

    M = np.array([
        [cy, 0, sy],
        [sx * sy, cx, -sx * cy],
        [-cx * sy, sx, cx * cy]
    ])

    point = np.array([x, y, z])
    return (M @ point).tolist()