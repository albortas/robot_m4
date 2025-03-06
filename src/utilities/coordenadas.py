class Coordinates:
    # Axis coordinates for animation
    X_AXIS = {"x": [0, 100], "y": [0, 0], "z": [0, 0]}
    Y_AXIS = {"x": [0, 0], "y": [0, 100], "z": [0, 0]}
    Z_AXIS = {"x": [0, 0], "y": [0, 0], "z": [0, 100]}

    # Floor coordinates for animation
    FLOOR = {
        "x": [500, 500, -500, -500],
        "y": [500, -500, -500, 500],
        "z": [0, 0, 0, 0],
    }