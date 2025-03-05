from math import pi,sin,asin,cos,atan,acos,sqrt
from src.utilities.parametros import *

def IK(x, y, z, side):  # Inverse Kinematics
    """
    s = 1 for left leg
    s = -1 for right leg
    """
    t2 = y ** 2
    t3 = z ** 2
    t4 = t2 + t3
    t5 = 1 / sqrt(t4)
    t6 = L0 ** 2
    t7 = t2 + t3 - t6
    t8 = sqrt(t7)
    t9 = d - t8
    t10 = x ** 2
    t11 = t9 ** 2
    t15 = L1 ** 2
    t16 = L2 ** 2
    t12 = t10 + t11 - t15 - t16
    t13 = t10 + t11
    t14 = 1 / sqrt(t13)
    error = False
    
    try:
        theta1 = side * (-pi / 2 + asin(t5 * t8)) + asin(t5 * y)
        theta2 = -asin(t14 * x) + asin(L2 * t14 * sqrt(1 / t15 * 1 / t16 * t12 ** 2 * (-1 / 4) + 1))
        theta3 = -pi + acos(-t12 / 2 / (L1 * L2))

    except ValueError:
        print('ValueError IK')
        error = True
        theta1 = 90
        theta2 = 90
        theta3 = 90



    theta = [theta1, theta2, theta3]
    return theta, error

def CI(x, y, z, side):
    C = sqrt(y**2 + z**2)
    D = sqrt(C**2 - L0**2) - d
    G = sqrt(x**2 + D**2)
    error = False
    
    try:
        theta1 = side * (-pi / 2 + asin((D + d)/C)) + asin(y/C)
        #theta1 = side *( -pi/2 + asin(z/C) + asin(L0/C))
        theta2 = acos((L1**2 + L2**2 - G**2)/(2 * L1 * L2))
        theta3 = atan(x/D) + ((L2 * sin(theta2))/(G))
        
    except ValueError:
        print('ValueError IK')
        error = True
        theta1 = 90
        theta2 = 90
        theta3 = 90
    
    theta = [theta1, theta2, theta3]
    return theta,error

def FK(theta, side):  # Forward Kinematics
    """ Calculation of articulation points """
    """
    s = 1 for left leg
    s = -1 for right leg
    """
    x_shoulder1 = 0
    y_shoulder1 = d * sin(theta[0])
    z_shoulder1 = -d * cos(theta[0])

    x_shoulder2 = 0
    y_shoulder2 = side * L0 * cos(theta[0]) + d * sin(theta[0])
    z_shoulder2 = side * L0 * sin(theta[0]) - d * cos(theta[0])

    x_elbow = -L1 * sin(theta[1])
    y_elbow = side * L0 * cos(theta[0]) - (-d - L1 * cos(theta[1])) * sin(theta[0])
    z_elbow = side * L0 * sin(theta[0]) + (-d - L1 * cos(theta[1])) * cos(theta[0])

    return [x_shoulder1, x_shoulder2, x_elbow, y_shoulder1, y_shoulder2, y_elbow, z_shoulder1, z_shoulder2, z_elbow]

def FK_Weight(theta, side):  # Cinemática directa para el cálculo del centro de gravedad
    """ Calculation of articulation points """
    """
    side = 1 for left leg
    side = -1 for right leg
    """

    xCG_Shoulder1 = xCG_Shoulder
    yCG_Shoulder1 = side * yCG_Shoulder * cos(theta[0]) - zCG_Shoulder * sin(theta[0])
    zCG_Shoulder1 = side * yCG_Shoulder * sin(theta[0]) + zCG_Shoulder * cos(theta[0])

    xCG_Leg1 = xCG_Leg * cos(theta[1]) + zCG_Leg * sin(theta[1])
    yCG_Leg1 = cos(theta[0]) * (L0 * side + side * yCG_Leg) + sin(theta[0]) * (
                d - zCG_Leg * cos(theta[1]) + xCG_Leg * sin(theta[1]))
    zCG_Leg1 = sin(theta[0]) * (L0 * side + side * yCG_Leg) - cos(theta[0]) * (
                d - zCG_Leg * cos(theta[1]) + xCG_Leg * sin(theta[1]))

    xCG_Foreleg1 = cos(theta[1]) * (xCG_Foreleg * cos(theta[2]) + zCG_Foreleg * sin(theta[2])) - sin(
        theta[1]) * (L1 - zCG_Foreleg * cos(theta[2]) + xCG_Foreleg * sin(theta[2]))
    yCG_Foreleg1 = cos(theta[0]) * (L0 * side + side * yCG_Foreleg) + sin(theta[0]) * (
                d + sin(theta[1]) * (
                    xCG_Foreleg * cos(theta[2]) + zCG_Foreleg * sin(theta[2])) + cos(theta[1]) * (
                            L1 - zCG_Foreleg * cos(theta[2]) + xCG_Foreleg * sin(theta[2])))
    zCG_Foreleg1 = sin(theta[0]) * (L0 * side + side * yCG_Foreleg) - cos(theta[0]) * (
                d + sin(theta[1]) * (
                    xCG_Foreleg * cos(theta[2]) + zCG_Foreleg * sin(theta[2])) + cos(theta[1]) * (
                            L1 - zCG_Foreleg * cos(theta[2]) + xCG_Foreleg * sin(theta[2])))

    return [xCG_Shoulder1, xCG_Leg1, xCG_Foreleg1, yCG_Shoulder1, yCG_Leg1, yCG_Foreleg1, zCG_Shoulder1, zCG_Leg1,
            zCG_Foreleg1]