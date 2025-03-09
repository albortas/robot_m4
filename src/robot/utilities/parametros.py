"""" Robot dimensions """
Wb = 78  # Shoulder/hip width
Lb = 187.1  # Shoulder to hip length
d = 10.73  # Shoulder articulation height
L0 = 58.09  # Shoulder articulation width
L1 = 108.31  # Leg length
L2 = 138  # Foreleg length

"""Inertia Centers"""
""" Body"""
xCG_Body = 0
yCG_Body = 0
zCG_Body = 0
Weight_Body = 897

# Left Shoulder
xCG_Shoulder = 0
yCG_Shoulder = 5  # menos para el lado derecho
zCG_Shoulder = -9
Weight_Shoulder = 99.3

# Left Leg
xCG_Leg = 0
yCG_Leg = 0
zCG_Leg = -31
Weight_Leg = 133.3

# Left Foreleg
xCG_Foreleg = 0
yCG_Foreleg = 0
zCG_Foreleg = -28
Weight_Foreleg = 107

""" Anchor points """
        
""" Front left shoulder"""
xlf = 93.55
ylf = 39
zlf = 0

"""Front right shoulder"""
xrf = 93.55
yrf = -39
zrf = 0

"""Rear  left hip """
xlr = -93.55
ylr = 39
zlr = 0

"""Rear right hip """
xrr = -93.55
yrr = -39
zrr = 0


seq = [0, 0.5, 0.25, 0.75] # secuencia de movimiento de las patas
track = 0.3  # Separación entre patas
h_amp = 0.1  # Amplitud horizontal del movimiento
v_amp = 0.05  # Amplitud vertical del movimiento
stepl = 0.2  # Longitud del paso