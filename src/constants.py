import numpy as np

MAXV = .2  # Maximum velocity
MAXW = np.deg2rad(45)   # Maximum angular velocity
L = .230    # Distance between wheels
R = .035     # Radius of wheels
KATT = 4.     # Attraction constant
KREP = .0005    # Repulsion constant
KT = .15    # Angular control gain
KR = .07    # Linear control gain
RANGE = 1.4     # Sensor range
MAX_ATTR = .2   # Maximum attraction force
MAX_REP = .25    # Maximum repulsion force