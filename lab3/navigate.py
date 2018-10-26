import math
import numpy as np

pt1 = [0.0, 0.0]
pt2 = [-1, 1.0]

distance = math.sqrt( ( pt1[0] - pt2[0] ) ** 2 + ( pt1[1] - pt2[1] ) ** 2) 
angle = 0.0

if( ( pt1[0] - pt2[0] ) == 0):
	angle = math.pi / 2
else:
	angle = math.atan( ( pt1[1] - pt2[1] ) / ( pt1[0] - pt2[0] ) );

print(distance)
print(np.rad2deg(angle))
