import PathPlannerModule as PPM
import math

pathPlanner = PPM.PathPlanner()

origin = (0, 0)
r = 10000

for angle in range(0, 360):
    ang = angle * math.pi / 180
    end = (int(round(math.sin(ang) * r)), int(round(math.cos(ang) * r)))
    print("end = " + str(end))
    pathPlanner.Raycast(origin, end)
    print("")