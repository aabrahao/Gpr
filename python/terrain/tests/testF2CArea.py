import math
import fields2cover as f2c
import matplotlib.pyplot as plt

def plot(cells, headlands, path, swaths, title):
    f2c.Visualizer.figure()
    f2c.Visualizer.plot(cells)
    f2c.Visualizer.plot(headlands)
    f2c.Visualizer.plot(path)
    f2c.Visualizer.plot(swaths)
    f2c.Visualizer.save(title)

def toVectorPoint(points):
    vector = []
    for point in points:
        vector.append( (point[0], point[1]) )
    return f2c.VectorPoint(vector)

def toRing(points):
    ring = f2c.LinearRing()
    for point in points:
        ring.addPoint(point[0],point[1])
    first = points[0]
    ring.addPoint(first[0], first[1])
    return ring

robot = f2c.Robot(2.0, 6.0)

ring1 = toRing([(0,0), (60,-60), (60,60), (0,100)])
ring2 = toRing([(12,12), (12,18),(18,18),(18,12)])
ring3 = toRing([(36,36), (36,48),(48,48),(48,36)])
ring4 = toRing([(0+100,0+100), (60+100,-60+100), (60+100,60+100), (0+100,100+100)])

cell1 = f2c.Cell( ring1 )
cell2 = f2c.Cell( ring4 )

cells = f2c.Cells()
cells.addGeometry(cell1)
cells.addRing(0,ring2)
cells.addRing(0,ring3)

cells.addGeometry(cell2)


# Headland
headland_const = f2c.HG_Const_gen()
headlands = headland_const.generateHeadlands(cells, 3.0 * robot.getWidth())

# Swaths
bf = f2c.SG_BruteForce()
swaths = bf.generateSwaths(math.pi, robot.getCovWidth(), headlands.getGeometry(0))
#swaths = bf.generateBestSwaths(bf, robot.getCovWidth(), headlands);

pattern = f2c.RP_Boustrophedon();
swaths = pattern.genSortedSwaths(swaths);

# Path Planner
robot.setMinTurningRadius(2)  # m
robot.setMaxDiffCurv(0.1)  # 1/m^2
path_planner = f2c.PP_PathPlanning()

# Dubins curves
dubins = f2c.PP_DubinsCurves()
path_dubins = path_planner.planPath(robot, swaths, dubins)
plot(cells, headlands, path_dubins, swaths, 'gpr-robot.png')
