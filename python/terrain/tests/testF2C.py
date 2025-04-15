import math
import fields2cover as f2c


def plot(cells, headlands, path, swaths, title):
    f2c.Visualizer.figure()
    f2c.Visualizer.plot(cells)
    f2c.Visualizer.plot(headlands)
    f2c.Visualizer.plot(path)
    f2c.Visualizer.plot(swaths)
    f2c.Visualizer.save(title)

rand = f2c.Random(42)

robot = f2c.Robot(2.0, 6.0)

field = rand.generateRandField(1e4, 5)

field = f2c.Field()
cells = field.getField()


headland_const = f2c.HG_Const_gen()
headlands = headland_const.generateHeadlands(cells, 3.0 * robot.getWidth())

bf = f2c.SG_BruteForce()
swaths = bf.generateSwaths(math.pi, robot.getCovWidth(), headlands.getGeometry(0))

snake_sorter = f2c.RP_Snake()
swaths = snake_sorter.genSortedSwaths(swaths)
swaths.at(0).getPath().exportToWkt()


robot.setMinTurningRadius(2)  # m
robot.setMaxDiffCurv(0.1)  # 1/m^2
path_planner = f2c.PP_PathPlanning()

dubins = f2c.PP_DubinsCurves()
path_dubins = path_planner.planPath(robot, swaths, dubins)
plot(cells, headlands, path_dubins, swaths, 'Tutorial_6_1_Dubins.png')

# Dubins curves with Continuous curvature
dubins_cc = f2c.PP_DubinsCurvesCC()
path_dubins_cc = path_planner.planPath(robot, swaths, dubins_cc)
plot(cells, headlands, path_dubins_cc, swaths, 'Tutorial_6_2_Dubins_CC.png')

# Reeds-Shepp curves
reeds_shepp = f2c.PP_ReedsSheppCurves()
path_reeds_shepp = path_planner.planPath(robot, swaths, reeds_shepp)
plot(cells, headlands, path_reeds_shepp, swaths, 'Tutorial_6_3_Reeds_Shepp.png')

# Reeds-Shepp curves with Continuous curvature
reeds_shepp_hc = f2c.PP_ReedsSheppCurvesHC()
path_reeds_shepp_hc = path_planner.planPath(robot, swaths, reeds_shepp_hc)
plot(cells, headlands, path_reeds_shepp_hc, swaths, 'Tutorial_6_4_Reeds_Shepp_HC.png')