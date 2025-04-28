import robot.visualizer.plotter.canvas as cv
import robot.utilities.numerics as nm

x = nm.random(10)
y = nm.random(10)

canvas = cv.Canvas()
canvas.zoom(cv.limits(x,y))
canvas.points(x,y,labels='indices')
canvas.points(x,y)
canvas.lines(x,y)
canvas.show()

