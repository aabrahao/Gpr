import Dubins as db
import matplotlib.pyplot as plt

planner = db.planner(radius = 2.0, point_separation = 0.1)

x, y = db.path(planner, 0, 0, 0, 20, 20, 3.141)
l = db.length(planner, 20, 20, 3.141, 20, 20, 3.141)

print(len(x))
print(l)
plt.plot(x,y)
plt.grid(True)
plt.axis("equal")
plt.show()