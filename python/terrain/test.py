import pyvista
from pyvista import examples
import time

# load and shrink airplane
airplane = pyvista.PolyData(examples.planefile)
airplane.points /= 10 # shrink by 10x

# rotate and translate ant so it is on the plane
ant = pyvista.PolyData(examples.antfile)
ant.rotate_x(90, inplace=True)
ant.translate([90, 60, 15], inplace=True)

# Make a copy and add another ant
ant_copy = ant.copy()
ant_copy.translate([30, 0, -10], inplace=True)

# Create plotter object
plotter = pyvista.Plotter()
plotter.add_mesh(ant, 'r')
plotter.add_mesh(ant_copy, 'b')

# Add airplane mesh and make the color equal to the Y position. Add a
# scalar bar associated with this mesh
plane_scalars = airplane.points[:, 1]
plotter.add_mesh(airplane, scalars=plane_scalars,
                 scalar_bar_args={'title': 'Airplane Y\nLocation'})

# Add annotation text
plotter.add_text('Ants and Plane Example')
plotter.show()

# Initialize plotter
plotter.show(auto_close=False, interactive_update=True)

# Create animation by updating the point position
t = 0
dt = 0.1
while True:
    x = np.sin(t)
    y = np.cos(t)
    ant.translate([x, y, 0], inplace=True)

    plotter.update()
    time.sleep(0.03)
    t = t + dt

# Close the plotter when animation is done
plotter.close()