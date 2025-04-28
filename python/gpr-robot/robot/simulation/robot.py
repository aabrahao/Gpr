import numpy as np
import robot.visualizer.plotter.canvas as cnv
import robot.visualizer.plotter.shapes as shp

def feet(inches):
    return inches/12

# Wheels
tire_radius = 0.5*feet( 13.0 )
tire_width  = feet( 4.0 )

# Chassis
wheelbase   = feet( 41.0 )
track_width = feet( 28.0 )

# Maximum steering
turning_radius = feet( 65.0 )
steering_angle = np.deg2rad( 30.0 )

# Antenna coverage
antenna_radious = 1.0 # feet

# Lidar mapping area
area_distance = 4.3 # Relative to the center of the robot
area_with = 30.0
area_height = 18.0

# Robot start postion
robot_x = 0.0
robot_y = -area_distance
robot_heading = np.deg2rad(90)

def width():
    return wheelbase + 2.0*tire_radius
    
def height():
    return track_width + tire_width

###############################################################################
# Graphics

def limits(): 
    w = area_with
    h = area_height
    l = max(area_with,area_height) # Square
    mx = 0.2*l
    my = 0.5*(l-h)
    return (-(0.5*l+mx),(0.5*l+mx),-(my+mx),h+my+mx)

def area(canvas,fill=True):
    a = shp.Rectangle( 0.0, 0.5*area_height, area_with, area_height,
                      color='orange', 
                      thickness = 2,
                      fill = fill,
                      canvas = canvas)

def body(x,y,heading,canvas,color='red', fill=True):
    w1 = shp.Rectangle( 0.5*wheelbase, 0.5*track_width, 2.0*tire_radius, tire_width, color=color, fill=fill)
    w2 = shp.Rectangle( 0.5*wheelbase,-0.5*track_width, 2.0*tire_radius, tire_width, color=color, fill=fill)
    w3 = shp.Rectangle(-0.5*wheelbase, 0.5*track_width, 2.0*tire_radius, tire_width, color=color, fill=fill)
    w4 = shp.Rectangle(-0.5*wheelbase,-0.5*track_width, 2.0*tire_radius, tire_width, color=color, fill=fill)
    a = shp.Circle(0,0,antenna_radious, color=color, fill=fill)
    c = shp.Circle(0,0,feet(2), color=color, fill=fill)
    return shp.Block([w1,w2,w3,w4,a,c],x,y,heading,canvas=canvas)

