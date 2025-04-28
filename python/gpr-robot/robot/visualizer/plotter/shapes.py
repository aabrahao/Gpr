import numpy as np
import matplotlib.patches as patches
import matplotlib.collections as collections
import matplotlib.transforms as transforms

# Angles in radians!
# Rectangles origins at centers

class Shape:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.a = angle
        self.patch = None
    # Members
    def position(self):
        return self.x, self.y
    def angle(self):
        return self.a
    # Methods
    def move(self,x,y,angle=None):
        self.x = x
        self.y = y
        if angle is not None:
            self.a = angle
        self.update()
    def rotate(self, angle):
        self.a = angle
        self.update()
    def show(self):
        if self.patch:
            self.patch.set_visible(True)
    def hide(self):
        if self.patch:
            self.patch.set_visible(False)
    def inside(self, x, y):
        return self._inside(x,y)
    def update(self):
        if self.patch:
            self._update_patch()
    # Privates
    def _attach(self, canvas):
        if canvas is None:
            return
        if self.patch is None:
            self.patch = self._make_patch()
            self._add_patch(canvas)
        else:
            print('Ops, alreay attached!')
    # Virtuals !
    def _inside(self, x, y):
        return False
    def _make_patch(self):
        pass
    def _update_patch(self):
        pass
    def _add_patch(self, canvas): # Patch or collections ?
        canvas.axes.add_patch(self.patch)   

class Colored(Shape):
    def __init__(self, x, y, angle, color, fill, thickness, alpha):
        super().__init__(x, y, angle)
        self.color = color
        self.fill = fill
        self.thickness = thickness
        self.alpha = alpha
    
class Circle(Colored):
    def __init__(self, x=0, y=0, radius=1, angle=0,
                 color=None, fill=True, thickness=None, alpha=0.5, canvas=None):
        super().__init__(x, y, angle, color, fill, thickness, alpha)
        self.radius = radius
        self._attach(canvas)
    def _make_patch(self):
        return patches.Circle((self.x,self.y),self.radius,
                              edgecolor=self.color,
                              facecolor=self.color,
                              fill=self.fill,
                              linewidth=self.thickness,
                              alpha=self.alpha)
    def _update_patch(self):
        self.patch.center = (self.x, self.y)
    def _inside(self, x, y):
        distance = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        return distance <= self.radius

class Rectangle(Colored):
    def __init__(self, x=0, y=0, width=2, height=1, angle=0, 
                 color=None, fill=True, thickness=None, alpha=0.5, canvas=None):
        super().__init__(x, y, angle, color, fill, thickness,alpha)
        self.width = width
        self.height = height
        self._attach(canvas)
    def _make_patch(self):
        x = self.x - 0.5*self.width
        y = self.y - 0.5*self.height
        return patches.Rectangle((x,y),self.width,self.height,angle=self.a,
                                 edgecolor=self.color,
                                 facecolor=self.color,
                                 fill=self.fill,
                                 linewidth=self.thickness,
                                 alpha=self.alpha,rotation_point='center')
    def _update_patch(self):
        x = self.x - 0.5*self.width
        y = self.y - 0.5*self.height
        self.patch.set_xy((x,y))
        self.patch.angle = self.a
    def _inside(self, x, y):
        a = self.a
        xc = x - self.x
        yc = y - self.y
        xr = xc * np.cos(-a) - yc * np.sin(-a)
        yr = xc * np.sin(-a) + yc * np.cos(-a)
        return (abs(xr) <= 0.5*self.width) and (abs(yr) <= 0.5*self.height)

class Block(Shape):
    def __init__(self, shapes, x=0, y=0, angle=0, canvas=None):
        super().__init__(x, y, angle)
        self.shapes = shapes
        self._attach(canvas)
    def _add_patch(self, canvas): # Patch or collections ?
        canvas.axes.add_collection(self.patch) 
        self.update()
    def _make_patch(self):
        patches=[]
        for shape in self.shapes:
            patch = shape._make_patch() # Copy!!!!
            patches.append(patch)
        collection = collections.PatchCollection(patches,match_original=True)
        return collection
    def _update_patch(self):
        transform = transforms.Affine2D()
        transform.rotate(self.a)
        transform.translate(self.x,self.y)
        self.patch.set_transform(transform + self.patch.axes.transData)
    def _inside(self, x, y):
        for shape in self.shapes:
            okay = shape.inside(x,y)
            if okay:
                return True
        return False