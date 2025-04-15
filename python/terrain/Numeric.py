import numpy as np

def points(x,y):
    return np.column_stack([x, y])

def xy(points):
    return points[:,0],points[:,1]

def distance(x1, y1, x2, y2):
        return np.sqrt( np.power(x1-x2,2) + 
                        np.power(y1-y2,2) )
    
def flatten(m):
    return m.flatten()

def unflaten(m,shape):
    return m.reshape(shape)

def rand(n=1):
    r = np.random.uniform(0.0, 1.0, n)
    if n==1:
        r = r[0]
    return r

def irand(start, end, n):
    r = np.random.randint(start, end, size=n)
    if n==1:
        r = r[0]
    return r

def dim(x):
    shape = x.shape
    if len(shape) == 1:
        return 1
    elif len(shape) == 2:
        if shape[0] == 1 or shape[1] == 1:
            return 1
        else:
            return 2
    else:
        return f"Ops, can't work {len(shape)}D Array!"
    
def constrain(v, vmin, vmax):
    x = v.copy()
    x[x < vmin] = vmin
    x[x > vmax] = vmax
    return x

def trim(v, vmin, vmax, value=np.nan):
    x = v.copy()
    x[x < vmin] = value
    if vmax is not None:
        x[x > vmax] = value
    return x

def norm(v):
    x = v.copy()
    xmin = np.nanmin(x)
    xmax = np.nanmax(x)
    x = (x - xmin) / (xmax - xmin)
    return x

def nan(v, value):
    x = v.copy()
    x[ np.isnan(v) ] = value
    return x