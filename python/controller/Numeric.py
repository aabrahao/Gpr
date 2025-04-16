import numpy as np

def points(x,y):
    return np.column_stack([x, y])

def xy(points):
    if len(points) == 0:
        return np.array([],dtype=float), np.array([],dtype=float)
    return points[:,0],points[:,1]

def distance(x1, y1, x2, y2):
        return np.sqrt( np.power(x1-x2,2) + 
                        np.power(y1-y2,2) )
def flatten(m):
    return m.flatten()

def unflaten(m,shape):
    return m.reshape(shape)

def random(n=1,m=None):
    if m is not None:
        return np.random.random((n,m))
    elif n==1:
        return np.random.random()
    else:
        r = np.random.random(n)
    return r

def irandom(start, end, n):
    r = np.randomom.randomint(start, end, size=n)
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
    
def clip(v, vmin, vmax):
    return np.clip(v, vmin, vmax)

def trim(v, vmin, vmax=None, value=0.0):
    x = v.copy()
    x[x < vmin] = value
    if vmax is not None:
        x[x > vmax] = value
    return x

def normalize(x):
    xmin = np.nanmin(x)
    xmax = np.nanmax(x)
    return (x - xmin) / (xmax - xmin)

def equal(x,v,tol=1e-6):
    return np.isclose(x, v, rtol=tol, atol=tol)

def clean(Z, value=0.0):
    return np.nan_to_num(Z, nan=value)

def nanify(Z, value, tol=1e-6):
    v = Z.copy()
    v[ equal(Z,tol) ] = np.nan
    return v

def mask(Z, lower, upper=None):
    M = 0.0*Z
    if upper is None:
        M[ Z > lower ] = 1.0
    else:
        M[ (Z > lower) & (Z < upper)] = 1.0
    return M

