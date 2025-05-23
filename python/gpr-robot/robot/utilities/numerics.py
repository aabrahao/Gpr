import numpy as np
from robot.utilities.assertion import ensure
from robot.utilities.assertion import error

def scalar(arg): # Is a scalar?
    if isinstance(arg, np.ndarray):
        return arg.size == 1  # Single element arrays are scalars
    elif isinstance(arg, (list, tuple)):
        return len(arg) == 1  # Single element lists/tuples
    else:
        # Numbers and other atomic types
        return np.isscalar(arg)

def null():
    return np.array([],dtype=float)

def vector(n = None):
    if n is None:
        return null()
    return np.zeros(n)

def matrix(n = None,m = None):
    if (n is not None) and (m is not None):
        return np.zeros((n,m))
    elif (n is None) and (m is None):
        return null()
    else:
        error(f'Matrix ({n},{m}) need two arguments or none of them!')

def points(x,y):
    return np.column_stack([x, y])

def xy(points):
    if len(points) == 0:
        return vector(), vector()
    return points[:,0],points[:,1]

def distance(x1, y1, x2, y2):
        return np.sqrt( (x1-x2)**2 +(y1-y2)**2 )

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

def length(x,y):
    dx = np.diff(x)
    dy = np.diff(y)
    return np.sum( np.sqrt(dx**2 + dy**2) )

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
    M = np.zeros(Z.shape)
    if upper is None:
        M[ Z > lower ] = 1.0
    else:
        M[ (Z > lower) & (Z < upper)] = 1.0
    return M

