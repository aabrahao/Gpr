import numpy as np
import robot.utilities.numerics as nm
from robot.utilities.assertion import ensure

r = nm.random(3,5)
r[0,0] = np.nan
r[-1,-1] = np.nan
print(r)
print( nm.clean(r) )

print( nm.normalize(r))

print( nm.vector() )
print( nm.vector(5) )

print( np.zeros([]))

print('-------')
print( nm.matrix() )
print( nm.matrix(3,2) )
print( nm.matrix(2) )
print( nm.matrix(3,2) )



