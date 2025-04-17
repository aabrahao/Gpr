import numpy as np
import Numeric as nm

r = nm.random(3,5)
r[0,0] = np.nan
r[-1,-1] = np.nan
print(r)
print( nm.clean(r) )

print( nm.normalize(r))

print( nm.vector() )
print( nm.vector(5) )

print( np.zeros([]))