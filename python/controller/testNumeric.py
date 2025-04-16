import numpy as np
import Numeric as nm

print( nm.irand(3,9, 10) )

print(nm.rand(1))
print(nm.rand(10))
print(nm.rand(3,4))

m = nm.rand(3,7)

print(nm.clip(m,0.3, 0.5))

r = nm.rand(3,5)
r[0,0] = np.nan
r[-1,-1] = np.nan
print(r)
print( nm.clean(r) )

print( nm.norm(r))