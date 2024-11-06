import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

def func(x, y):
    return x*(1-x)*np.cos(4*np.pi*x) * np.sin(4*np.pi*y**2)**2

x, y = np.meshgrid(np.linspace(0, 1, 100), np.linspace(0, 1, 200), indexing='ij')

rng = np.random.default_rng()
points = rng.random((1000, 2))
values = func(points[:,0], points[:,1])

z0 = sp.interpolate.griddata(points, values, (x, y), method='nearest')
z1 = sp.interpolate.griddata(points, values, (x, y), method='linear')
z2 = sp.interpolate.griddata(points, values, (x, y), method='cubic')

plt.subplot(221)
plt.imshow(func(x, y).T, extent=(0, 1, 0, 1), origin='lower')
plt.plot(points[:, 0], points[:, 1], 'k.', ms=1)   # data
plt.title('Original')
plt.subplot(222)
plt.imshow(z0.T, extent=(0, 1, 0, 1), origin='lower')
plt.title('Nearest')
plt.subplot(223)
plt.imshow(z1.T, extent=(0, 1, 0, 1), origin='lower')
plt.title('Linear')
plt.subplot(224)
plt.imshow(z2.T, extent=(0, 1, 0, 1), origin='lower')
plt.title('Cubic')
plt.gcf().set_size_inches(6, 6)
plt.show()