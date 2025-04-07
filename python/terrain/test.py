import numpy as np
from scipy.interpolate import RegularGridInterpolator
import matplotlib.pyplot as plt

# Original structured grid with different sizes in x and y
x = np.linspace(-10, 10, 50)
y = np.linspace(-10, 10, 100)
x_grid, y_grid = np.meshgrid(x, y)
z_grid = np.sin(np.sqrt(x_grid**2 + y_grid**2))

print(x.shape)
print(y.shape)
print(z_grid.shape)

# Create interpolator
interpolator = RegularGridInterpolator((x, y), z_grid)

# New refined grid with different sizes in x and y
x_new = np.linspace(-10, 10, 200)
y_new = np.linspace(-10, 10, 400)
x_new_grid, y_new_grid = np.meshgrid(x_new, y_new, indexing='ij')

# Interpolate onto the refined grid
z_new_grid = interpolator((x_new_grid, y_new_grid))

# Plot interpolated DEM
plt.figure(figsize=(10, 8))
plt.imshow(z_new_grid.T, extent=(x_new.min(), x_new.max(), y_new.min(), y_new.max()), origin='lower', cmap='terrain', aspect='auto')
plt.colorbar(label='Elevation')
plt.title('Interpolated Digital Elevation Model (DEM) with Different X-Y Sizes')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.show()