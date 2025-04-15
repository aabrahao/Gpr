import numpy as np

m = np.random.rand(5,3)
print(m)

print(m[0:3,:])
print(m[0:3,:])

print(m[:,-1:])

print(np.array([]))

x = np.array([1,2,3])

print(np.insert(x,0,10))
print(np.append(x,10))


print( np.insert(np.append(x, 111), 0, 111111) )


print(np.random.uniform(0.0, 1.0, 5))
print(np.random.uniform(0.0, 1.0, 5))
print(np.random.uniform(0.0, 1.0, 1))
print(np.random.uniform(0.0, 1.0, 1))

print("Shape #############")

u = np.random.random(5)
v = np.random.random((3,7))
print(u.shape)
print(v.shape)


print(np.isnan(u).any())