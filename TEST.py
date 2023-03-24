'''
from start import *

a = np.array([1,2])
p = FindperpenVec(a)
print(p)
'''
import matplotlib.pyplot as plt

# Create a plot
fig, ax = plt.subplots()

# Plot some data
x = [1, 2, 3]
y = [4, 5, 6]
ax.plot(x, y)

# Set the aspect ratio of the plot to "equal"
ax.set_aspect('equal')

# Show the plot
plt.show()
