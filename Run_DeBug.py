from start import *


p0 = np.array([0,6429000])
v0 = np.array([-9000,0])

T = 16000
Px,Py,Vel,tim = Simu(p0,v0,T)


# Set up the figure
fig, ax = plt.subplots()

# Draw a circle with center (0.5, 0.5) and radius 0.2
circle = plt.Circle((0, 0), 6371000)

# Add the circle to the plot
ax.add_artist(circle)

# Set the x and y limits of the plot

# Show the plot

plt.plot(Px,Py)

ax.set_aspect('equal')
plt.show()


