import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.interpolate import griddata

# --- Configuration ---
# Set the random seed for reproducibility.
np.random.seed(4242)

# --- Mars Surface Generation ---
# Create a grid for the Martian surface.
x_surface = np.arange(-10, 10, 0.2)
y_surface = np.arange(-10, 10, 0.2)
x_surface, y_surface = np.meshgrid(x_surface, y_surface)

# Generate a more flat surface with a subtle tilt.
# This creates a gentle slope instead of a large crater.
z_surface = 0.02 * x_surface + 0.03 * y_surface

# Add a few small, well-defined hills.
num_hills = 12
hill_x = np.random.uniform(-8, 8, num_hills)
hill_y = np.random.uniform(-8, 8, num_hills)
hill_height = np.random.uniform(0.2, 0.4, num_hills)
hill_steepness = 0.2

for i in range(num_hills):
    dist_sq = (x_surface - hill_x[i])**2 + (y_surface - hill_y[i])**2
    z_surface += hill_height[i] * np.exp(-dist_sq * hill_steepness)

# --- Rover Trajectory Generation ---
# The total number of steps for the animation.
total_steps = 500
x_traj = []
y_traj = []
z_traj = []

# Starting position and initial heading.
x_current, y_current = np.random.uniform(-8, 8, 2)
heading = np.random.uniform(0, 2 * np.pi)

# Generate a series of "drive straight" and "turn" commands.
for _ in range(10): # Generate 10 distinct path segments
    # Drive straight for a random distance.
    distance = np.random.uniform(2, 5)
    segment_steps = int(distance * 10)

    # Generate points for the straight segment.
    for i in range(segment_steps):
        x_current += np.cos(heading) * 0.1
        y_current += np.sin(heading) * 0.1
        
        # Clamp coordinates to stay within bounds.
        x_current = np.clip(x_current, -9.5, 9.5)
        y_current = np.clip(y_current, -9.5, 9.5)

        x_traj.append(x_current)
        y_traj.append(y_current)

    # Turn for the next segment.
    heading += np.random.uniform(-np.pi / 4, np.pi / 4) # Random turn angle

# Convert lists to NumPy arrays for easier processing.
x_traj = np.array(x_traj)
y_traj = np.array(y_traj)

# Use interpolation to find the Z coordinate for each point on the trajectory.
points = np.vstack([x_surface.flatten(), y_surface.flatten()]).T
values = z_surface.flatten()
z_traj = griddata(points, values, (x_traj, y_traj), method='cubic')

# --- 3D Plot and Animation ---
# Set up the figure and the 3D axes.
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')
ax.set_title('Rover Trajectory Visualization', fontsize=20)
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Altitude (m)')

# Plot the Mars-like surface.
surface_plot = ax.plot_surface(x_surface, y_surface, z_surface, cmap='Oranges', alpha=0.9, rstride=1, cstride=1)

# Plot the entire trajectory line at the start.
line, = ax.plot(x_traj, y_traj, z_traj, 'b-', lw=2, label='Rover Path')

# Initialize the rover marker. We will animate this marker along the path.
rover, = ax.plot([], [], [], 'bo', markersize=8, label='Rover')

# Set the axis limits.
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([np.min(z_surface) - 1, np.max(z_surface) + 1])

# Add a legend to the plot.
ax.legend()

# The `update` function for the animation.
def update(frame):
    """
    Updates the rover marker for each frame of the animation.
    """
    if frame >= len(x_traj):
        return rover,
    
    # Update the rover marker's position.
    rover.set_data(x_traj[frame], y_traj[frame])
    rover.set_3d_properties([z_traj[frame]])
    
    return rover,

# Create the animation.
# The `update` function is called for each frame, moving the rover.
# `repeat=True` makes the animation loop indefinitely.
ani = FuncAnimation(fig, update, frames=len(x_traj), interval=50, blit=True, repeat=True)

# Display the plot.
plt.show()
