import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
csv_file = "robot_path.csv"
try:
    data = pd.read_csv(csv_file)
except FileNotFoundError:
    print(f"Error: File {csv_file} not found.")
    exit(-1)

# Check if the CSV contains the necessary columns
if not {"x", "y", "z"}.issubset(data.columns):
    print("Error: CSV file does not contain 'x', 'y', 'z' columns.")
    exit(-1)

# Plot the 3D path
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(data['x'], data['y'], data['z'], label='Robot Path', marker='o')
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_zlabel('Z (meters)')
ax.set_title('Robot Path Visualization')
ax.legend()

# Show the plot
plt.show()
