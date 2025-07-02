import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import os

# Set global style parameters
plt.rcParams.update({
    'font.size': 12,
    'axes.labelsize': 12,
    'axes.titlesize': 12,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'legend.fontsize': 12,
    'axes.linewidth': 2,
    'grid.linewidth': 1,
    'lines.markersize': 9,
    'lines.markeredgewidth': 1.5
})

# Set up the figure
# plt.figure(figsize=(15, 15), dpi=300)
# plt.figure(dpi=300)

# Create grids and labels
plt.grid(True, which='both', linestyle='--', alpha=0.5)
plt.xlabel('X Coordinate (meters)', fontweight='bold')
plt.ylabel('Y Coordinate (meters)', fontweight='bold')
# plt.title('Indoor Office Scenario with 4 BSS Groups', fontweight='bold', pad=20)

# Set dimensions based on the diagram (D/4 and L)
W = 20  
L = 20  
plt.xlim(-2.5, L+2.5)
plt.ylim(-2.5, W+2.5)
plt.gca().set_aspect('equal', adjustable='box')

# Add solid walls (exterior)
building_wall_thickness = 0.1
wall_color = 'gray'
plt.gca().add_patch(Rectangle((0, 0), building_wall_thickness, W, color=wall_color))  # Left wall
plt.gca().add_patch(Rectangle((L-building_wall_thickness, 0), building_wall_thickness, W, color=wall_color))  # Right wall
plt.gca().add_patch(Rectangle((0, 0), L, building_wall_thickness, color=wall_color))  # Bottom wall
plt.gca().add_patch(Rectangle((0, W-building_wall_thickness), L, building_wall_thickness, color=wall_color))  # Top wall

# Add dashed interior walls (D/4 spacing)
room_wall_thickness = 0.05
walls_file = os.path.expanduser(
    '~/Desktop/Workspace/ns-allinone-3.43/ns-3.43/scratch/wifi-ehr-roaming-scenario/walls-assignment.txt'
)
with open(walls_file, 'r') as f:
    for line in f:
        if line.strip() == "" or line.startswith("roomX"):
            continue
        parts = line.split()
        xMin, xMax = float(parts[2]), float(parts[3])
        yMin, yMax = float(parts[4]), float(parts[5])
        # Draw vertical wall
        if abs(xMin - xMax) < 1e-4:
            x = xMin - room_wall_thickness / 2
            y = min(yMin, yMax)
            height = abs(yMax - yMin)
            plt.gca().add_patch(Rectangle((x, y), room_wall_thickness, height, color=wall_color, alpha=0.7))
        # Draw horizontal wall
        elif abs(yMin - yMax) < 1e-4:
            x = min(xMin, xMax)
            y = yMin - room_wall_thickness / 2
            width = abs(xMax - xMin)
            plt.gca().add_patch(Rectangle((x, y), width, room_wall_thickness, color=wall_color, alpha=0.7))

# Define positions for APs and STAs

# BSS 
ap = [(5, 5), (5, 15), (15,5), (15,15)]

# Plot APs (diamonds)
plt.scatter(*zip(*ap), marker='D', s=75, c='red', edgecolor='black', linewidths=1.5, label='AP')

# Add legend
legend = plt.legend(loc='lower left',
                   bbox_to_anchor=(-0.02, -0.02),
                   handletextpad=1.0,
                   handlelength=2.0,
                   borderaxespad=0.8,
                   labelspacing=1.2,
                   framealpha=1,
                   edgecolor='black')

# Make legend background visible
legend.get_frame().set_facecolor('white')
legend.get_frame().set_linewidth(1.5)

# Adjust layout
plt.tight_layout()

# Save as PDF
# plt.show()
plt.savefig('indoor-sequential-walk-scenario.pdf',
           bbox_inches='tight',
           dpi=300,
    	   format='pdf')
plt.close()

print("Plot saved as 'indoor-sequential-walk-scenario.pdf'")
