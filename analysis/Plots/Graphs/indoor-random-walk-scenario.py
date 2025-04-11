import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# Set global style parameters
plt.rcParams.update({
    'font.size': 20,
    'axes.labelsize': 20,
    'axes.titlesize': 25,
    'xtick.labelsize': 20,
    'ytick.labelsize': 20,
    'legend.fontsize': 12,
    'axes.linewidth': 2,
    'grid.linewidth': 1,
    'lines.markersize': 12,
    'lines.markeredgewidth': 1.5
})

# Set up the figure
plt.figure(figsize=(15, 10), dpi=300)

# Create grids and labels
plt.grid(True, which='both', linestyle='--', alpha=0.5)
plt.xlabel('X Coordinate (meters)', fontweight='bold')
plt.ylabel('Y Coordinate (meters)', fontweight='bold')
# plt.title('Indoor Office Scenario with 4 BSS Groups', fontweight='bold', pad=20)

# Set dimensions based on the diagram (D/4 and L)
W = 20  
L = 40  
plt.xlim(-5, L+5)
plt.ylim(-5, W+5)

# Add solid walls (exterior)
building_wall_thickness = 0.2
wall_color = 'gray'
plt.gca().add_patch(Rectangle((0, 0), building_wall_thickness, W, color=wall_color))  # Left wall
plt.gca().add_patch(Rectangle((L-building_wall_thickness, 0), building_wall_thickness, W, color=wall_color))  # Right wall
plt.gca().add_patch(Rectangle((0, 0), L, building_wall_thickness, color=wall_color))  # Bottom wall
plt.gca().add_patch(Rectangle((0, W-building_wall_thickness), L, building_wall_thickness, color=wall_color))  # Top wall

# Add dashed interior walls (D/4 spacing)
room_wall_thickness = 0.1
plt.gca().add_patch(Rectangle((5, 5), room_wall_thickness, 10, color=wall_color))
plt.gca().add_patch(Rectangle((5, 15), 10, room_wall_thickness, color=wall_color))  
plt.gca().add_patch(Rectangle((15, 5), room_wall_thickness, 10, color=wall_color))
plt.gca().add_patch(Rectangle((15, 5), 10, room_wall_thickness, color=wall_color))  
plt.gca().add_patch(Rectangle((25, 5), room_wall_thickness, 10, color=wall_color))
plt.gca().add_patch(Rectangle((25, 15), 10, room_wall_thickness, color=wall_color))
plt.gca().add_patch(Rectangle((35, 5), room_wall_thickness, 10, color=wall_color))
# plt.gca().add_patch(Rectangle((0, 3*W/4), L, room_wall_thickness, color=wall_color))  
# plt.gca().add_patch(Rectangle((L/3, 0), room_wall_thickness, W, color=wall_color))  
# plt.gca().add_patch(Rectangle((2*L/3, 0), room_wall_thickness, W, color=wall_color))  
# plt.gca().add_patch(Rectangle((0, W/4), L, room_wall_thickness, color=wall_color))  
# plt.gca().add_patch(Rectangle((0, 3*W/4), L, room_wall_thickness, color=wall_color))  
# plt.gca().add_patch(Rectangle((L/3, 0), room_wall_thickness, W, color=wall_color))  

# Define positions for APs and STAs

# BSS 
ap = [(10, 10), (30, 10)]

# Plot APs (diamonds)
plt.scatter(*zip(*ap), marker='D', s=300, c='red', edgecolor='black', linewidths=1.5, label='AP')

# Add legend
legend = plt.legend(loc='lower left',
                   bbox_to_anchor=(0.02, 0.02),
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
plt.savefig('indoor-random-walk-scenario.pdf',
           bbox_inches='tight',
           dpi=300)
plt.close()

print("Plot saved as 'indoor-random-walk-scenario.pdf'")