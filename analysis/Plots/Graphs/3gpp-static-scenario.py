import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

plt.rcParams.update({
    'font.size': 20,               # Larger base font size
    'axes.labelsize': 20,          # Bigger axis labels
    'axes.titlesize': 25,          # Larger title
    'xtick.labelsize': 20,         # Bigger x-axis ticks
    'ytick.labelsize': 20,         # Bigger y-axis ticks
    'legend.fontsize': 12,         # Larger legend text
    'axes.linewidth': 2,           # Thicker axis lines
    'grid.linewidth': 1,           # Thicker grid lines
    'lines.markersize': 12,       # Larger markers
    'lines.markeredgewidth': 1.5   # Thicker marker edges
})

# Set up the figure with larger size
plt.figure(figsize=(15, 10), dpi=300)

# Create grids and labels with enhanced visibility
plt.grid(True, which='both', linestyle='--', alpha=0.7)
plt.xlabel('X Coordinate (meters)', fontweight='bold')
plt.ylabel('Y Coordinate (meters)', fontweight='bold')
# plt.title('3GPP WiFi Network Scenario with 6 BSS Groups and Walls', fontsize=14, fontweight='bold')

# Set dimensions based on the diagram (W and D/2)
W = 50  
L = 120  
plt.xlim(-10, L+10)
plt.ylim(-10, W+10)

# Add rectangular walls based on D/2 dimensions
wall_thickness = 0.2  # Visual thickness of walls
wall_color = 'gray'

# Vertical walls
plt.gca().add_patch(Rectangle((0, 0), wall_thickness, W, color=wall_color))  # Left wall
plt.gca().add_patch(Rectangle((L-wall_thickness, 0), wall_thickness, W, color=wall_color))  # Right wall

# Horizontal walls
plt.gca().add_patch(Rectangle((0, 0), L, wall_thickness, color=wall_color))  # Bottom wall
plt.gca().add_patch(Rectangle((0, W-wall_thickness), L, wall_thickness, color=wall_color))  # Top wall

# Define positions for APs and STAs
# BSS 
ap1 = [(10, 35), (10, 15), (30, 15)]
sta1 = (20, 25)

# BSS 2
ap2 = [(30, 35), (50, 35), (50, 15)]
sta2 = (40, 25)

# BSS 3
ap3 = [(70, 35), (70, 15), (90, 35)]
sta3 = (80, 25)

# BSS 4
ap4 = [(90, 15), (110, 15), (110, 35)]
sta4 = (100, 25)

# Plot APs (diamonds)
plt.scatter(*zip(*ap1), marker='D', s=300, c='red', edgecolor='black', label='BSS 1 AP')
plt.scatter(*zip(*ap2), marker='D', s=300, c='blue', edgecolor='black', label='BSS 2 AP')
plt.scatter(*zip(*ap3), marker='D', s=300, c='green', edgecolor='black', label='BSS 3 AP')
plt.scatter(*zip(*ap4), marker='D', s=300, c='purple', edgecolor='black', label='BSS 4 AP')

# Plot STAs (circles)
plt.scatter(*sta1, marker='o', s=400, c='red', edgecolor='black', label='BSS 1 STA')
plt.scatter(*sta2, marker='o', s=400, c='blue', edgecolor='black', label='BSS 2 STA')
plt.scatter(*sta3, marker='o', s=400, c='green', edgecolor='black', label='BSS 3 STA')
plt.scatter(*sta4, marker='o', s=400, c='purple', edgecolor='black', label='BSS 4 STA')

# Add legend with increased vertical spacing
legend = plt.legend(loc='lower left',
                   bbox_to_anchor=(0.02, 0.02),
                   ncol=4,
                   handletextpad=1.0,
                   handlelength=2.0,
                   borderaxespad=0.8,
                   labelspacing=1.2, 
                   framealpha=1, 
                   edgecolor='black',
                   markerscale=0.6)

# Make legend background more visible
legend.get_frame().set_facecolor('white')
legend.get_frame().set_linewidth(1.5)

# Adjust layout to prevent cutoff
plt.tight_layout()

# Save as high-quality PDF
plt.savefig('wifi8-3gpp-scenario.pdf', 
           bbox_inches='tight',
           dpi=300)
plt.close()

print("Plot with walls saved as 'wifi8-3gpp-scenario.pdf'")