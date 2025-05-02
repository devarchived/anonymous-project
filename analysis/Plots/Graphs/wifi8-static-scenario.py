import matplotlib.pyplot as plt

plt.rcParams.update({
    'font.size': 20,               # Larger base font size
    'axes.labelsize': 16,          # Bigger axis labels
    'axes.titlesize': 25,          # Larger title
    'xtick.labelsize': 20,         # Bigger x-axis ticks
    'ytick.labelsize': 20,         # Bigger y-axis ticks
    'legend.fontsize': 12,         # Larger legend text
    'axes.linewidth': 2,           # Thicker axis lines
    'grid.linewidth': 1,           # Thicker grid lines
    'lines.markersize': 12,       # Larger markers
    'lines.markeredgewidth': 1.5   # Thicker marker edges
})

# Set up the figure
plt.figure(figsize=(10, 10), dpi=300)

# Create grids and labels
plt.grid(True, which='both', linestyle='--', alpha=0.7)
plt.xlabel('X Coordinate (meters)', fontweight='bold')
plt.ylabel('Y Coordinate (meters)', fontweight='bold')
# plt.title('WiFi 8 Joint Transmission Static Scenario')

# Set absolute axis limits (starting from 0)
plt.xlim(-10, 90)
plt.ylim(-10, 90)

# Make ticks more visible
ax = plt.gca()
ax.tick_params(axis='both', which='major', width=2, length=6)

# Define positions for APs and STAs
# BSS 
ap1 = [(30, 50), (10, 50), (30, 70)]
sta1 = (20, 60)

# BSS 2
ap2 = [(50, 50), (50, 70), (70, 50)]
sta2 = (60, 60)

# BSS 3
ap3 = [(30, 30), (10, 30), (30, 10)]
sta3 = (20, 20)

# BSS 4
ap4 = [(50, 30), (50, 10), (70, 30)]
sta4 = (60, 20)

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

# Adjust layout to prevent label cutoff
plt.tight_layout(pad=3.0)

# Save as PDF
plt.savefig('wifi8-static-scenario.pdf', 
               dpi=300, 
               bbox_inches='tight', 
               pad_inches=0.1,
               format='pdf')
plt.close()

print("Plot saved as 'wifi8-static-scenario.pdf'")
