import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import os
import numpy as np

from scipy.interpolate import griddata

# Set global style parameters
plt.rcParams.update({
    'font.size': 12,
    'axes.labelsize': 12,
    'axes.titlesize': 12,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'legend.fontsize': 10,
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
ap1 = [(5, 5)]
ap2 = [(15, 5)]
ap3 = [(15, 15)]
ap4 = [(5, 15)]

# Plot APs (diamonds)
plt.scatter(*zip(*ap1), marker='D', s=75, c='#90EE90', edgecolor='black', linewidths=1.5, label='AP 2.4 GHz')
plt.scatter(*zip(*ap2), marker='D', s=75, c='green', edgecolor='black', linewidths=1.5, label='AP 6 GHz')
plt.scatter(*zip(*ap3), marker='D', s=75, c='blue', edgecolor='black', linewidths=1.5, label='AP 5 GHz')
plt.scatter(*zip(*ap4), marker='D', s=75, c='skyblue', edgecolor='black', linewidths=1.5, label='AP 6 GHz')

# Add legend
legend = plt.legend(loc='lower left',
                   bbox_to_anchor=(-0.02, -0.02),
                   ncol=4,
                   handletextpad=0.1,
                   handlelength=1.5,
                   borderaxespad=0.8,
                   labelspacing=0.6,
                   columnspacing=0.1,
                   framealpha=1,
                   edgecolor='black')

# Make legend background visible
legend.get_frame().set_facecolor('white')
legend.get_frame().set_linewidth(1.5)

# --- Read waypoints ---
waypoints_file = os.path.expanduser(
    '~/Desktop/Workspace/ns-allinone-3.43/ns-3.43/scratch/wifi-eht-roaming-scenario/random-human-waypoints.txt'
)
waypoints = []
with open(waypoints_file, 'r') as f:
    for line in f:
        x, y, _ = map(float, line.strip().split())
        waypoints.append((x, y))

# --- Parse log for association and RxPower ---
log_file = os.path.expanduser(
    '~/Desktop/Workspace/ns-allinone-3.43/ns-3.43/scratch/wifi-ehr-roaming-scenario/assoc-rxpower-logs/wifi-ehr-roaming-rxPower-unsaturated-beacons-9-wall-loss-9-seed-21.txt'
)
base_name = os.path.basename(log_file)

if "ehr" in log_file:
    out_prefix = "wifi8-roaming"
elif "eht" in log_file:
    out_prefix = "wifi7-roaming"
else:
    out_prefix = "wifi-roaming"

if "wifi-ehr-roaming-rxPower-" in base_name:
    scenario_part = base_name.split("wifi-ehr-roaming-rxPower-")[1].replace(".txt", "")
else:
    scenario_part = base_name.replace(".txt", "")

heatmap_x, heatmap_y, heatmap_val, heatmap_color = [], [], [], []

with open(log_file, 'r') as f:
    lines = f.readlines()

i = 0
while i < len(lines):
    line = lines[i]
    if line.startswith("New position:"):
        pos = [p.strip() for p in line.split(":")[1:]]
        x, y = float(pos[0]), float(pos[1])
        max_rx = None
        # Read RxPower lines
        while i+1 < len(lines) and lines[i+1].startswith("RxPower"):
            i += 1
            rx_line = lines[i]
            try:
                rx_val = float(rx_line.split(":")[-1].replace("dB", "").strip())
                if max_rx is None or rx_val > max_rx:
                    max_rx = rx_val
            except:
                continue
        if max_rx is not None:
            heatmap_x.append(x)
            heatmap_y.append(y)
            heatmap_val.append(max_rx)
            heatmap_color.append(None)
        else:
            heatmap_x.append(x)
            heatmap_y.append(y)
            heatmap_val.append(-82)
            heatmap_color.append('grey')
    i += 1

# --- Plot heatmap as discrete squares ---
import matplotlib as mpl

cmap = plt.get_cmap('inferno')
norm = mpl.colors.Normalize(vmin=-100, vmax=-30)

for x, y, val, color in zip(heatmap_x, heatmap_y, heatmap_val, heatmap_color):
    if color == 'black':
        plt.scatter(x, y, c='black', s=60, marker='s', edgecolor='none', zorder=10)
    else:
        plt.scatter(x, y, c=[cmap(norm(val))], s=60, marker='s', edgecolor='none', zorder=10)

# Add colorbar for RxPower
sm = mpl.cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])
cbar = plt.colorbar(sm, label='Max RxPower (dB)')
# cbar.ax.invert_yaxis()


# Adjust layout
plt.tight_layout()

# Save as PDF
plot_filename = f'{out_prefix}/{out_prefix}-{scenario_part}-heatmap.pdf'
plt.savefig(plot_filename,
           bbox_inches='tight',
           dpi=1200,
    	   format='pdf')