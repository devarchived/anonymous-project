import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
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

# --- Global Parameters ---
W = 20
L = 20
cell_size = 0.4

# --- Define AP Positions and Colors ---
ap_coords = {
    "AP1": (5, 5),
    "AP2": (15, 5),
    "AP3": (15, 15),
    "AP4": (5, 15),
}
ap_colors = {
    "AP1": '#90EE90',
    "AP2": 'green',
    "AP3": 'blue',
    "AP4": 'skyblue',
}

# --- CONFIG ---
log_file = os.path.expanduser(
    '~/Desktop/Workspace/ns-allinone-3.43/ns-3.43/scratch/wifi-eht-roaming-scenario/assoc-rxpower-logs/wifi-eht-roaming-assoc-unsaturated-beacons-9-wall-loss-9-seed-21.txt'
)
walls_file = os.path.expanduser(
    '~/Desktop/Workspace/ns-allinone-3.43/ns-3.43/scratch/wifi-ehr-roaming-scenario/walls-assignment.txt'
)

base_name = os.path.basename(log_file)
if "ehr" in log_file:
    out_prefix = "wifi8-roaming"
elif "eht" in log_file:
    out_prefix = "wifi7-roaming"
else:
    out_prefix = "wifi-roaming"

if "wifi-eht-roaming-assoc-" in base_name:
    scenario_part = base_name.split("wifi-eht-roaming-assoc-")[1].replace(".txt", "")
else:
    scenario_part = base_name.replace(".txt", "")

position_to_aps = {}
with open(log_file, 'r') as f:
    lines = f.readlines()

i = 0
while i < len(lines):
    line = lines[i]
    if line.startswith("New position:"):
        pos = tuple(map(float, line.split(":")[1:4]))  # (x, y, z)
        i += 1
        aps = []
        while i < len(lines) and "STA is associated with AP" in lines[i]:
            ap = lines[i].strip().split("with")[-1].strip()
            aps.append(ap)
            i += 1
        position_to_aps[pos[:2]] = aps
    else:
        i += 1

# Create grids and labels
plt.grid(True, which='both', linestyle='--', alpha=0.5)
plt.xlabel('X Coordinate (meters)', fontweight='bold')
plt.ylabel('Y Coordinate (meters)', fontweight='bold')
plt.xlim(-2.5, L+2.5)
plt.ylim(-2.5, W+2.5)
plt.gca().set_aspect('equal', adjustable='box')

# --- Start Plot ---
plt.grid(True, which='both', linestyle='--', alpha=0.5)
plt.xlabel('X Coordinate (meters)', fontweight='bold')
plt.ylabel('Y Coordinate (meters)', fontweight='bold')
plt.xlim(-2.5, L + 2.5)
plt.ylim(-2.5, W + 2.5)
plt.gca().set_aspect('equal', adjustable='box')

# BSS 
ap1 = [(5, 5)]
ap2 = [(15, 5)]
ap3 = [(15, 15)]
ap4 = [(5, 15)]

# Plot APs (diamonds)
plt.scatter(*zip(*ap1), marker='D', s=75, c='#90EE90', edgecolor='black', linewidths=1.5, label='AP 1')
plt.scatter(*zip(*ap2), marker='D', s=75, c='green', edgecolor='black', linewidths=1.5, label='AP 2')
plt.scatter(*zip(*ap3), marker='D', s=75, c='blue', edgecolor='black', linewidths=1.5, label='AP 3')
plt.scatter(*zip(*ap4), marker='D', s=75, c='skyblue', edgecolor='black', linewidths=1.5, label='AP 4')

# Add legend
legend = plt.legend(loc='lower left',
                   bbox_to_anchor=(-0.02, -0.02),
                   ncol=4,
                   handletextpad=0.1,
                   handlelength=1.5,
                   borderaxespad=0.8,
                   labelspacing=0.6,
                   columnspacing=1.0,
                   framealpha=1,
                   edgecolor='black')

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

# --- Plot STA Association as Striped Squares ---
for (x, y), aps in position_to_aps.items():
    x0, y0 = x - cell_size / 2, y - cell_size / 2
    if not aps:
        plt.gca().add_patch(Rectangle((x0, y0), cell_size, cell_size, color='gray'))
    else:
        n = len(aps)
        strip_width = cell_size / n
        for i, ap in enumerate(aps):
            if ap not in ap_colors:
                continue
            left = x0 + i * strip_width
            plt.gca().add_patch(Rectangle((left, y0), strip_width, cell_size, color=ap_colors[ap]))


plt.tight_layout()

plot_filename = f'{out_prefix}/{out_prefix}-{scenario_part}-assocmap.pdf'
plt.savefig(plot_filename, bbox_inches='tight', dpi=1200, format='pdf')