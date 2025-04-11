import matplotlib.pyplot as plt
import numpy as np

def generate_wifi_scenario(num_bss=6, width=30, height=30, output_file='WiFi_Scenario.pdf'):
    """
    Generate a WiFi network scenario with adjustable number of BSS groups.
    
    Parameters:
    - num_bss: Number of BSS groups (default: 6)
    - width: Width of the area in meters (default: 30)
    - height: Height of the area in meters (default: 30)
    - output_file: Name of the output PDF file
    """
    
    # Set up the figure
    plt.figure(figsize=(12, 10))
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.xlabel('X Coordinate (meters)')
    plt.ylabel('Y Coordinate (meters)')
    plt.title(f'WiFi Network Scenario with {num_bss} BSS Groups')
    plt.xlim(0, width)
    plt.ylim(0, height)

    # Define colors for BSS groups
    colors = plt.cm.tab10.colors  # Using matplotlib's tab10 color palette
    if num_bss > 10:
        colors = plt.cm.tab20.colors  # Switch to tab20 if more than 10 BSS groups

    # Generate positions in a circular pattern
    center_x, center_y = width/2, height/2
    radius = min(width, height) * 0.35
    
    for i in range(num_bss):
        # Calculate AP position (evenly spaced in circle)
        angle = 2 * np.pi * i / num_bss
        ap_x = center_x + radius * np.cos(angle)
        ap_y = center_y + radius * np.sin(angle)
        
        # Calculate STA positions (around the AP)
        sta_positions = []
        for sta_angle in [angle - np.pi/4, angle, angle + np.pi/4]:
            sta_x = ap_x + radius/3 * np.cos(sta_angle)
            sta_y = ap_y + radius/3 * np.sin(sta_angle)
            sta_positions.append((sta_x, sta_y))
        
        # Plot AP (diamond)
        plt.scatter(ap_x, ap_y, marker='D', s=150, 
                   c=colors[i % len(colors)], edgecolor='black', 
                   label=f'BSS {i+1} AP')
        
        # Plot STAs (circles)
        plt.scatter(*zip(*sta_positions), marker='o', s=100,
                   c=colors[i % len(colors)], edgecolor='black',
                   label=f'BSS {i+1} STA')

    # Configure legend based on number of BSS groups
    ncol = 2 if num_bss <= 6 else 3
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',
              handletextpad=0.5, handlelength=1.5,
              borderaxespad=0.5, labelspacing=1.2,
              ncol=ncol, columnspacing=1.5)

    plt.tight_layout()
    plt.savefig(output_file, bbox_inches='tight')
    plt.close()
    print(f"Plot saved as '{output_file}'")

# Example usage:
generate_wifi_scenario(num_bss=6, width=30, height=30, output_file='WiFi8_3GPP_Scenario_6BSS.pdf')
