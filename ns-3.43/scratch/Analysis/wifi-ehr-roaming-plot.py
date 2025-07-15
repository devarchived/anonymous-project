import os
import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import scipy
import pandas as pd
import math

# Set global style parameters
plt.rcParams.update({
    'font.size': 20,
    'axes.labelsize': 20,
    'axes.titlesize': 20,
    'xtick.labelsize': 20,
    'ytick.labelsize': 20,
    'legend.fontsize': 15,
    'axes.linewidth': 2,
    'grid.linewidth': 1,
    'lines.markersize': 9,
    'lines.markeredgewidth': 1.5
})

def filter_extreme_values(metric_values, lower_percentile=5, upper_percentile=90):
    if len(metric_values) == 0:
        return []
    lower_bound = np.percentile(metric_values, lower_percentile)
    upper_bound = np.percentile(metric_values, upper_percentile)

    return [value for value in metric_values if lower_bound <= value <= upper_bound]

def main():
    # %% Plotting for Wifi 8
    
    # Read the data file
    input_filename = '../wifi-ehr-roaming-scenario/result-logs/sorted-wifi-ehr-roaming-results-unsaturated-9-dB-reliability.txt'
    
    # Generate input filename
    base_name = os.path.basename(input_filename)
    out_prefix = 'wifi8' if 'ehr' in base_name else 'wifi7' if 'eht' in base_name else 'wifi'
    reliability_str = '-reliability' if 'reliability' in base_name else ''
    # Extract the dB part (e.g., '9-dB') from the filename
    db_part = ''
    for i, part in enumerate(base_name.split('-')):
        if 'dB' in part:
            # Get the previous part and this part, e.g., '9-dB'
            db_part = f"{base_name.split('-')[i-1]}-dB"
            break
    if not db_part:
        # fallback: try to get the last part before .txt
        db_part = base_name.split('.txt')[0].split('-')[-2] + '-dB'

    wifi_ehr = pd.read_csv(input_filename, header=None, 
                    names=['seedNumber', 'maxMissedBeacons', 'Throughput', 'PacketDropReliability', 
                            'PacketReceivedReliability', 'EndToEndDelay', 'ChannelAccessDelay','AssociationDelay','AssociationCount'])

    # Clean the data (remove units from delay columns)
    wifi_ehr['EndToEndDelay'] = wifi_ehr['EndToEndDelay'].str.replace('ms', '').str.replace('+', '').astype(float)
    wifi_ehr['ChannelAccessDelay'] = wifi_ehr['ChannelAccessDelay'].str.replace('ms', '').str.replace('+', '').astype(float)
    wifi_ehr['AssociationDelay'] = (wifi_ehr['AssociationDelay'].str.replace('ms', '').str.replace('+', '').astype(float) / 1000
)

    # Set style for all plots
    plt.rcParams['figure.figsize'] = (10, 6)
    plt.rcParams['grid.color'] = '0.8'
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.linewidth'] = 0.5

    # Plot throughput (Simulation vs Analysis)
    th_data = [wifi_ehr[wifi_ehr['maxMissedBeacons'] == val]['Throughput'].values for val in sorted(wifi_ehr['maxMissedBeacons'].unique())]
    positions = sorted(wifi_ehr['maxMissedBeacons'].unique())
    
    plt.figure()
    th_box = plt.violinplot(th_data, 
                      positions=positions,
                      vert=True,
                      widths=0.7,
                      showmeans=False,
                      showmedians=True,
                      showextrema=True)
    th_box_label = mpatches.Patch(facecolor='skyblue', edgecolor='navy', 
                          label='WiFi 8 JT (ns-3 Simulation)')
    
    
    plt.xlabel('Number of maximum missed beacons', fontsize=20)
    plt.ylabel('Throughput (Mbps)', fontsize=20)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 20)
    plt.legend(handles=[th_box_label], loc='best')

    plot_filename_th = f'../Graphs/{out_prefix}-roaming/th-{out_prefix}{reliability_str}-roaming-{db_part}.pdf'
    plt.savefig(plot_filename_th, dpi=1200)

    # Plot reliability (Simulation vs Analysis)
    rel_data = [wifi_ehr[wifi_ehr['maxMissedBeacons'] == val]['PacketReceivedReliability'].values for val in sorted(wifi_ehr['maxMissedBeacons'].unique())]
    positions = sorted(wifi_ehr['maxMissedBeacons'].unique())

    plt.figure()
    rel_box = plt.violinplot(rel_data, 
                      positions=positions,
                      vert=True,
                      widths=0.7,
                      showmeans=False,
                      showmedians=True,
                      showextrema=True)
    rel_box_label = mpatches.Patch(facecolor='skyblue', edgecolor='navy', 
                          label='WiFi 8 JT (ns-3 Simulation)')
    
    plt.xlabel('Number of maximum missed beacons', fontsize=20)
    plt.ylabel('Reliability (%)', fontsize=20)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 100)
    plt.legend(handles=[rel_box_label], loc='best')
    
    plot_filename_rel = f'../Graphs/{out_prefix}-roaming/rel-{out_prefix}{reliability_str}-roaming-{db_part}.pdf'
    plt.savefig(plot_filename_rel, dpi=1200)

    # Plot delay (Simulation vs Analysis)
    delay_data = [wifi_ehr[wifi_ehr['maxMissedBeacons'] == val]['ChannelAccessDelay'].values for val in sorted(wifi_ehr['maxMissedBeacons'].unique())]
    positions = sorted(wifi_ehr['maxMissedBeacons'].unique())

    plt.figure()
    delay_box = plt.violinplot(delay_data, 
                      positions=positions,
                      vert=True,
                      widths=0.7,
                      showmeans=False,
                      showmedians=True,
                      showextrema=True)
    delay_box_label = mpatches.Patch(facecolor='skyblue', edgecolor='navy', 
                          label='WiFi 8 JT (ns-3 Simulation)')
    
    plt.xlabel('Number of maximum missed beacons', fontsize=20)
    plt.ylabel('Delay (ms)', fontsize=20)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 5)
    plt.legend(handles=[delay_box_label], loc='best')
    
    plot_filename_delay = f'../Graphs/{out_prefix}-roaming/ch-delay-{out_prefix}{reliability_str}-roaming-{db_part}.pdf'
    plt.savefig(plot_filename_delay, dpi=1200)

    # Plot assoc delay (Simulation vs Analysis)
    assoc_delay_data = [filter_extreme_values(wifi_ehr[wifi_ehr['maxMissedBeacons'] == val]['AssociationDelay'].values) for val in sorted(wifi_ehr['maxMissedBeacons'].unique())]
    positions = sorted(wifi_ehr['maxMissedBeacons'].unique())

    plt.figure()
    assoc_delay_box = plt.violinplot(assoc_delay_data, 
                      positions=positions,
                      vert=True,
                      widths=0.7,
                      showmeans=False,
                      showmedians=True,
                      showextrema=True)
    assoc_delay_box_label = mpatches.Patch(facecolor='skyblue', edgecolor='navy', 
                          label='WiFi 8 JT (ns-3 Simulation)')
    
    plt.xlabel('Number of maximum missed beacons', fontsize=20)
    plt.ylabel('Association Delay (s)', fontsize=20)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 20)
    plt.legend(handles=[assoc_delay_box_label], loc='best')
    
    plot_filename_assoc_delay = f'../Graphs/{out_prefix}-roaming/assoc-delay-{out_prefix}{reliability_str}-roaming-{db_part}.pdf'
    plt.savefig(plot_filename_assoc_delay, dpi=1200)

    # Plot assoc count (Simulation vs Analysis)
    assoc_count_data = [filter_extreme_values(wifi_ehr[wifi_ehr['maxMissedBeacons'] == val]['AssociationCount'].values) for val in sorted(wifi_ehr['maxMissedBeacons'].unique())]
    positions = sorted(wifi_ehr['maxMissedBeacons'].unique())

    plt.figure()
    assoc_count_box = plt.violinplot(assoc_count_data, 
                      positions=positions,
                      vert=True,
                      widths=0.7,
                      showmeans=False,
                      showmedians=True,
                      showextrema=True)
    assoc_count_box_label = mpatches.Patch(facecolor='skyblue', edgecolor='navy', 
                          label='WiFi 8 JT (ns-3 Simulation)')
    
    plt.xlabel('Number of maximum missed beacons', fontsize=20)
    plt.ylabel('(Re)Association Count', fontsize=20)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(0, 10)
    plt.show()
    plt.legend(handles=[assoc_count_box_label], loc='best')
    
    plot_filename_assoc_delay = f'../Graphs/{out_prefix}-roaming/assoc-count-{out_prefix}{reliability_str}-roaming-{db_part}.pdf'
    plt.savefig(plot_filename_assoc_delay, dpi=1200)

if __name__ == "__main__":
    main()

        
