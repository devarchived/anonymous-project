import os
import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import scipy
import pandas as pd
import math
import glob

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
    # %% Plotting across wallloss values for fixed maxMissedBeacons
    target_beacons = 4
    input_files = glob.glob('../wifi-eht-roaming-scenario/result-logs/sorted-wifi-eht-roaming-results-unsaturated-*-dB.txt')

    throughput_data = {}
    reliability_data = {}
    ch_delay_data = {}
    assoc_delay_data = {}
    assoc_count_data = {}

    for input_filename in input_files:
        base_name = os.path.basename(input_filename)
        out_prefix = 'wifi8' if 'ehr' in base_name else 'wifi7' if 'eht' in base_name else 'wifi'
        reliability_str = '-reliability' if 'reliability' in base_name else ''

        # Extract wallloss e.g., '5-dB'
        db_part = ''
        for i, part in enumerate(base_name.split('-')):
            if 'dB' in part:
                db_part = f"{base_name.split('-')[i-1]}"
                break
        if not db_part:
            db_part = base_name.split('.txt')[0].split('-')[-2] + ''

        df = pd.read_csv(input_filename, header=None, 
                    names=['seedNumber', 'maxMissedBeacons', 'Throughput', 'PacketDropReliability', 
                            'PacketReceivedReliability', 'EndToEndDelay', 'ChannelAccessDelay','AssociationDelay','AssociationCount'])

        df['EndToEndDelay'] = df['EndToEndDelay'].str.replace('ms', '').str.replace('+', '').astype(float)
        df['ChannelAccessDelay'] = df['ChannelAccessDelay'].str.replace('ms', '').str.replace('+', '').astype(float)
        df['AssociationDelay'] = df['AssociationDelay'].str.replace('ms', '').str.replace('+', '').astype(float) / 1000

        df_filtered = df[df['maxMissedBeacons'] == target_beacons]

        if df_filtered.empty:
            continue

        throughput_data[db_part] = df_filtered['Throughput'].values
        reliability_data[db_part] = df_filtered['PacketReceivedReliability'].values
        ch_delay_data[db_part] = df_filtered['ChannelAccessDelay'].values
        assoc_delay_data[db_part] = filter_extreme_values(df_filtered['AssociationDelay'].values)
        assoc_count_data[db_part] = df_filtered['AssociationCount'].values

    # Sort wallloss values for x-axis
    def sort_db_keys(d):
        return sorted(d.keys(), key=lambda x: int(x.split('-')[0]))

    def plot_violin(metric_dict, ylabel, ylim, filename, out_prefix):
        plt.figure()
        sorted_keys = sort_db_keys(metric_dict)
        data = [metric_dict[k] for k in sorted_keys]
        positions = list(range(len(sorted_keys)))

        plt.violinplot(data, positions=positions, vert=True, widths=0.7,
                       showmeans=False, showmedians=True, showextrema=True)

        plt.xticks(positions, sorted_keys)
        plt.xlabel('Wall Loss (dB)', fontsize=20)
        plt.ylabel(ylabel, fontsize=20)
        plt.grid(True, alpha=0.5)
        plt.tight_layout()
        plt.ylim(ylim)
        plt.show()
        patch = mpatches.Patch(facecolor='skyblue', edgecolor='navy', label='WiFi 7 MLO-STR (ns-3 Simulation)')
        plt.legend(handles=[patch], loc='best')

        os.makedirs(os.path.dirname(filename), exist_ok=True)
        plt.savefig(filename, dpi=1200)

    plot_violin(throughput_data, 'Throughput (Mbps)', (0, 20),
                f'../Graphs/{out_prefix}-roaming/th-{out_prefix}{reliability_str}-fixed-beacon-{target_beacons}.pdf',
                out_prefix)

    plot_violin(reliability_data, 'Reliability (%)', (0, 120),
                f'../Graphs/{out_prefix}-roaming/rel-{out_prefix}{reliability_str}-fixed-beacon-{target_beacons}.pdf',
                out_prefix)

    plot_violin(ch_delay_data, 'Delay (ms)', (0, 10),
                f'../Graphs/{out_prefix}-roaming/ch-delay-{out_prefix}{reliability_str}-fixed-beacon-{target_beacons}.pdf',
                out_prefix)

    plot_violin(assoc_delay_data, 'Association Delay (s)', (0, 20),
                f'../Graphs/{out_prefix}-roaming/assoc-delay-{out_prefix}{reliability_str}-fixed-beacon-{target_beacons}.pdf',
                out_prefix)

    plot_violin(assoc_count_data, '(Re)Association Count', (0, 10),
                f'../Graphs/{out_prefix}-roaming/assoc-count-{out_prefix}{reliability_str}-fixed-beacon-{target_beacons}.pdf',
                out_prefix)

if __name__ == "__main__":
    main()

