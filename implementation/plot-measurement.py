import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Set global plot style
plt.rcParams.update({
    'font.size': 16,
    'axes.labelsize': 16,
    'axes.titlesize': 18,
    'xtick.labelsize': 14,
    'ytick.labelsize': 14,
    'legend.fontsize': 14,
    'axes.linewidth': 1.5,
    'grid.linewidth': 0.8,
})

# Input file paths and labels
files = {
    '802.11ax 2.4GHz': 'final-results/ax2-log-with-delay.txt',
    '802.11ax 5GHz': 'final-results/ax5-log-with-delay.txt',
    '802.11bn\ndefault-mode': 'final-results/ehr-default-log-with-delay.txt',
    '802.11bn\nreliability-mode': 'final-results/ehr-reliability-log-with-delay.txt'
}

# Output directory for plots
output_dir = 'plots'
os.makedirs(output_dir, exist_ok=True)

# Containers for metrics
throughput_data = []
reliability_data = []
delay_data = []

# Load and process data
for label, filename in files.items():
    df = pd.read_csv(filename)
    df.columns = df.columns.str.lower()  # Normalize column names

    # Compute metrics
    throughput = df['datarate']
    reliability = (df['totalpackets'] - df['packetlost']) / df['totalpackets'] * 100
    delay = df['packetdelay'] * 1000  # Convert to ms

    # Append labeled data
    throughput_data.extend([(label, val) for val in throughput])
    reliability_data.extend([(label, val) for val in reliability])
    delay_data.extend([(label, val) for val in delay])

# Convert to DataFrames
df_throughput = pd.DataFrame(throughput_data, columns=['Config', 'Throughput (Mbps)'])
df_reliability = pd.DataFrame(reliability_data, columns=['Config', 'Reliability (%)'])
df_delay = pd.DataFrame(delay_data, columns=['Config', 'Delay (ms)'])

def plot_violin(df, y_label, ylim, filename):
    plt.figure(figsize=(10, 6))
    
    # Group the data by 'Config' and extract values for violinplot
    grouped = [df[df['Config'] == label].iloc[:, 1].values for label in df['Config'].unique()]
    positions = range(len(grouped))
    labels = df['Config'].unique()
    
    plt.violinplot(dataset=grouped, positions=positions, vert=True, widths=0.7,
                   showmeans=False, showmedians=True, showextrema=True)
    
    plt.xticks(ticks=positions, labels=labels)
    plt.ylabel(y_label)
    plt.grid(True, axis='y', alpha=0.5)
    plt.ylim(ylim)
    plt.tight_layout()
    plt.savefig(filename, dpi=1200)
    plt.close()

# Generate and save plots
plot_violin(df_throughput, 'Throughput (Mbps)', (0, 50), os.path.join(output_dir, 'th_measurement.pdf'))
plot_violin(df_reliability, 'Reliability (%)', (99, 101), os.path.join(output_dir, 'rel_measurement.pdf'))
plot_violin(df_delay, 'Delay (ms)', (0, 2), os.path.join(output_dir, 'delay_measurement.pdf'))
