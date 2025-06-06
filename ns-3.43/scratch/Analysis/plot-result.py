import pandas as pd
import matplotlib.pyplot as plt

# Read the data file
df = pd.read_csv('wifi-ehr-static-scenario/sorted-wifi-ehr-results-reliability.txt', header=None, 
                 names=['seedNumber', 'numBSS', 'Throughput', 'PacketDropReliability', 
                        'PacketReceivedReliability', 'EndToEndDelay', 'ChannelAccessDelay'])

# Clean the data (remove units from delay columns)
df['EndToEndDelay'] = df['EndToEndDelay'].str.replace('ms', '').str.replace('+', '').astype(float)
df['ChannelAccessDelay'] = df['ChannelAccessDelay'].str.replace('ms', '').str.replace('+', '').astype(float)

# Set style for all plots
plt.rcParams['figure.figsize'] = (10, 6)
plt.rcParams['grid.color'] = '0.8'
plt.rcParams['grid.linestyle'] = '--'
plt.rcParams['grid.linewidth'] = 0.5

# Function to create matplotlib boxplots
def create_boxplot(data, x_col, y_col, title, ylabel, ymin, ymax):
    # Group data by x_col and prepare data for boxplot
    grouped_data = [data[data[x_col] == val][y_col].values for val in sorted(data[x_col].unique())]
    positions = sorted(data[x_col].unique())
    
    plt.figure(dpi=150)
    box = plt.boxplot(grouped_data, 
                      positions=positions,
                      notch=False,
                      vert=True,
                      patch_artist=False,
                      widths=0.7,
                     #  showmeans=True,
                      showfliers=True,
                     #  whiskerprops=dict(color="orange"),
                     #  capprops=dict(color="green"),
                      medianprops=dict(color="red"))
                     #  meanprops=dict(marker='D', markersize=8),
                     #  flierprops=dict(marker='o', color='yellow', alpha=0.5))
    
#     plt.title(title, fontsize=14)
    plt.xlabel('Number of BSS', fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.grid(True, alpha=0.5)
    plt.tight_layout()
    plt.ylim(ymin, ymax)
    plt.show()

#     plt.savefig('indoor-random-walk-scenario.pdf',
#            bbox_inches='tight',
#            dpi=300)

# Throughput Plot
create_boxplot(df, 'numBSS', 'Throughput', 
               'Throughput Across Different BSS Counts', 
               'Throughput (Mbps)',0, 150)

# # Packet Received Reliability Plot
# create_boxplot(df, 'numBSS', 'PacketReceivedReliability', 
#                'Packet Received Reliability Across Different BSS Counts', 
#                'Packet Received Reliability (%)')

# # End-To-End Delay Plot
# create_boxplot(df, 'numBSS', 'EndToEndDelay', 
#                'End-To-End Delay Across Different BSS Counts', 
#                'End-To-End Delay (ms)')

# Channel Access Delay Plot
create_boxplot(df, 'numBSS', 'ChannelAccessDelay', 
               'Channel Access Delay Across Different BSS Counts', 
               'Channel Access Delay (ms)',0,2)

# Packet Drop Reliability Plot
create_boxplot(df, 'numBSS', 'PacketDropReliability', 
               'Packet Drop Reliability Across Different BSS Counts', 
               'Packet Drop Reliability (%)',95, 100)

print("All plots displayed.")