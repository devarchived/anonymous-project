import pandas as pd
import os

def calculate_average_delays(prefix, count, directory='.'):
    """Calculate average frame.time_delta_displayed for each numbered CSV file."""
    delays = []
    for i in range(1, count + 1):
        file_path = os.path.join(directory, f"{prefix}{i}.csv")
        try:
            df = pd.read_csv(file_path)
            if 'frame.time_delta_displayed' in df.columns:
                avg_delay = df['frame.time_delta_displayed'].mean()
            else:
                print(f"Warning: Column missing in {file_path}")
                avg_delay = None
        except Exception as e:
            print(f"Error reading {file_path}: {e}")
            avg_delay = None
        delays.append(avg_delay)
    return delays

def add_packetdelay_to_log(log_file, output_file, csv_prefix, csv_count, directory='.'):
    # Load log file
    log_path = os.path.join(directory, log_file)
    df_log = pd.read_csv(log_path)

    # Get delay values
    delays = calculate_average_delays(csv_prefix, csv_count, directory)

    # Append new column
    df_log['packetdelay'] = delays

    # Save result
    output_path = os.path.join(directory, output_file)
    df_log.to_csv(output_path, index=False)
    print(f"Updated log saved to {output_file}")

if __name__ == "__main__":
    # Customize your values here:
    log_file = "final-results/ehr5-log-results.txt"
    output_file = "final-results/ehr5-log-with-delay.txt"
    csv_prefix = "results-2/ehr-results/pcap/ehr-ap2-pcap-"
    csv_count = 10
    data_dir = "."

    add_packetdelay_to_log(log_file, output_file, csv_prefix, csv_count, data_dir)
