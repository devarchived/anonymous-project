import pandas as pd
import os

def compile_log_summaries(input_files, output_file="compiled_summary.csv"):
    results = []

    for file_path in input_files:
        if not os.path.exists(file_path):
            print(f"File not found: {file_path}, skipping.")
            continue

        df = pd.read_csv(file_path)

        total_packets = len(df)
        total_bytes = df["ip.len"].sum()
        rx_bytes_mbytes = total_bytes / 1e6
        data_rate_mbps = (rx_bytes_mbytes * 8) / 30
        jitter = 0

        frame_numbers = df["frame.number"].sort_values().values
        packet_lost = sum((frame_numbers[i + 1] - frame_numbers[i]) > 1 for i in range(len(frame_numbers) - 1))

        packet_delay = df["frame.time_delta_displayed"].mean()

        results.append({
            "rxBytes": rx_bytes_mbytes,
            "dataRate": data_rate_mbps,
            "jitter": jitter,
            "packetLost": packet_lost,
            "totalPackets": total_packets,
            "packetDelay": packet_delay
        })

    summary_df = pd.DataFrame(results)
    summary_df.to_csv(output_file, index=False)
    print(f"Summary saved to {output_file}")

input_files = [f"results-2/ehr-results/pcap/ehr-reliability-{i}.csv" for i in range(1, 11)]
output_summary_file = "final-results/ehr-realibility-log-with-delay.txt"
compile_log_summaries(input_files, output_summary_file)
