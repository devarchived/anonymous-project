import subprocess
import os

# Fields you want to extract
fields = [
    "frame.number",
    "frame.time",
    "ip.src",
    "ip.dst",
    "ip.len",
    "frame.time_delta_displayed"
]

# Build field arguments
field_args = []
for field in fields:
    field_args += ["-e", field]

# Static TShark output format arguments
output_args = [
    "-T", "fields",
    "-E", "separator=,",
    "-E", "header=y",
    "-E", "quote=d"
] + field_args

# Process each file
for i in range(1, 11):
    pcap_file = f"results-2/ehr-results/pcap/iperf_ap2_EHR_{i}.pcap"
    csv_file = f"results-2/ehr-results/pcap/ehr-ap2-pcap-{i}.csv"

    if os.path.exists(pcap_file):
        print(f"Processing {pcap_file}...")
        with open(csv_file, "w") as out:
            # Build full TShark command with correct order
            cmd = ["tshark", "-r", pcap_file] + output_args
            subprocess.run(cmd, stdout=out)
        print(f"Saved to {csv_file}")
    else:
        print(f"{pcap_file} not found. Skipping.")
