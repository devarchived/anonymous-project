import re

# Output CSV file
output_file = "final-results/ax5-log-results.txt"

# Write header
with open(output_file, "w") as out:
    out.write("rxBytes,dataRate,jitter,packetLost,totalPackets\n")

# Regex pattern to extract data from receiver line
pattern = re.compile(
    r"\[\s*\d+\]\s+\d+\.\d+-\d+\.\d+\s+sec\s+([\d.]+)\s+MBytes\s+([\d.]+)\s+Mbits/sec\s+([\d.]+)\s+ms\s+(\d+)/(\d+).+receiver"
)


# Process each file
for i in range(1, 11):
    filename = f"results-2/ax5-results/logs/ap2/iperf3_client_ax5_ap2_AX5_{i}.log"
    try:
        with open(filename, "r") as f:
            for line in f:
                if "receiver" in line:
                    match = pattern.search(line)
                    if match:
                        rxBytes = match.group(1)
                        dataRate = match.group(2)
                        jitter = match.group(3)
                        packetLost = match.group(4)
                        totalPackets = match.group(5)
                        with open(output_file, "a") as out:
                            out.write(f"{rxBytes},{dataRate},{jitter},{packetLost},{totalPackets}\n")
                        break  # Stop after first match
    except FileNotFoundError:
        print(f"File {filename} not found. Skipping.")
