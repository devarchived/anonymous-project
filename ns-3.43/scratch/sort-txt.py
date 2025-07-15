# Read the file and sort the lines
input_files = [
    'wifi-ehr-roaming-scenario/result-logs/wifi-ehr-roaming-results-unsaturated-0dB.txt',
    'wifi-ehr-roaming-scenario/result-logs/wifi-ehr-roaming-results-unsaturated-0dB-reliability.txt',
    'wifi-eht-roaming-scenario/result-logs/wifi-eht-roaming-results-unsaturated-3dB.txt',
    # Add more input files as needed
]

for input_file in input_files:
    parts = input_file.rsplit('/', 1)
    output_file = f"{parts[0]}/sorted-{parts[1]}" if len(parts) > 1 else f"sorted-{input_file}"

    with open(input_file, 'r') as f:
        lines = f.readlines()

    sorted_lines = sorted(
        lines,
        key=lambda line: (int(line.split(',')[1]), int(line.split(',')[0]))
    )

    with open(output_file, 'w') as f:
        f.writelines(sorted_lines)

    print(f"Sorted data saved to {output_file}")
