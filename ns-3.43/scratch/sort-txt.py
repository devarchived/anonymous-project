# Read the file, sort the lines, and write back
input_file = 'wifi-ehr-roaming-scenario/wifi-ehr-roaming-results-unsaturated-5-dB.txt'
output_file = 'wifi-ehr-roaming-scenario/sorted-wifi-ehr-roaming-results-unsaturated-5-dB.txt'

with open(input_file, 'r') as f:
    lines = f.readlines()

# Sort lines by first column, then second column (as integers)
sorted_lines = sorted(
    lines,
    key=lambda line: (int(line.split(',')[1]), int(line.split(',')[0]))
)

with open(output_file, 'w') as f:
    f.writelines(sorted_lines)

print(f"Sorted data saved to {output_file}")