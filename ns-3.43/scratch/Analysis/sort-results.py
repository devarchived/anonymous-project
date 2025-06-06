#python3 sort-results.py wifi-ehr-static-scenario/wifi-ehr-results.txt wifi-ehr-static-scenario/sorted-wifi-ehr-results.txt

import sys

def reorder_file(input_file, output_file):
    # Read all lines from the input file
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    # Parse each line into components
    data = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) >= 2:
            try:
                seed = int(parts[0])
                numBss = int(parts[1])
                data.append((seed, numBss, line))
            except ValueError:
                # Skip lines that don't have valid seed/numBss numbers
                continue
    
    # Sort the data:
    # 1. First by seed (1-50) within each numBss group
    # 2. Then by numBss (1-4)
    data.sort(key=lambda x: (x[1], x[0]))
    
    # Write the sorted data to the output file
    with open(output_file, 'w') as f:
        for item in data:
            f.write(item[2] + '\n')

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python reorder_results.py <input_file> <output_file>")
        print("Example: python reorder_results.py wifi-ehr-results.txt sorted-results.txt")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    reorder_file(input_file, output_file)
    print(f"File successfully reordered and saved to {output_file}")