import pandas as pd

# Define file paths here
input_file1 = "final-results/ehr2-log-with-delay.txt"
input_file2 = "final-results/ehr5-log-with-delay.txt"
output_file = "final-results/ehr-default-log-with-delay.txt"

def combine_csv_metrics(file1, file2, output_file):
    # Load CSV files
    try:
        df1 = pd.read_csv(file1)
        df2 = pd.read_csv(file2)
    except FileNotFoundError as e:
        print(f"File not found: {e.filename}")
        return
    except Exception as e:
        print(f"Error loading files: {e}")
        return

    # Define columns to sum and average
    sum_columns = ['rxBytes', 'dataRate', 'packetLost', 'totalPackets']
    avg_columns = ['jitter', 'packetdelay']

    # Check for required columns
    missing_columns = [col for col in sum_columns + avg_columns if col not in df1.columns or col not in df2.columns]
    if missing_columns:
        print(f"Missing columns in one of the files: {missing_columns}")
        return

    # Initialize result DataFrame
    df_result = pd.DataFrame()

    # Additive columns
    for col in sum_columns:
        df_result[col] = df1[col] + df2[col]

    # Averaged columns
    for col in avg_columns:
        df_result[col] = (df1[col] + df2[col]) / 2

    # Save output
    df_result.to_csv(output_file, index=False)
    print(f"Combined result saved to: {output_file}")

# Run the function with predefined files
combine_csv_metrics(input_file1, input_file2, output_file)
