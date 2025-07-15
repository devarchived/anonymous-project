import pandas as pd
from datetime import datetime
import os

def synchronize_and_merge_logs(file_former, file_latter, output_file):
    # Load CSVs
    df_former = pd.read_csv(file_former)
    df_latter = pd.read_csv(file_latter)

    # Parse frame.time to datetime (remove timezone string if present)
    df_former["parsed_time"] = pd.to_datetime(df_former["frame.time"].str.replace(" CEST", ""), format="%b %d, %Y %H:%M:%S.%f")
    df_latter["parsed_time"] = pd.to_datetime(df_latter["frame.time"].str.replace(" CEST", ""), format="%b %d, %Y %H:%M:%S.%f")

    # Determine which file is earlier
    if df_former["parsed_time"].iloc[0] > df_latter["parsed_time"].iloc[0]:
        # Swap if necessary
        df_former, df_latter = df_latter, df_former
        print(f"Swapped order for: {file_former} and {file_latter}")

    # Find the matching frame number in former log
    latter_start_time = df_latter["parsed_time"].iloc[0]
    matching_frame = df_former[df_former["parsed_time"] <= latter_start_time].iloc[-1]
    offset = matching_frame["frame.number"]

    # Offset the frame numbers in latter log
    df_latter["frame.number"] = df_latter["frame.number"] + offset

    # Drop helper column before merging
    df_former = df_former.drop(columns=["parsed_time"])
    df_latter = df_latter.drop(columns=["parsed_time"])

    # Merge both logs
    combined_df = pd.concat([df_former, df_latter], ignore_index=True)

    # Resolve duplicates: keep the one with lower frame.time_delta_displayed
    combined_df = combined_df.sort_values(by=["frame.number", "frame.time_delta_displayed"], ascending=[True, True])
    combined_df = combined_df.drop_duplicates(subset=["frame.number"], keep="first")

    # Final sort by frame.number
    combined_df = combined_df.sort_values(by="frame.number").reset_index(drop=True)

    # Save to output
    combined_df.to_csv(output_file, index=False)
    print(f"Merged log saved to {output_file}")

# Process multiple file pairs
for i in range(1, 11):
    file_former = f"results-2/ehr-results/pcap/ehr-ap1-pcap-{i}.csv"
    file_latter = f"results-2/ehr-results/pcap/ehr-ap2-pcap-{i}.csv"
    output_file = f"results-2/ehr-results/pcap/ehr-reliability-{i}.csv"

    if os.path.exists(file_former) and os.path.exists(file_latter):
        synchronize_and_merge_logs(file_former, file_latter, output_file)
    else:
        print(f"Skipping pair {i}: one or both files are missing.")
