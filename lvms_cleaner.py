import pandas as pd

# Input and output file paths
input_file = "lvms_rc_mincurv_iqp_traj_race_cl.csv"
output_file = "lvms_rc_mincurv_iqp_traj_race_cl_cleaned.csv"

# Read skipping first three lines
df = pd.read_csv(input_file, skiprows=3, sep=';', engine='python')

# Remove leading/trailing whitespace from column names
df.columns = [col.strip() for col in df.columns]

# Save with new formatting
df.to_csv(output_file, index=False)

