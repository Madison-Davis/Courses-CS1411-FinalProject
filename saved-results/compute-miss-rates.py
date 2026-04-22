#!/usr/bin/env python3

"""
compute-miss-rates.py
Located in: saved-results/
Reads all .out files in saved-results/part1/ and computes branch miss rates.
"""

# Imports
import os
import glob

# TODO: change 'part 1' for the part you want to compute miss rates for!
PART1_DIR = os.path.join(os.path.dirname(__file__), "part1") 
OUTPUT_FILE = os.path.join(PART1_DIR, "miss-rates.txt")

# Helper function: parse out information from a file
def parse_out_file(filepath):
    count_seen = None
    count_correct = None
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if line.startswith("Count Seen:"):
                count_seen = int(line.split(":")[1].strip())
            elif line.startswith("Count Correct:"):
                count_correct = int(line.split(":")[1].strip())
    if count_seen and count_seen > 0:
        miss_rate = (1 - count_correct / count_seen) * 100
        return miss_rate
    return None

# Collect all .out files
results = []
for filepath in glob.glob(os.path.join(PART1_DIR, "*.out")):
    name = os.path.splitext(os.path.basename(filepath))[0]
    miss_rate = parse_out_file(filepath)
    if miss_rate is not None:
        results.append((name, miss_rate))
    else:
        results.append((name, "ERROR: missing fields"))

results.sort(key=lambda x: x[1] if isinstance(x[1], float) else float("inf"))

# Write table
col_width = max(len(r[0]) for r in results) + 2
header = f"{'Experiment':<{col_width}} {'Miss Rate (%)':>15}"
separator = "-" * len(header)
lines = [header, separator]
for name, miss_rate in results:
    if isinstance(miss_rate, float):
        lines.append(f"{name:<{col_width}} {miss_rate:>14.4f}%")
    else:
        lines.append(f"{name:<{col_width}} {miss_rate:>15}")
output = "\n".join(lines) + "\n"
with open(OUTPUT_FILE, "w") as f:
    f.write(output)
print(f"Results saved to {OUTPUT_FILE}")
