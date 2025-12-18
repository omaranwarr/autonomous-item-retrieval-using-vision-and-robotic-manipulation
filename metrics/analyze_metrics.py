import csv
import numpy as np

FILE = "pickplace_metrics.csv"

cols = [
    "timestamp","ik","plan_hover","plan_grasp","plan_lift","plan_place",
    "exec_hover","exec_grasp","exec_lift","exec_place","pipeline"
]

data = {c: [] for c in cols[1:]}  # skip timestamp

with open(FILE) as f:
    reader = csv.reader(f)
    next(reader)
    for row in reader:
        for i, c in enumerate(cols[1:], start=1):
            data[c].append(float(row[i]))

print("\n===== METRIC REPORT =====")
for k,v in data.items():
    print(f"{k:15s} avg={np.mean(v):.4f}s  min={np.min(v):.4f}s  max={np.max(v):.4f}s")

