import re
import statistics

num_runs = 10

# Preprocessing step regex
preprocess_step_regex = re.compile(
    r"(Assigned areas to buildings|Street names assigned|Inverted index built|Suffix array built)\s*\[([0-9\.]+)(ms|µs|s)?\]"
)
# Total preprocessing time regex
preprocess_total_regex = re.compile(r"Preprocessing done\s*\[([0-9\.]+)s\]")

# Query regex
query_inverted_index_regex = re.compile(r"search_buildings_inverted_index\(\) ran in \[([0-9\.]+)(µs|ms|s)?\]")
query_suffix_array_regex = re.compile(r"search_buildings\(\) ran in \[([0-9\.]+)(µs|ms|s)?\]")
suffix_array_closest_flag = False
query_suffix_array_closest_regex = re.compile(r"Suffix Array Closest:")

# Data storage
preprocessing_times_all = {step: [] for step in [
    "Assigned areas to buildings",
    "Street names assigned",
    "Inverted index built",
    "Suffix array built"
]}
preprocessing_total_times = []

query_times = {
    "inverted_index": [],
    "suffix_array": [],
    "suffix_array_closest": []
}

# Read files
for i in range(1, num_runs+1):
    filename = f"{i}_stuttgart-regbez-251010.txt"
    with open(filename, "r") as f:
        content = f.readlines()

    current_block_closest = False

    for line in content:
        # Check Suffix Array Closest start
        if query_suffix_array_closest_regex.search(line):
            current_block_closest = True
            continue

        # Preprocessing steps
        step_match = preprocess_step_regex.search(line)
        if step_match:
            step_name, time_val, unit = step_match.groups()
            if unit == "µs":
                time_s = float(time_val)/1_000_000
            elif unit == "ms":
                time_s = float(time_val)/1000
            else:
                time_s = float(time_val)
            preprocessing_times_all[step_name].append(time_s)

        # Total preprocessing
        total_match = preprocess_total_regex.search(line)
        if total_match:
            preprocessing_total_times.append(float(total_match.group(1)))

        # Queries
        if "search_buildings_inverted_index" in line:
            match = query_inverted_index_regex.search(line)
            if match:
                val, unit = match.groups()
                if unit == "µs":
                    val_s = float(val)/1_000_000
                elif unit == "ms":
                    val_s = float(val)/1000
                else:
                    val_s = float(val)
                query_times["inverted_index"].append(val_s)

        elif "search_buildings()" in line:
            match = query_suffix_array_regex.search(line)
            if match:
                val, unit = match.groups()
                if unit == "µs":
                    val_s = float(val)/1_000_000
                elif unit == "ms":
                    val_s = float(val)/1000
                else:
                    val_s = float(val)
                if current_block_closest:
                    query_times["suffix_array_closest"].append(val_s)
                else:
                    query_times["suffix_array"].append(val_s)

# Print preprocessing step times
print("=== Preprocessing Step Times (all runs) ===")
for step, times in preprocessing_times_all.items():
    print(f"{step}: {times}")

# Print total preprocessing times
print("\nTotal preprocessing times:", preprocessing_total_times)

# Print query times split by block
print("\n=== Query Times (seconds) ===")
for block, times in query_times.items():
    print(f"{block}: {times}")

# Compute and print averages
print("\n=== Averages ===")
for step, times in preprocessing_times_all.items():
    avg = statistics.mean(times) if times else 0
    print(f"{step}: {avg:.4f}s")

if preprocessing_total_times:
    print(f"Total preprocessing average: {statistics.mean(preprocessing_total_times):.4f}s")

for block, times in query_times.items():
    avg = statistics.mean(times) if times else 0
    print(f"{block} average: {avg*1000:.3f}ms")