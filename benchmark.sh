#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <input_file>"
    exit 1
fi

INPUT_FILE="$1"
BASENAME=$(basename "$INPUT_FILE")
BASENAME_NO_EXT="${BASENAME%.osm.pbf}"

for i in {1..10}; do
    OUTPUT_FILE="benchmarks/${i}_${BASENAME_NO_EXT}.txt"
    echo "Run #$i -> $OUTPUT_FILE"
    ./osm_release "$INPUT_FILE" > "$OUTPUT_FILE"
done

echo "All 10 runs completed."