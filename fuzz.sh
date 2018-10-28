#!/bin/bash
# ---------------------------------------------------
# Please make sure that you installed afl before running
# eg : cargo install afl
# ----------------------------------------------------

# Compile using afl
cargo afl build
# Start afl
AFL_SKIP_CPUFREQ=1 cargo afl fuzz -i tests/fuzzing_data -o out target/debug/fuzzing
