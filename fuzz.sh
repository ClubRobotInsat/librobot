#!/bin/bash

# AFL
# ---------------------------------------------------
# Please make sure that you installed afl before running
# eg : cargo install afl
# ----------------------------------------------------

# Compile using afl
#cargo afl build
# Start afl
#AFL_SKIP_CPUFREQ=1 cargo afl fuzz -i tests/fuzzing_data -o out target/debug/fuzzing

# --------------------------------
# Cargo fuzz :
# Please make sure that you have installed cargo fuzz before
# ---------------
 cargo fuzz run -j 8 fuzz_target_1 tests/fuzzing_data -- -max_len=256
