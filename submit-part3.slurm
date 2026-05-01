#!/bin/bash
#SBATCH --job-name=run-part3
#SBATCH --partition=general
#SBATCH --time=6:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --output=run-part3-%x_%A_%a.out
#SBATCH --error=run-part3-%x_%A_%a.err
#SBATCH --array=0-3  # 2 pintools x 2 benchmarks = 4 combos (0-3)

cd "$SLURM_SUBMIT_DIR" || exit 1
export PIN_ROOT=/shared/courseSharedFolders/162593outer/162593/spack/opt/spack/linux-cascadelake/intel-pin-3.31-icrnczcau2lf67etmw5jlzavqlz2hdug

# Tools and Benchmarks

# TODO: change this to your specific .so file!
# to do that, (1) add your custom .cpp file to the makefile.rules, and then (2) execute 'make' from the predictors directory to compile 
PINTOOLS=(
    predictors/obj-intel64/part3-perceptron-tage.so
    predictors/obj-intel64/part3-perceptron-training-freezing.so
)
# TODO: change this to reflect your custom file name!
PINTOOL_NAMES=(
    "perceptron-tage"
    "perceptron-training-freezing"
) 
# NOTE: DO NOT CHANGE FOR PERCEPTRON!
# If you are testing more than 1 pintool at a time, make copies of this string
PINTOOL_ARGS=(
    ""
    ""
)
# TODO: uncomment which benchmark you wish to test!
# start with libquantum, move to dealII if libquantum finishes stronger
BENCHMARKS=(
    ./benchmarks/dealII_O3
    ./benchmarks/libquantum_O3
)

# Map SLURM array index to (pintool, benchmark) combo
NUM_BENCHMARKS=${#BENCHMARKS[@]}
PINTOOL_IDX=$(( SLURM_ARRAY_TASK_ID / NUM_BENCHMARKS ))
BENCHMARK_IDX=$(( SLURM_ARRAY_TASK_ID % NUM_BENCHMARKS ))
PINTOOL="${PINTOOLS[$PINTOOL_IDX]}"
PINTOOL_NAME="${PINTOOL_NAMES[$PINTOOL_IDX]}"
PINTOOL_ARGS_STR="${PINTOOL_ARGS[$PINTOOL_IDX]}"
BENCHMARK="${BENCHMARKS[$BENCHMARK_IDX]}"

# Benchmark name and args
case "$BENCHMARK" in
    *libquantum*) BENCHMARK_ARGS="400 25" ; BENCHMARK_NAME="quantum" ;;
    *dealII*)     BENCHMARK_ARGS="10"     ; BENCHMARK_NAME="dealII"  ;;
esac

# Create output file name
OUTPUT_FILE="saved-results/part3/${PINTOOL_NAME}_DEFAULTARGSV2_${BENCHMARK_NAME}.out"

# Run the results
echo "=== Job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} ==="
echo "    Pintool:   $PINTOOL_NAME  ($PINTOOL)"
echo "    Args:      ${PINTOOL_ARGS_STR:-<none>}"
echo "    Benchmark: $BENCHMARK_NAME  ($BENCHMARK $BENCHMARK_ARGS)"
echo "    Output:    $OUTPUT_FILE"
echo "    Started:   $(date)"

mkdir -p saved-results/part1 logs
$PIN_ROOT/pin -t $PINTOOL $PINTOOL_ARGS_STR -o $OUTPUT_FILE -- $BENCHMARK $BENCHMARK_ARGS
echo "=== Done: $(date) ==="
