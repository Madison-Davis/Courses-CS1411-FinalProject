#!/bin/bash
#SBATCH --job-name=run-part2
#SBATCH --partition=general
#SBATCH --time=4:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --output=run-part2-%x_%A_%a.out
#SBATCH --error=run-part2-%x_%A_%a.err
#SBATCH --array=0-1   # 2 pintools x 1 benchmarks = 2 combos (0-1)

cd "$SLURM_SUBMIT_DIR" || exit 1
export PIN_ROOT=/shared/courseSharedFolders/162593outer/162593/spack/opt/spack/linux-cascadelake/intel-pin-3.31-icrnczcau2lf67etmw5jlzavqlz2hdug

# Tools and Benchmarks
PINTOOLS=(
    predictors/obj-intel64/part1-tage-predictor.so
    predictors/obj-intel64/part2-tage-custom-confidence-predictor.so 
    # TODO: change this to your specific .so file!
    # to do that, (1) add your custom .cpp file to the makefile.rules, and then (2) execute 'make' from the predictors directory to compile 
)
PINTOOL_NAMES=("tage" "tage-custom-confidence") # TODO: change the custom one to your name!
PINTOOL_ARGS=(
    "-L1 12 -R 2 -ntables 10 -nentries 4096 -pcbits 12 -tagbits 10 -prob 3" # NOTE: this is our baseline set!
    "-L1 12 -R 2 -ntables 10 -nentries 4096 -pcbits 12 -tagbits 10 -prob 3" 
)
BENCHMARKS=(
    #./benchmarks/dealII_O3 # TODO: for early testing, only need to do libquantum... if libquantum shows custom is better, try dealII!
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
OUTPUT_FILE="saved-results/part2/${PINTOOL_NAME}_DEFAULTARGS_${BENCHMARK_NAME}.out"

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
