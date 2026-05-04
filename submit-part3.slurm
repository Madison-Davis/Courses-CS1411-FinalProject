#!/bin/bash
#SBATCH --job-name=run-part3
#SBATCH --partition=general
#SBATCH --time=6:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --output=run-part3-%x_%A_%a.out
#SBATCH --error=run-part3-%x_%A_%a.err
#SBATCH --array=0-3  # 4 pintools x 1 benchmark = 4 combos

cd "$SLURM_SUBMIT_DIR" || exit 1
export PIN_ROOT=/shared/courseSharedFolders/162593outer/162593/spack/opt/spack/linux-cascadelake/intel-pin-3.31-icrnczcau2lf67etmw5jlzavqlz2hdug

# Tools and Benchmarks

# TODO: change this to your specific .so file!
# to do that, (1) add your custom .cpp file to the makefile.rules, and then (2) execute 'make' from the predictors directory to compile 
PINTOOLS=(
    predictors/obj-intel64/part1-1bit-predictor.so
    predictors/obj-intel64/part1-2bit-predictor.so
    predictors/obj-intel64/part1-2bitbht-predictor.so
    predictors/obj-intel64/part1-tage-predictor.so
)
# TODO: change this to reflect your custom file name! 
PINTOOL_NAMES=("1bit" "2bit" "2bit-bht" "tage")

# NOTE: DO NOT CHANGE FOR PERCEPTRON!
# If you are testing more than 1 pintool at a time, make copies of this string
PINTOOL_ARGS=(
    "-s 1024" 
    "-s 1024" 
    "-r 512 -p 512" 
    "-L1 2 -R 2 -ntables 8 -nentries 4096 -pcbits 12 -tagbits 10 -prob 3"
)

# TODO: uncomment which benchmark you wish to test!
# start with libquantum, move to dealII if libquantum finishes stronger
BENCHMARKS=(
    ./benchmarks/hmmer_O3
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
    *hmmer*) BENCHMARK_ARGS="benchmarks/inputs/nph3.hmm benchmarks/inputs/swiss41" ; BENCHMARK_NAME="hmmer" ;;
esac

# Create output file name
OUTPUT_FILE="saved-results/part3/${PINTOOL_NAME}_DEFAULTARGS_${BENCHMARK_NAME}.out"

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
