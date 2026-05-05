#!/bin/bash
#SBATCH --job-name=run-part3-sweep
#SBATCH --partition=general
#SBATCH --time=6:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --output=run-part3-sweep-num-perceptrons_%A_%a.out
#SBATCH --error=run-part3-sweep-num-perceptrons_%A_%a.err
#SBATCH --array=0-15   # 8 NUM_PERCEPTRON sizes x 2 benchmarks = 16 combos

cd "$SLURM_SUBMIT_DIR" || exit 1
export PIN_ROOT=/shared/courseSharedFolders/162593outer/162593/spack/opt/spack/linux-cascadelake/intel-pin-3.31-icrnczcau2lf67etmw5jlzavqlz2hdug

# NUM_PERCEPTRONS sizes to sweep
NUM_PERCEPTRONS_SIZES=(16 32 64 128 256 512 1024 2048)

# Benchmarks
BENCHMARKS=(
    "./benchmarks/hmmer_O3"
)
# Map SLURM array index to (size, benchmark) combo
NUM_BENCHMARKS=${#BENCHMARKS[@]}
SIZE_IDX=$(( SLURM_ARRAY_TASK_ID / NUM_BENCHMARKS ))
BENCHMARK_IDX=$(( SLURM_ARRAY_TASK_ID % NUM_BENCHMARKS ))

NUM_PERC=${NUM_PERCEPTRONS_SIZES[$SIZE_IDX]}
BENCHMARK=${BENCHMARKS[$BENCHMARK_IDX]}

case "$BENCHMARK" in
    *hmmer*)
        BENCHMARK_ARGS=(benchmarks/inputs/nph3.hmm benchmarks/inputs/swiss41)
        BENCHMARK_NAME="hmmer"
        ;;
    *libquantum*)
        BENCHMARK_ARGS=(400 25)
        BENCHMARK_NAME="quantum"
        ;;
    *dealII*)
        BENCHMARK_ARGS=(10)
        BENCHMARK_NAME="dealII"
        ;;
    *)
        echo "Unknown benchmark: $BENCHMARK"
        exit 1
        ;;
esac

PINTOOL=predictors/obj-intel64/part3-perceptron-predictor.so
PINTOOL_NAME="perceptron-sweep-numperc"

# Create output file name
OUTPUT_FILE="saved-results/part3/perceptron_sweep_numperc_${NUM_PERC}_${BENCHMARK_NAME}.out"

# Run the results
echo "=== Job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} ==="
echo "    Pintool:         $PINTOOL_NAME  ($PINTOOL)"
echo "    NUM_PERCEPTRONS: $NUM_PERC"
echo "    Benchmark:       $BENCHMARK_NAME  ($BENCHMARK $BENCHMARK_ARGS)"
echo "    Output:          $OUTPUT_FILE"
echo "    Started:         $(date)"

mkdir -p saved-results/part3 logs
$PIN_ROOT/pin -t $PINTOOL -n $NUM_PERC -o $OUTPUT_FILE -- $BENCHMARK $BENCHMARK_ARGS
echo "=== Done: $(date) ==="