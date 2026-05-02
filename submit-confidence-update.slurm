#!/bin/bash
#SBATCH --job-name=run-part3
#SBATCH --partition=general
#SBATCH --time=6:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --output=run-part3-%A_%a.out
#SBATCH --error=run-part3-%A_%a.err
#SBATCH --array=0-9

set -euo pipefail

cd "$SLURM_SUBMIT_DIR" || exit 1

export PIN_ROOT=/shared/courseSharedFolders/162593outer/162593/spack/opt/spack/linux-cascadelake/intel-pin-3.31-icrnczcau2lf67etmw5jlzavqlz2hdug

PINTOOL="predictors/obj-intel64/part3-perceptron-confidence-update.so"
PINTOOL_NAME="perceptron-custom-multcap"

BENCHMARKS=(
    ./benchmarks/libquantum_O3
    ./benchmarks/dealII_O3
)

MULT_CAPS=(1 2 3 4 5)

NUM_BENCHMARKS=${#BENCHMARKS[@]}
NUM_MULT_CAPS=${#MULT_CAPS[@]}

BENCHMARK_IDX=$(( SLURM_ARRAY_TASK_ID % NUM_BENCHMARKS ))
MULT_CAP_IDX=$(( SLURM_ARRAY_TASK_ID / NUM_BENCHMARKS ))

BENCHMARK="${BENCHMARKS[$BENCHMARK_IDX]}"
MULT_CAP="${MULT_CAPS[$MULT_CAP_IDX]}"

case "$BENCHMARK" in
    *libquantum*) BENCHMARK_ARGS="400 25" ; BENCHMARK_NAME="quantum" ;;
    *dealII*)     BENCHMARK_ARGS="10"     ; BENCHMARK_NAME="dealII"  ;;
esac

OUTPUT_DIR="saved-results/part3-saved-logs"
mkdir -p "$OUTPUT_DIR" logs

OUTPUT_FILE="${OUTPUT_DIR}/${PINTOOL_NAME}_multcap${MULT_CAP}_${BENCHMARK_NAME}.out"

echo "=== Job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} ==="
echo "    Pintool:   $PINTOOL_NAME ($PINTOOL)"
echo "    Benchmark: $BENCHMARK_NAME ($BENCHMARK $BENCHMARK_ARGS)"
echo "    mult_cap:  $MULT_CAP"
echo "    Output:    $OUTPUT_FILE"
echo "    Started:   $(date)"

$PIN_ROOT/pin -t "$PINTOOL" -mult_cap "$MULT_CAP" -o "$OUTPUT_FILE" -- "$BENCHMARK" $BENCHMARK_ARGS

echo "=== Done: $(date) ==="