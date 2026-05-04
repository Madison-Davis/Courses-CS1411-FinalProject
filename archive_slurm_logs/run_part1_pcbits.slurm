#!/bin/bash
#SBATCH --job-name=pcnenbits-run-part1
#SBATCH --partition=general
#SBATCH --time=5:00:00
#SBATCH --ntasks=3
#SBATCH --cpus-per-task=1
#SBATCH --output=run2-part1-%x_%A_%a.out
#SBATCH --error=run2-part1-%x_%A_%a.err
#SBATCH --array=0-13   # 3 PC bits × 2 benchmarks = 12 jobs

cd "$SLURM_SUBMIT_DIR" || exit 1

export PIN_ROOT=/shared/courseSharedFolders/162593outer/162593/spack/opt/spack/linux-cascadelake/intel-pin-3.31-icrnczcau2lf67etmw5jlzavqlz2hdug

PINTOOLS=(predictors/obj-intel64/part1-tage-predictor.so)
PINTOOL_NAMES=("tage")

BENCHMARKS=(
    ./benchmarks/dealII_O3
    ./benchmarks/libquantum_O3
)

# Sweep values
PC_BITS=(4 8 10 12 14 16)

NUM_BENCHMARKS=${#BENCHMARKS[@]}   # 2
NUM_PC=${#PC_BITS[@]}              # 3

# -------------------------
# Index mapping
# -------------------------
PC_IDX=$(( SLURM_ARRAY_TASK_ID / NUM_BENCHMARKS ))
BENCHMARK_IDX=$(( SLURM_ARRAY_TASK_ID % NUM_BENCHMARKS ))

PC=${PC_BITS[$PC_IDX]}
NENTRIES=$((2**PC))

PINTOOL="${PINTOOLS[0]}"
PINTOOL_NAME="${PINTOOL_NAMES[0]}"
BENCHMARK="${BENCHMARKS[$BENCHMARK_IDX]}"

# -------------------------
# Benchmark args
# -------------------------
case "$BENCHMARK" in
    *libquantum*) BENCHMARK_ARGS="400 25" ; BENCHMARK_NAME="quantum" ;;
    *dealII*)     BENCHMARK_ARGS="10"     ; BENCHMARK_NAME="dealII"  ;;
esac

# -------------------------
# Output naming
# -------------------------
OUTPUT_FILE="saved-results/part1_table/${PINTOOL_NAME}_pc${PC}_n${NENTRIES}_${BENCHMARK_NAME}.out"

# -------------------------
# Logging
# -------------------------
echo "=== Job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} ==="
echo "    PC BITS:   $PC"
echo "    NENTRIES:  $NENTRIES"
echo "    Benchmark: $BENCHMARK_NAME"
echo "    Output:    $OUTPUT_FILE"
echo "    Started:   $(date)"

mkdir -p saved-results/part1_pc_nenbits logs

# -------------------------
# Run
# -------------------------
$PIN_ROOT/pin -t $PINTOOL \
    -pcbits $PC \
    -nentries $NENTRIES \
    -o $OUTPUT_FILE \
    -- $BENCHMARK $BENCHMARK_ARGS

echo "=== Done: $(date) ==="