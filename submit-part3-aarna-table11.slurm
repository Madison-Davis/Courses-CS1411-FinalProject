#!/bin/bash
#SBATCH --job-name=run-tage-pcbias
#SBATCH --partition=general
#SBATCH --time=6:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --output=run-tage-pcbias-%x_%A_%a.out
#SBATCH --error=run-tage-pcbias-%x_%A_%a.err
#SBATCH --array=0-2   # 3 predictor configurations

cd "$SLURM_SUBMIT_DIR" || exit 1

export PIN_ROOT=/shared/courseSharedFolders/162593outer/162593/spack/opt/spack/linux-cascadelake/intel-pin-3.31-icrnczcau2lf67etmw5jlzavqlz2hdug

# =========================================================
# Predictors
# =========================================================
# Baseline:
#   Standard TAGE predictor
#
# Custom I:
#   512-entry bias table
#   strong bias if counter <=1 or >=6
#
# Custom II:
#   4096-entry bias table
#   strong bias if counter ==0 or ==7

PINTOOLS=(
    predictors/obj-intel64/part1-tage-predictor.so
    predictors/obj-intel64/part2-tage-custom-pcbias-predictor-v1.so
    predictors/obj-intel64/part2-tage-custom-pcbias-predictor-v2.so
)

PINTOOL_NAMES=(
    "tage-baseline"
    "tage-pcbias-customI"
    "tage-pcbias-customII"
)

# =========================================================
# Shared Default TAGE Parameters
# =========================================================

PINTOOL_ARGS=(
    "-L1 2 -R 2 -ntables 8 -nentries 4096 -pcbits 12 -tagbits 10 -prob 3"

    "-L1 2 -R 2 -ntables 8 -nentries 4096 -pcbits 12 -tagbits 10 -prob 3"

    "-L1 2 -R 2 -ntables 8 -nentries 4096 -pcbits 12 -tagbits 10 -prob 3"
)

# =========================================================
# Benchmark
# =========================================================

BENCHMARKS=(
    ./benchmarks/dealII_O3
)

# =========================================================
# SLURM Array Mapping
# =========================================================

NUM_BENCHMARKS=${#BENCHMARKS[@]}

PINTOOL_IDX=$(( SLURM_ARRAY_TASK_ID / NUM_BENCHMARKS ))
BENCHMARK_IDX=$(( SLURM_ARRAY_TASK_ID % NUM_BENCHMARKS ))

PINTOOL="${PINTOOLS[$PINTOOL_IDX]}"
PINTOOL_NAME="${PINTOOL_NAMES[$PINTOOL_IDX]}"
PINTOOL_ARGS_STR="${PINTOOL_ARGS[$PINTOOL_IDX]}"
BENCHMARK="${BENCHMARKS[$BENCHMARK_IDX]}"

# =========================================================
# Benchmark Arguments
# =========================================================

case "$BENCHMARK" in
    *dealII*)
        BENCHMARK_ARGS="23"
        BENCHMARK_NAME="dealII"
        ;;
esac

# =========================================================
# Output File
# =========================================================

OUTPUT_FILE="saved-results/part2/${PINTOOL_NAME}_${BENCHMARK_NAME}.out"

# =========================================================
# Run Info
# =========================================================

echo "=== Job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} ==="
echo "    Pintool:   $PINTOOL_NAME"
echo "    Binary:    $PINTOOL"
echo "    Args:      $PINTOOL_ARGS_STR"
echo "    Benchmark: $BENCHMARK_NAME ($BENCHMARK $BENCHMARK_ARGS)"
echo "    Output:    $OUTPUT_FILE"
echo "    Started:   $(date)"

mkdir -p saved-results/part2 logs

# =========================================================
# Execute
# =========================================================

$PIN_ROOT/pin \
    -t $PINTOOL \
    $PINTOOL_ARGS_STR \
    -o $OUTPUT_FILE \
    -- \
    $BENCHMARK $BENCHMARK_ARGS

echo "=== Done: $(date) ==="