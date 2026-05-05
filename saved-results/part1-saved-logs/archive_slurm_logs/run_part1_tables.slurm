#!/bin/bash
#SBATCH --job-name=run-part1-tables
#SBATCH --partition=general
#SBATCH --time=5:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --output=run2-part1-%x_%A_%a.out
#SBATCH --error=run2-part1-%x_%A_%a.err
#SBATCH --array=0-15   # 4 pintools × 2 benchmarks × 2 table values = 16 jobs

cd "$SLURM_SUBMIT_DIR" || exit 1

export PIN_ROOT=/shared/courseSharedFolders/162593outer/162593/spack/opt/spack/linux-cascadelake/intel-pin-3.31-icrnczcau2lf67etmw5jlzavqlz2hdug

# -------------------------
# Tools and Benchmarks
# -------------------------
PINTOOLS=(
    predictors/obj-intel64/part1-tage-predictor.so
)

PINTOOL_NAMES=("tage")
PINTOOL_ARGS=("-s 1024" "-s 1024" "-r 512 -p 512" "")

BENCHMARKS=(
    ./benchmarks/dealII_O3
    ./benchmarks/libquantum_O3
)

# Sweep over ntables
TABLES=(12 16 32)

# -------------------------
# Index Mapping
# -------------------------
NUM_BENCHMARKS=${#BENCHMARKS[@]}   # 2
NUM_TABLES=${#TABLES[@]}           # 2

PINTOOL_IDX=$(( SLURM_ARRAY_TASK_ID / (NUM_BENCHMARKS * NUM_TABLES) ))
REMAINDER=$(( SLURM_ARRAY_TASK_ID % (NUM_BENCHMARKS * NUM_TABLES) ))

TABLE_IDX=$(( REMAINDER / NUM_BENCHMARKS ))
BENCHMARK_IDX=$(( REMAINDER % NUM_BENCHMARKS ))

PINTOOL="${PINTOOLS[$PINTOOL_IDX]}"
PINTOOL_NAME="${PINTOOL_NAMES[$PINTOOL_IDX]}"
PINTOOL_ARGS_STR="${PINTOOL_ARGS[$PINTOOL_IDX]}"
BENCHMARK="${BENCHMARKS[$BENCHMARK_IDX]}"
TABLE="${TABLES[$TABLE_IDX]}"

# -------------------------
# Benchmark-specific args
# -------------------------
case "$BENCHMARK" in
    *libquantum*) BENCHMARK_ARGS="400 25" ; BENCHMARK_NAME="quantum" ;;
    *dealII*)     BENCHMARK_ARGS="10"     ; BENCHMARK_NAME="dealII"  ;;
esac

# -------------------------
# Output naming
# -------------------------
OUTPUT_FILE="saved-results/part1_table/${PINTOOL_NAME}_t${TABLE}_${BENCHMARK_NAME}.out"

# -------------------------
# Logging
# -------------------------
echo "=== Job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} ==="
echo "    Pintool:   $PINTOOL_NAME ($PINTOOL)"
echo "    Args:      ${PINTOOL_ARGS_STR:-<none>}"
echo "    Tables:    $TABLE"
echo "    Benchmark: $BENCHMARK_NAME ($BENCHMARK $BENCHMARK_ARGS)"
echo "    Output:    $OUTPUT_FILE"
echo "    Started:   $(date)"

mkdir -p saved-results/part1_table logs

# -------------------------
# Run
# -------------------------
$PIN_ROOT/pin -t $PINTOOL -ntables $TABLE -o $OUTPUT_FILE -- $BENCHMARK $BENCHMARK_ARGS

echo "=== Done: $(date) ==="


    # $PIN_ROOT/pin -t   predictors/obj-intel64/part1-tage-predictor.so -ntables 16 -o "test.txt" --     ./benchmarks/dealII_O3


