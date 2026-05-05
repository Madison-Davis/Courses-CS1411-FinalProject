#!/bin/bash
#SBATCH --job-name=run-part1-prob-sweep
#SBATCH --partition=general
#SBATCH --time=6:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --output=run-part1-%x_%A_%a.out
#SBATCH --error=run-part1-%x_%A_%a.err
#SBATCH --array=0-2   # prob = 2,3,4

cd "$SLURM_SUBMIT_DIR" || exit 1

export PIN_ROOT=/shared/courseSharedFolders/162593outer/162593/spack/opt/spack/linux-cascadelake/intel-pin-3.31-icrnczcau2lf67etmw5jlzavqlz2hdug

# TAGE predictor only
PINTOOL=predictors/obj-intel64/part1-tage-predictor.so
PINTOOL_NAME="tage"

# Sweep prob knob
PROBS=(2 3 4)
PROB=${PROBS[$SLURM_ARRAY_TASK_ID]}

# Fixed TAGE args + swept prob
PINTOOL_ARGS="-L1 2 -R 2 -ntables 8 -nentries 4096 -pcbits 12 -tagbits 10 -prob $PROB"

# Benchmark
BENCHMARK=./benchmarks/hmmer_O3
BENCHMARK_ARGS="benchmarks/inputs/nph3.hmm benchmarks/inputs/swiss41"
BENCHMARK_NAME="hmmer"

# Output file
OUTPUT_FILE="saved-results/part1/${PINTOOL_NAME}_prob${PROB}_${BENCHMARK_NAME}.out"

echo "=== Job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} ==="
echo "    Pintool:   $PINTOOL_NAME"
echo "    Prob:      $PROB"
echo "    Args:      $PINTOOL_ARGS"
echo "    Benchmark: $BENCHMARK_NAME ($BENCHMARK $BENCHMARK_ARGS)"
echo "    Output:    $OUTPUT_FILE"
echo "    Started:   $(date)"

mkdir -p saved-results/part1 logs

$PIN_ROOT/pin \
    -t $PINTOOL \
    $PINTOOL_ARGS \
    -o $OUTPUT_FILE \
    -- \
    $BENCHMARK $BENCHMARK_ARGS

echo "=== Done: $(date) ==="