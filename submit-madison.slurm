#!/bin/bash
#SBATCH --job-name=run-part3
#SBATCH --partition=general
#SBATCH --time=6:00:00
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --output=run-part1-sweep-num_tables__%A_%a.out
#SBATCH --error=run-part1-sweep-num_tables_%A_%a.err
#SBATCH --array=0-1

cd "$SLURM_SUBMIT_DIR" || exit 1
export PIN_ROOT=/shared/courseSharedFolders/162593outer/162593/spack/opt/spack/linux-cascadelake/intel-pin-3.31-icrnczcau2lf67etmw5jlzavqlz2hdug

N_TABLES=(12)
BENCHMARKS=(./benchmarks/hmmer_O3)

NUM_BENCHMARKS=${#BENCHMARKS[@]}
SIZE_IDX=$(( SLURM_ARRAY_TASK_ID / NUM_BENCHMARKS ))
BENCHMARK_IDX=$(( SLURM_ARRAY_TASK_ID % NUM_BENCHMARKS ))

N_TABLE=${N_TABLES[$SIZE_IDX]}
BENCHMARK=${BENCHMARKS[$BENCHMARK_IDX]}

case "$BENCHMARK" in
    *hmmer*) BENCHMARK_ARGS="benchmarks/inputs/nph3.hmm benchmarks/inputs/swiss41" ; BENCHMARK_NAME="hmmer" ;;
esac

PINTOOL=predictors/obj-intel64/part1-tage-predictor.so
PINTOOL_NAME="tage"
PINTOOL_ARGS="-L1 4 -R 2 -ntables $N_TABLE -nentries 1024 -pcbits 10 -tagbits 8 -prob 2"

OUTPUT_FILE="saved-results/part1/tage_sweep_numtables_${N_TABLE}_${BENCHMARK_NAME}.out"

echo "=== Job ${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID} ==="
echo "    Pintool:         $PINTOOL_NAME  ($PINTOOL)"
echo "    Args:            $PINTOOL_ARGS"
echo "    N_TABLE:         $N_TABLE"
echo "    Benchmark:       $BENCHMARK_NAME  ($BENCHMARK $BENCHMARK_ARGS)"
echo "    Output:          $OUTPUT_FILE"
echo "    Started:         $(date)"

mkdir -p saved-results/part1 logs

$PIN_ROOT/pin -t $PINTOOL $PINTOOL_ARGS -o $OUTPUT_FILE -- $BENCHMARK $BENCHMARK_ARGS

echo "=== Done: $(date) ==="
