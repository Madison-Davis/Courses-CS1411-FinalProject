# Courses-CS1411-FinalProject

## Logistics
- Copy this repo into the CS 1411 cluster
- Edit files as necessary
- Push often back to GitHub

## Deadlines
- Friday April 10, I will show them the trace files and point them in the right direction
- By Tuesday April 14, submit your team of up to 4
- By Friday April 24, have (stride prefetcher) or (tage branch predictor) implemented and validated (beats cache miss rate/branch prediction accuracy on libquantum and dealii programs from PA2/3) + propose your scheme
- By May 5, implement one paper’s prefetcher/predictor and something on your own
- Make sure you show tradeoff between hardware cost (the number of bits) and performance (cache miss rate/prediction accuracy improvement, end to end speedup)


## Grading 
- Minimum - implement something that has already been done in a paper (and show how the cache miss rate/prediction accuracy improves)
- Better - sweep parameters to show design space
- Best - make a novel novel change to an existing paper to show improvement

## File Setup
- benchmarks: libquanutm and dealii executables
- predictors: part 1 (tage), part 2 (paper), and part 3 (custom) predictors and executables, along with makefiles
- saved-results: for any results we want to save for reference after a slurm script submission, these will be found here
- submit.slurm: what we use to submit jobs to the slurm cluster
