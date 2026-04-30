#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <map>
#include <cstdlib>
#include "pin.H"

using std::ofstream;
using std::cout;
using std::cerr;
using std::endl;
using std::string;


/* ===================================================================== */
/* Commandline Switches */
/* ===================================================================== */
KNOB<string> KnobOutputFile(KNOB_MODE_WRITEONCE, "pintool",
                            "o", "hw3.out", "specify output file name");
KNOB<BOOL>   KnobPid(KNOB_MODE_WRITEONCE, "pintool",
                            "i", "0", "append pid to output");
KNOB<UINT64> KnobBranchLimit(KNOB_MODE_WRITEONCE, "pintool",
                            "l", "0", "set limit of branches analyzed");


/* ===================================================================== */
/* Global Variables */
/* ===================================================================== */
UINT64 CountSeen = 0;
UINT64 CountTaken = 0;
UINT64 CountCorrect = 0;


/* ===================================================================== */
/* WRITTEN: Perceptron Branch Predictor                                  */
/* Based on: "Dynamic Branch Prediction with Perceptrons"                */
/* ===================================================================== */

/*
[How Perceptron Works]
[Overview]
You have a PC, a global history register (GHR) of n bits, and a table of N rows.
- Each row in the table holds a vector of weights (w1...wn) representing a perceptron.
- These weights are learned through training
- A large weight means there is correlation
- A weight close to 0 means there is little correlation
- A positive weight indicates we predict taken, negative not taken

[How a prediction and update step works]
- First, take the PC and hash it to give you an index into the table.
- Select the row you indexed into: this gives you a perceptron, represented by its weights w1...wn
- Compute the dot product of these weights with the bits of the global history register to get a number.
- If the number > 0 predict taken, otherwise not taken.
- Compare the prediction with the actual.
- Run the training algorithm given the results (did we get it right, and was our dot product result)

[What does the training step look like]
- basically, only do the training if our prediction (taken/not taken) does not match what actually happened
- OR if our prediction is too weak/not confident enough (below some threshold theta)
if sign(y_out) != t or |y_out| <= theta then
    for i in range(0, n) do
        wi = wi + t xi
    end for
end if


[Assumptions]
- The paper does not state what to initialize the weights to, we start at 0 (the midpoint between our min and mix values)
- The paper does not mention what to initialize the GHR to, but a nice conservative standard is to set all bits to "not taken," i.e -1
- The paper does not specify what is meant by hashing the index.  A naive approach would be to do PC % NUM_PERCEPTRONS, but
  if branches are aligned by certain byte boundaries, then this could cause aliasing.  To give more robustness, we do
  a folding mechanism ((pc ^ (pc >> 8)) % NUM_PERCEPTRONS so that two instructions with the same bottom bits but different top
  bits will hash to different indices. 

*/

/* ===================================================================== */
/* Runtime-Configurable Perceptron Parameters                            */
/* ===================================================================== */
const int HISTORY_LEN       = 32;                               // global history register (GHR) bits (paper states btw 12-62; 28 for 4KB budget, 59-62 for larger)
const int NUM_PERCEPTRONS   = 256;                              // size of perceptron table
const int THETA             = (int)(1.93 * HISTORY_LEN + 14);   // training threshold from paper (empirically derived, floor [1.93h + 14]
const int WEIGHT_MAX        = 127;                              // 2^7; saturation bounds for weights, paper used 7-9 bit signed weights (7 for hist length 12, 9 for 62)
const int WEIGHT_MIN        = -127;                             // 2^7; saturation bounds for weights, paper used 7-9 bit signed weights (7 for hist length 12, 9 for 62)


/* ===================================================================== */
/* Table Structures                                                      */
/* ===================================================================== */
/* 1. Global History Register (GHR): outcomes of the HISTORY_LEN most recent branches
    - +1 = taken
    - -1 = not taken
    - Initialized to "not taken" (-1)
*/
std::vector<int> GHR(HISTORY_LEN, -1);

/* 2. Perceptron Table
   - NUM_PERCEPTRONS rows
   - Indexed by branch PC
   - Each perceptron contains:
        weights[0] = bias weight, always 1 (from the paper)
        weights[1...HISTORY_LEN] = history correlation weights
   - Initialized to all 0s
*/
std::vector<std::vector<int>> perceptron_table(
    NUM_PERCEPTRONS,
    std::vector<int>(HISTORY_LEN + 1, 0)
);


/* ===================================================================== */
/* Perceptron Functions                                                  */
/* ===================================================================== */
/*  saturate
    add helper function to keep weights within bounds
*/
int saturate(int value)
{
    if(value > WEIGHT_MAX) return WEIGHT_MAX;
    if(value < WEIGHT_MIN) return WEIGHT_MIN;
    return value;
}

/*  perceptron_output
    Perform dot-product rule for prediction
    y  = weights[0] + SUM(GHR[i] * weights[i])
    if y >= 0 => predict TAKEN
    else      => predict NOT TAKEN
*/
int perceptron_output(ADDRINT pc)
{
    // Modulo (with added entropy) indexing into perceptron table
    int index = (int)((pc ^ (pc >> 8)) % NUM_PERCEPTRONS);
    std::vector<int>& weights = perceptron_table[index];
    // Start with bias weight
    int y = weights[0]; 
    // Add all correlation terms
    for(int i = 0; i < HISTORY_LEN; ++i)
    {
        y += weights[i + 1] * GHR[i];
    }
    return y;
}

/*  perceptron_prediction
    return true/false prediction
*/  
bool perceptron_prediction(ADDRINT pc)
{
    int y = perceptron_output(pc);
    return (y >= 0);
}

/* perceptron_update
   - Perceptron learning rule
   - Train if: prediction was wrong OR confidence was low (|y| <= THETA)
   - Perceptron update rule: weight[i] = weight[i] + t * x[i]
     where:
        t  = actual branch outcome (+1 or -1)
        x[0] = 1 (bias input)
        x[i] = GHR[i-1] for i > 0 (history bits)
   - always update GHR with actual outcome after training
*/
void perceptron_update(ADDRINT pc, bool taken)
{
    int index = (int)((pc ^ (pc >> 8)) % NUM_PERCEPTRONS);
    std::vector<int>& weights = perceptron_table[index];
    int t = taken ? 1 : -1;        // Actual branch outcome as +1/-1
    int y = perceptron_output(pc); // Recompute perceptron output
    bool prediction = (y >= 0);    // Current prediction based on output

    if((prediction != taken) || (abs(y) <= THETA))
    {
        weights[0] = saturate(weights[0] + t); // Update bias weight (note x0 is always 1)

        // Update history correlation weights
        for(int i = 0; i < HISTORY_LEN; ++i)
        {
            weights[i + 1] = saturate(weights[i + 1] + (t * GHR[i]));
        }
    }

    // Update GHR: shift right and insert new outcome at front
    for(int i = HISTORY_LEN - 1; i > 0; --i)
    {
        GHR[i] = GHR[i - 1];
    }
    GHR[0] = t;
}

/*  perceptron_init
    - initialize perceptron predictor structures.
    - Set predictor parameters
    - Initialize GHR to all NOT TAKEN (-1)
    - Initialize perceptron table
        - weights[0] = bias weight
        - weights[1..HISTORY_LEN] = history correlation weights
    - Initialize all weights to 0
    - Cold-start behavior matches the paper
*/
void perceptron_init()
{
    GHR.resize(HISTORY_LEN);

    for(int i = 0; i < HISTORY_LEN; i++)
    {
        GHR[i] = -1;
    }

    perceptron_table.resize(NUM_PERCEPTRONS);

    for(int i = 0; i < NUM_PERCEPTRONS; i++)
    {
        perceptron_table[i].resize(HISTORY_LEN + 1);

        for(int j = 0; j <= HISTORY_LEN; j++)
        {
            perceptron_table[i][j] = 0;
        }
    }
}


/* ===================================================================== */
/* Helper Functions                                                      */
/* ===================================================================== */
static INT32 Usage()
{
    cerr << "This pin tool collects a profile of jump/branch/call instructions for an application\n";

    cerr << KNOB_BASE::StringKnobSummary();

    cerr << endl;
    return -1;
}

VOID write_results(bool limit_reached)
{
    string output_file = KnobOutputFile.Value();
    if(KnobPid) output_file += "." + decstr(getpid());

    std::ofstream out(output_file.c_str());

    if(limit_reached)
        out << "Reason: limit reached\n";
    else
        out << "Reason: fini\n";
    out << "Count Seen: " << CountSeen << endl;
    out << "Count Taken: " << CountTaken << endl;
    out << "Count Correct: " << CountCorrect << endl;
    out.close();
}


/* ===================================================================== */
/* Instrumentation Code and Analysis Code                                */
/* ===================================================================== */
VOID br_predict(ADDRINT ins_ptr, INT32 taken)
{
    // Count the number of branches seen
	CountSeen++;
	// Count the take branches
	if (taken){
			CountTaken++;
	}

	// Count the correctly predicted branches
	if(perceptron_prediction(ins_ptr) == taken)
		CountCorrect++;

	// Update branch prediction buffer
	perceptron_update(ins_ptr, taken);
    if(CountSeen == KnobBranchLimit.Value())
    {
        write_results(true);
        exit(0);
    }
}

VOID Instruction(INS ins, void *v)
{
    // The subcases of direct branch and indirect branch are
    // broken into "call" or "not call".  Call is for a subroutine
    // These are left as subcases in case the programmer wants
    // to extend the statistics to see how sub cases of branches behave
    if( INS_IsRet(ins) )
    {
        INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR) br_predict,
            IARG_INST_PTR, IARG_BRANCH_TAKEN,  IARG_END);
    }
    else if( INS_IsSyscall(ins) )
    {
        INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR) br_predict,
            IARG_INST_PTR, IARG_BRANCH_TAKEN,  IARG_END);
    }
    else if (INS_IsBranch(ins))
    {
        if( INS_IsCall(ins) ) {
            INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR) br_predict,
                IARG_INST_PTR, IARG_BRANCH_TAKEN,  IARG_END);
        }
        else {
            INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR) br_predict,
                IARG_INST_PTR, IARG_BRANCH_TAKEN,  IARG_END);
        }
    }

}


/* ===================================================================== */
/* Finish Functions                                                      */
/* ===================================================================== */
#define OUT(n, a, b) out << n << " " << a << setw(16) << CountSeen. b  << " " << setw(16) << CountTaken. b << endl

VOID Fini(int n, void *v)
{
    write_results(false);
}


/* ===================================================================== */
/* Main Function                                                         */
/* ===================================================================== */
int main(int argc, char *argv[])
{

    if( PIN_Init(argc,argv) )
    {
        return Usage();
    }

    perceptron_init();

    INS_AddInstrumentFunction(Instruction, 0);
    PIN_AddFiniFunction(Fini, 0);

    PIN_StartProgram();

    return 0;
}


/* ===================================================================== */
/* eof */
/* ===================================================================== */