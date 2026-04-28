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
/* Perceptron Branch Predictor                                           */
/* Based on: "Dynamic Branch Prediction with Perceptrons"                */
/* ===================================================================== */

// ============================ PARAMETERS ==============================
const int HISTORY_LEN = 32;                       // GHR bits (28 for 4KB budget, 59-62 for larger)
const int NUM_PERCEPTRONS = 256;                  // Size of perceptron table
const int THETA = (int)(1.93 * HISTORY_LEN + 14); // Training threshold from paper (empirically derived)

// Saturation bounds for weights, paper used 7-9 bit signed weights
const int WEIGHT_MAX = 127;
const int WEIGHT_MIN = -127;

/*
============================ DATA STRUCTURES ============================
1. Global History Register (GHR): outcomes of the HISTORY_LEN most recent branches
   - +1 = taken
   - -1 = not taken
   - Initialized to "not taken" (-1)
*/

std::vector<int> GHR(HISTORY_LEN, -1);

/*
2. Perceptron Table
   - NUM_PERCEPTRONS rows
   - Indexed by branch PC
   - Each perceptron contains:
        weights[0] = bias weight
        weights[1..HISTORY_LEN] = history correlation weights
   - Initialized to all 0s
*/

std::vector<std::vector<int>> perceptron_table(
    NUM_PERCEPTRONS,
    std::vector<int>(HISTORY_LEN + 1, 0)
);

// Saturating add helper function to keep weights within bounds
int saturate(int value)
{
    if(value > WEIGHT_MAX) return WEIGHT_MAX;
    if(value < WEIGHT_MIN) return WEIGHT_MIN;
    return value;
}

/*
============================ PREDICTION RULE ============================
3. Dot-product prediction
    y = weights[0] + SUM(GHR[i] * weights[i])
    if y >= 0 => predict TAKEN
    else      => predict NOT TAKEN
*/

int perceptron_output(ADDRINT pc)
{
    int index = pc % NUM_PERCEPTRONS; // Modulo indexing into perceptron table
    std::vector<int>& weights = perceptron_table[index];
    int y = weights[0]; // Start with bias weight

    // Add all correlation terms
    for(int i = 0; i < HISTORY_LEN; i++)
    {
        y += weights[i + 1] * GHR[i];
    }

    return y;
}

bool perceptron_prediction(ADDRINT pc)
{
    int y = perceptron_output(pc);
    return (y >= 0);
}

/*
============================== UPDATE RULE ==============================
4. Perceptron learning rule
   - Train if: prediction was wrong OR confidence was low (|y| <= THETA)
   - Perceptron update rule: weight[i] = weight[i] + t * x[i]
     where:
        t  = actual branch outcome (+1 or -1)
        x[0] = 1 (bias input)
        x[i] = GHR[i-1] for i > 0 (history bits)
5. Always update GHR with actual outcome after training
*/

void perceptron_update(ADDRINT pc, bool taken)
{
    int index = pc % NUM_PERCEPTRONS;
    std::vector<int>& weights = perceptron_table[index];
    int t = taken ? 1 : -1;        // Actual branch outcome as +1/-1
    int y = perceptron_output(pc); // Recompute perceptron output
    bool prediction = (y >= 0);    // Current prediction based on output

    if((prediction != taken) || (abs(y) <= THETA))
    {
        weights[0] = saturate(weights[0] + t); // Update bias weight (note x0 is always 1)

        // Update history correlation weights
        for(int i = 0; i < HISTORY_LEN; i++)
        {
            weights[i + 1] =
                saturate(weights[i + 1] + (t * GHR[i]));
        }
    }

    // Update GHR: shift right and insert new outcome at front
    for(int i = HISTORY_LEN - 1; i > 0; i--)
    {
        GHR[i] = GHR[i - 1];
    }
    GHR[0] = t;
}

/*
============================= INITIALIZATION =============================

Initialize perceptron predictor structures.

1. Set predictor parameters
2. Initialize GHR to all NOT TAKEN (-1)
3. Initialize perceptron table
    - weights[0] = bias weight
    - weights[1..HISTORY_LEN] = history correlation weights
4. Initialize all weights to 0

Cold-start behavior matches the paper.
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