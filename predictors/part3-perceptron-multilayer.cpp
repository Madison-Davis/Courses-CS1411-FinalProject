#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <map>
#include <cstdlib>
#include "pin.H"
#include <cmath> // EXTENSION: for tanh and fabsf

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
KNOB<UINT32> KnobNumPerceptrons(KNOB_MODE_WRITEONCE, "pintool",
                            "n", "256", "number of perceptrons in table");

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
// EXTENSION: moved to floats to allow for gradient updates
const int HISTORY_LEN       = 32;                               // global history register (GHR) bits (paper states btw 12-62; 28 for 4KB budget, 59-62 for larger)
const int NUM_PERCEPTRONS   = 256;                              // size of perceptron table
const float THETA           = 1.93f * HISTORY_LEN + 14.0f;      // training threshold from paper (empirically derived, floor [1.93h + 14]
const float WEIGHT_MAX        = 127.0f;                         // 2^7; saturation bounds for weights, paper used 7-9 bit signed weights (7 for hist length 12, 9 for 62)
const float WEIGHT_MIN        = -127.0f;                        // 2^7; saturation bounds for weights, paper used 7-9 bit signed weights (7 for hist length 12, 9 for 62)
// EXTENSION
const int HIDDEN_SIZE       = 8;
const int INPUT_SIZE        = HISTORY_LEN + 1;                  // increase = more capacity, but more weights to saturate
const float LR              = 0.01f;                            // learning rate: increase = faster learning, but less stable

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
   - EXTENSION: multi-layer!
   - Each row contains a 2-layer perceptron
   - Initialized to all 0s
*/
struct MLPEntry {
    float W1[INPUT_SIZE][HIDDEN_SIZE]; // input -> hidden layer
    float W2[HIDDEN_SIZE];             // hidden -> output layer
};
std::vector<MLPEntry> perceptron_table;


/* ===================================================================== */
/* Perceptron Functions                                                  */
/* ===================================================================== */
/*  saturate
    add helper function to keep weights within bounds
*/
float saturate(float value)
{
    if(value > WEIGHT_MAX) return WEIGHT_MAX;
    if(value < WEIGHT_MIN) return WEIGHT_MIN;
    return value;
}

/*  perceptron_output
    // EXTENSION: perform a forward pass!
*/
float perceptron_output(ADDRINT pc, float h_out[HIDDEN_SIZE]) {
    // Modulo (with added entropy) indexing into perceptron table
    int num_perc = (int)KnobNumPerceptrons.Value();
    int index = (int)((pc ^ (pc >> 8)) % num_perc);

    MLPEntry& e = perceptron_table[index];
    // Hidden layer
    for (int j = 0; j < HIDDEN_SIZE; j++) {
        // Start with bias weight
        float z = e.W1[0][j];
        for (int i = 0; i < HISTORY_LEN; i++)
            z += GHR[i] * e.W1[i+1][j];
        h_out[j] = tanhf(z);
    }
    // Output layer
    float y = 0.0f;
    for (int j = 0; j < HIDDEN_SIZE; j++)
        y += h_out[j] * e.W2[j];
    return y;
}

/*  perceptron_prediction
    return true/false prediction
    // EXTENSION: base off of new perceptron_output!
*/  
bool perceptron_prediction(ADDRINT pc)
{
    float h[HIDDEN_SIZE];
    float y =  perceptron_output(pc, h);
    return (y >= 0.0f);
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
    int num_perc = (int)KnobNumPerceptrons.Value();
    int index = (int)((pc ^ (pc >> 8)) % num_perc);
    // EXTENSION: use 2-layer perceptron table and updated perceptron_output args!
    MLPEntry& e = perceptron_table[index];
    float t = taken ? 1.0f : -1.0f;         // Actual branch outcome as +1/-1
    float h[HIDDEN_SIZE];
    float y = perceptron_output(pc, h);     // Recompute perceptron output
    bool prediction = (y >= 0.0f);          // Current prediction based on output

    if((prediction != taken) || (fabsf(y) <= THETA))
    {
        // EXTENSION: 2-layer update!
        float d_out = y - t;
        for (int j = 0; j < HIDDEN_SIZE; j++) {
            // Update layer 2 weights
            float d_h = d_out * e.W2[j] * (1.0f - h[j]*h[j]); // tanh'
            e.W2[j] = saturate(e.W2[j] - LR * d_out * h[j]);
            // Update bias weight (note x0 is always 1)
            e.W1[0][j] = saturate(e.W1[0][j] - LR * d_h);
            // Update layer 1 weights
            for (int i = 0; i < HISTORY_LEN; i++)
                e.W1[i+1][j] = saturate(e.W1[i+1][j] - LR * d_h * GHR[i]);
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
    int num_perc = (int)KnobNumPerceptrons.Value();

    GHR.resize(HISTORY_LEN);

    for(int i = 0; i < HISTORY_LEN; i++)
    {
        GHR[i] = -1;
    }

    perceptron_table.resize(num_perc);

    // EXTENSION: 2 layer weights!
    for (int i = 0; i < num_perc; i++) 
    {
        // Fill in the layer 1 weights
        for (int a = 0; a < INPUT_SIZE; a++)
            for (int b = 0; b < HIDDEN_SIZE; b++)
                perceptron_table[i].W1[a][b] = 0.0f;
        // Fill in the layer 2 weights
        for (int b = 0; b < HIDDEN_SIZE; b++)
            perceptron_table[i].W2[b] = 0.0f;
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