#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <map>
#include <cstdlib>
#include "pin.H"

using std::cerr;
using std::cout;
using std::endl;
using std::ofstream;
using std::string;

/* ===================================================================== */
/* Commandline Switches */
/* ===================================================================== */
KNOB<string> KnobOutputFile(KNOB_MODE_WRITEONCE, "pintool",
                            "o", "hw3.out", "specify output file name");
KNOB<BOOL> KnobPid(KNOB_MODE_WRITEONCE, "pintool",
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
/* TODO: TAGE Predictor                                                  */
/* ===================================================================== */
// TODO!

/*
How TAGE works:

[Overview]
- N-component TAGE has 1 base predictor table and N tagged predictor tables
- At prediction time, base predictor table and tagged component tables are accessed simultaneously
    - Base predictor = default prediction
    - Tagged component provides a prediction only on a tag match
    - Predciction = hit tagged predictor that uses longest history
    - If no tagged predictor hit, then use base predictor

[Assumptions]
- Each tagged predictor table has the same # of rows

[The PC and the history are used for two separate purposes]:
- 1: index into the tagged predictor table.  If your hashed PC + history is N bits, then your table has 2^N rows.
- 2: compare the tag inside the table.  If your tag is M bits, then you want the hashed PC + history to become an M-bit number.

[How to index into the tagged predictor tables]
- Index using both the branch PC and some global history bits
- Ex:
    - Suppose we have 4 tagged predictor tables and are indexed respectively with the 10, 20, 40, and 80 most recent bits in the 80-bit global history, as indicated on Figure 1.
    - Table 3 is indexed with 40 history bits.  The index = pc[0 : 9] ⊕ h[0 : 9] ⊕ h[10 : 19] ⊕ h[20 : 29] ⊕ h[30 : 39] where ⊕ denotes the bit-wise XOR
    - History lengths should follow L(i) = (int) (R^(i-1) L(1) + 0.5)
    - Ex: R=2, L(1)=2, history={0,2,4,8,16,32,64}

[What is inside each predictor table]
- T0: base predictor, my guess is a 2-bit saturated counter
- T#: pred (2-bit saturated counter), tag (ignore the 'u' useful counter in the paper)


[How to update an entry]
-

*/

// Variables
#define R 2           // geometric series  common ratio: L(i) = (int) (R^(i-1) L(1) + 0.5)
#define L1 2          // geometric series starting val: L(i) = (int) (R^(i-1) L(1) + 0.5)
#define NTABLES 4     // # of predictor tables including the base predictor; by default you always have a base predictor
#define NENTRIES 1024 // # of entries in each table; should be divisible by log_2(#)
#define PC_BITS 10    // # of lower-bits to use for the PC hashing
#define TAG_BITS 8    // # of bits for the tag, 2nd hash func should ret # = to this

struct table_entry
{
    int tag;
    int two_bit_counter; // cap this 0-3
    int u;
};

struct table
{
    table_entry entries[NENTIRES];
    int n_hist;
};

table tables[NTABLES];
UINT64 hist = 0;
UINT64 mask = (1U << PC_BITS) - 1;
// PHT table initialization
void tage_init()
{
    for (int i = 0; i < NTABLES; ++i)
    {
        table curr = tables[i];
        curr.n_hist = (i == 0) ? 0 : std::power(R, i);
        // TODO: base predictor doesn't need all this stuff
        for (int k = 0; k < NENTIRES; ++k)
        {
            curr.entires[k].two_bit_counter = 0;
            curr.entries[k].u = 0;
            curr.entries[k].tag = 0;
        }
    }
}

void tage_update(ADDRINT inst_ptr, bool taken)
{
    // need to update the useful counter
    // it the prediction is incorrect, then need to allocate new entry
    // in a longer history table if u is 0, then decrement u values in all tables with longer history
}

bool tage_predict(ADDINRT inst_ptr)
{
    int pc = instr_ptr & mask;
    for (int i = NTABLES - 1; i >= 0; --i)
    {
        table curr_table = tables[i];
        int hist_mask = (1UL << n_hist) - 1;
        int index = hash_index(pc, hist & hist_mask);
        int tag = hash_tag(pc, hist & hist_mask);

        table_entry curr_entry = curr_table.entries[index];
        if (curr_entry.tag == tag || i == 0)
        {
            return (curr_entry.two_bit_counter >= 2);
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
    if (KnobPid)
        output_file += "." + decstr(getpid());

    std::ofstream out(output_file.c_str());

    if (limit_reached)
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
    if (taken)
    {
        CountTaken++;
    }

    // Count the correctly predicted branches
    if (BPB_prediction(ins_ptr) == taken)
        CountCorrect++;

    // Update branch prediction buffer
    BPB_update(ins_ptr, taken);
    if (CountSeen == KnobBranchLimit.Value())
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
    if (INS_IsRet(ins))
    {
        INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)br_predict,
                       IARG_INST_PTR, IARG_BRANCH_TAKEN, IARG_END);
    }
    else if (INS_IsSyscall(ins))
    {
        INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)br_predict,
                       IARG_INST_PTR, IARG_BRANCH_TAKEN, IARG_END);
    }
    else if (INS_IsBranch(ins))
    {
        if (INS_IsCall(ins))
        {
            INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)br_predict,
                           IARG_INST_PTR, IARG_BRANCH_TAKEN, IARG_END);
        }
        else
        {
            INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)br_predict,
                           IARG_INST_PTR, IARG_BRANCH_TAKEN, IARG_END);
        }
    }
}

/* ===================================================================== */
/* Finish Functions                                                      */
/* ===================================================================== */
#define OUT(n, a, b) out << n << " " << a << setw(16) << CountSeen.b << " " << setw(16) << CountTaken.b << endl

VOID Fini(int n, void *v)
{
    write_results(false);
}

/* ===================================================================== */
/* Main Function                                                         */
/* ===================================================================== */
int main(int argc, char *argv[])
{

    if (PIN_Init(argc, argv))
    {
        return Usage();
    }

    // TODO: initiate TAGE predictor!

    INS_AddInstrumentFunction(Instruction, 0);
    PIN_AddFiniFunction(Fini, 0);

    PIN_StartProgram();

    return 0;
}

/* ===================================================================== */
/* eof */
/* ===================================================================== */
