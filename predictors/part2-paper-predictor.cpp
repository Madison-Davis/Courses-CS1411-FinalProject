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
/* TODO: Paper Predictor                                                 */
/* ===================================================================== */
// TODO!


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
	if(BPB_prediction(ins_ptr) == taken) 
		CountCorrect++;

	// Update branch prediction buffer
	BPB_update(ins_ptr, taken);
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

    // TODO: initiate paper predictor!
        
    INS_AddInstrumentFunction(Instruction, 0);
    PIN_AddFiniFunction(Fini, 0);

    PIN_StartProgram();
    
    return 0;
}


/* ===================================================================== */
/* eof */
/* ===================================================================== */
