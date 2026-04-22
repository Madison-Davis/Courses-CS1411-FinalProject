#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <map>
#include <cstdlib>
#include "pin.H"
#include <cmath> // for std::pow

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

/*
[How TAGE Works]
[Overview]
- N-component TAGE has 1 base predictor table and N tagged predictor tables
   - Assume each tagged predictor table has the same # of rows
- At prediction time, base predictor table and tagged component tables are accessed simultaneously
   - Base predictor offers a default prediction
   - Tagged component offers a prediction only on a tag match
       - Prediction = hit tagged predictor that uses longest history
       - If no tagged predictor hit, then use base predictor

[What is inside each predictor table]
- T0: base predictor, my guess is a 2-bit saturated counter
- T#: 3 values
    - pred (2-bit saturated counter)
    - tag (ignore the 'u' useful counter in the paper)
    - useful counter (2-bit saturated counter)... min is 0 and max is 3
       - when an entry is freshly allocated, u = 0

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

[How to update an entry]
- If correct:
   - PART 1: update current prediction counter
   - PART 2: update current u: increment if you chose T(i) and the prediction was correct, and T(i-#) was wrong
- If incorrect: update prediction counter
   - PART 1: update current u: decrement if you chose T(i) and the prediction was wrong, and T(i-#) was correct
   - PART 2: add a new entry and update all u's in the next longer history table, if applicable
       - If the table we were looking at is not the one with the longest history, try to allocate an entry on the next table that has a longer history
       - At most a single entry is allocated
       - If there exists an entry from k s.t u(k)=0 then allocate a new component there
       - If two tables Tj and Tk, j<k, can be allocated (i.e uj=uk=0) then choose Tj with higher probability
       - Else, decrement all counters in T(j) where i<j<M
*/


// PART 0: Variables
#define R           2       // geometric series common ratio: L(i) = (int) (R^(i-1) L(1) + 0.5)
#define L1          2       // geometric series starting val: L(i) = (int) (R^(i-1) L(1) + 0.5)
#define NTABLES     4       // # of predictor tables including the base predictor; by default you always have a base predictor
#define NENTRIES    1024    // # of entries in each table; should be divisible by log_2(#)
#define PC_BITS     10      // # of lower-bits to use for the PC hashing
#define TAG_BITS    8       // # of bits for the tag, 2nd hash func should ret # = to this
#define NHIST       60      // # of bits used for the global history register
#define PROB        3       // choose secondary with prob 1/PROB 


// PART 1: Base and Tagged Predictor Tables
struct base_table_entry
{
    int pred;               // 2-bit satured counter: [0..3]
};

struct base_table
{
    base_table_entry entries[NENTRIES];
};

struct tagged_table_entry
{
    int tag;
    int pred;               // 2-bit satured counter: [0..3]
    int u;                  // 2-bit satured counter: [0..3]
};

struct tagged_table
{
    tagged_table_entry entries[NENTRIES];
    int n_hist;
};


// PART 2: Globals
base_table t0;                              // base table, 1 of these
tagged_table tagged_tables[NTABLES - 1];    // tagged tables, NTABLES - 1 of these
UINT64 hist = 0;                            // global history, of size NHIST
static int  g_provider = -1;                // index for table providing the prediction: -1 = base, 0..NTABLES-2 = tagged table index
static bool g_prediction = false;           // prediction given by the table we chose
static bool g_altpred = false;              // prediction given by the next-lowest matching table, if applicable


// PART 3: Helper Functions
static inline UINT64 hist_mask(int n_hist)
// Compute a history mask based on n_hist, capping at 64-bits
{
    return (n_hist == 0) ? 0ULL
         : (n_hist >= 64) ? ~0ULL
         : (1ULL << n_hist) - 1;
}

static int compress_history(UINT64 h, int n_hist, int out_bits)
// Used to fold successive history bits into out_bits width
// Helpful for hash_index and hash_tag
// Ex: for n_hist of width out_bits, we do result = h[0:7] XOR h[8:15] XOR h[16:19]
{
    int result = 0;
    for (int shift = 0; shift < n_hist; shift += out_bits)
    {
        result ^= (int)((h >> shift) & ((1 << out_bits) - 1));
    }
    return result;
}

// Helpful functions to increment or decrement saturated counters (pred or u) 
static inline int sat_inc(int v, int max) { return (v < max) ? v + 1 : max; }
static inline int sat_dec(int v)          { return (v > 0)   ? v - 1 : 0;   }

static int hash_index(int pc, UINT64 h, int n_hist)
// Paper Section 2.4: Hash = PC + folded-history
// XOR successive PC_BITS-wide chunks of history into low PC bits
// Ex: to hash 30-bit hist to 10 bits, index = PC[0:9] XOR h[0:9] XOR h[10:19] XOR h[20:29]
{
    int pc_bits = pc & ((1 << PC_BITS) - 1);                // Ex: PC[0:9]
    int csr     = compress_history(h, n_hist, PC_BITS);     // Ex: h[0:9] XOR h[10:19] XOR h[20:29]
    return (pc_bits ^ csr) & ((1 << PC_BITS) - 1);
}

static int hash_tag(int pc, UINT64 h, int n_hist)
// Paper Section 2.4: Hash = PC + circular shift register (CSRs)
// Ex: suppose our hash is 8 bits.  Each tagged table uses CSR1 and CSR2 of width 8 and 8-1=7 bits.
    // index = PC[0:9] XOR CSR1 XOR (CSR2 << 1)
{
    int pc_bits = pc & ((1 << TAG_BITS) - 1);
    int csr1    = compress_history(h, n_hist, TAG_BITS);
    int csr2    = compress_history(h, n_hist, TAG_BITS - 1);
    return (pc_bits ^ csr1 ^ (csr2 << 1)) & ((1 << TAG_BITS) - 1);
}

// PART 4: Main Functions
void tage_init()
// Initialize TAGE
{
    // Base predictor table
    for (int k = 0; k < NENTRIES; ++k)
        t0.entries[k].pred = 0;

    // Tagged predictor tables
    for (int i = 0; i < NTABLES - 1; ++i)
    {
        // Geometric series
        // L(i) = (int)(R^(i-1) * L1 + 0.5) assumes i starts at 1
        // Therefore, since our i=0, L(i+1) = (int)(R^(i) * L1 + 0.5)
        // can omit the +0.5 because we are working strictly with int params
        tagged_tables[i].n_hist = (int)(std::pow((double)R, i) * L1);
        for (int k = 0; k < NENTRIES; ++k)
        {
            tagged_tables[i].entries[k].tag = 0;
            tagged_tables[i].entries[k].pred = 0;
            tagged_tables[i].entries[k].u = 0;
        }
    }
}

void tage_update(ADDRINT inst_ptr, bool taken)
// Update using TAGE
{
    int pc = (int)(inst_ptr & ((1 << PC_BITS) - 1));

    // STEP 1: Update the prediction counters (and the u counters) of the providing table
    if (g_provider == -1)
    {
        // Base table predictor
        // Update the prediction counter based on if right or wrong, capping at [0...3]
        int idx = pc & (NENTRIES - 1);
        t0.entries[idx].pred = taken ? sat_inc(t0.entries[idx].pred, 3) 
                                     : sat_dec(t0.entries[idx].pred);   
    }
    else
    {
        // Tagged table predictor
        // Update the prediction counter based on if right or wrong, capping at [0...3]
        int n_hist              = tagged_tables[g_provider].n_hist;
        int idx                 = hash_index(pc, hist & hist_mask(n_hist), n_hist);
        tagged_table_entry& e   = tagged_tables[g_provider].entries[idx];
        e.pred                  = taken ? sat_inc(e.pred, 3) : sat_dec(e.pred);
        
        // Update useful counter u
        // Increment when provider correct AND alt wrong
        // Decrement when provider wrong AND alt correct
        bool provider_correct   = (g_prediction == taken);
        bool alt_correct        = (g_altpred    == taken);
        if (provider_correct && !alt_correct) 
        {
            e.u = sat_inc(e.u, 3);
        } else if (!provider_correct && alt_correct)
        {
            e.u = sat_dec(e.u);
        }
    }

    // STEP 2: If the prediction is incorrect, then need to allocate a new entry
    // in a longer history table if there exists u = 0 in it,
    // ideally the shorter longest history table, if multiple.
    // Else, decrement u values in all tagged tables with longer history
    if (g_prediction != taken)
    {
        // Locate the first tagged table longer than current provider
        int start = (g_provider == -1) ? 0 : g_provider + 1;

        // Collect potential candidates for eviction across matching tagged table indices, where u == 0
        int  candidates[NTABLES];
        int  n_candidates = 0;
        for (int j = start; j < NTABLES - 1; ++j)
        {
            int n_hist   = tagged_tables[j].n_hist;
            int idx      = hash_index(pc, hist & hist_mask(n_hist), n_hist);
            if (tagged_tables[j].entries[idx].u == 0) candidates[n_candidates++] = j;
        }

        // If we have a candidate for eviction (exists u = 0)...
        if (n_candidates > 0)
        {
            // If we have several candidates for eviction, choose the table with shorter history
            // with higher probability (paper does not specify probability, let's do 2:1 odds)
            // i.e prefer lower index (shorter history) with ~(PROB-1):1 odds
            int chosen = candidates[0];
            if (n_candidates > 1 && (rand() % PROB == 0))
                chosen = candidates[1];

            // Find the index to overwrite in the chosen tagged table, then overwrite it
            int n_hist  = tagged_tables[chosen].n_hist;
            int idx     = hash_index(pc, hist & hist_mask(n_hist), n_hist);
            int tag     = hash_tag (pc, hist & hist_mask(n_hist), n_hist);

            tagged_tables[chosen].entries[idx].tag  = tag;
            tagged_tables[chosen].entries[idx].pred = taken ? 2 : 1; // start in weak state!
            tagged_tables[chosen].entries[idx].u    = 0;
        }
        // If we don't have a candidate for eviction (no u = 0)...
        else
        {
            // No free slot: decrement matching pc-hashed u in all longer tables
            for (int j = start; j < NTABLES - 1; ++j)
            {
                int n_hist  = tagged_tables[j].n_hist;
                int idx     = hash_index(pc, hist & hist_mask(n_hist), n_hist);
                tagged_tables[j].entries[idx].u = sat_dec(tagged_tables[j].entries[idx].u);
            }
        }
    }

    // STEP 3: Shift the global history based on recent branch's outcome
    hist = (hist << 1) | (taken ? 1ULL : 0ULL);
}

bool tage_predict(ADDRINT inst_ptr)
// Predict using TAGE
{
    int pc = (int)(inst_ptr & ((1 << PC_BITS) - 1));

    // Default: base predictor
    g_provider   = -1;
    g_prediction = (t0.entries[pc & (NENTRIES - 1)].pred >= 2);
    g_altpred    = g_prediction;
    bool found_provider = false;

    // Walk longest => shortest
    for (int i = NTABLES - 2; i >= 0; --i)
    {
        int n_hist   = tagged_tables[i].n_hist;
        int idx = hash_index(pc, hist & hist_mask(n_hist), n_hist);
        int tag = hash_tag  (pc, hist & hist_mask(n_hist), n_hist);

        // If we found a match...
        if (tagged_tables[i].entries[idx].tag == tag)
        {
            if (!found_provider)
            {
                // First (longest) hit: this is our provider!
                g_provider    = i;
                g_prediction  = (tagged_tables[i].entries[idx].pred >= 2);
                found_provider = true;
            }
            else
            {
                // Second hit: this is our altpred!
                g_altpred = (tagged_tables[i].entries[idx].pred >= 2);
                break; // have both provider and alt, all done!
            }
        }
    }
    return g_prediction;
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
    if (tage_predict(ins_ptr) == (bool) taken)
        CountCorrect++;

    // Update TAGE
    tage_update(ins_ptr, (bool)taken);

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
    tage_init();

    INS_AddInstrumentFunction(Instruction, 0);
    PIN_AddFiniFunction(Fini, 0);

    PIN_StartProgram();

    return 0;
}

/* ===================================================================== */
/* eof */
/* ===================================================================== */
