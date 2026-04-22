#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <map>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include "pin.H"
#include <cmath>

using std::cerr;
using std::cout;
using std::endl;
using std::ofstream;
using std::string;

/* ===================================================================== */
/* Commandline Switches                                                  */
/* ===================================================================== */
KNOB<string> KnobOutputFile(KNOB_MODE_WRITEONCE, "pintool",
                            "o", "hw3.out", "specify output file name");
KNOB<BOOL> KnobPid(KNOB_MODE_WRITEONCE, "pintool",
                   "i", "0", "append pid to output");
KNOB<UINT64> KnobBranchLimit(KNOB_MODE_WRITEONCE, "pintool",
                             "l", "0", "set limit of branches analyzed");

KNOB<UINT32> KnobL1(KNOB_MODE_WRITEONCE, "pintool",
                    "L1", "4", "starting history length L1 for TAGE");
KNOB<UINT32> KnobR(KNOB_MODE_WRITEONCE, "pintool",
                   "R", "2", "geometric ratio R for TAGE history lengths");
KNOB<UINT32> KnobNTables(KNOB_MODE_WRITEONCE, "pintool",
                         "ntables", "6", "number of predictor tables including the base predictor");
KNOB<UINT32> KnobNEntries(KNOB_MODE_WRITEONCE, "pintool",
                          "nentries", "4096", "number of entries per predictor table (must be power of 2)");
KNOB<UINT32> KnobPCBits(KNOB_MODE_WRITEONCE, "pintool",
                        "pcbits", "12", "number of low PC bits used in index hash");
KNOB<UINT32> KnobTagBits(KNOB_MODE_WRITEONCE, "pintool",
                         "tagbits", "10", "number of bits used for tags");
KNOB<UINT32> KnobProb(KNOB_MODE_WRITEONCE, "pintool",
                      "prob", "3", "prefer shortest allocatable table with probability (prob-1)/prob");

/* ===================================================================== */
/* Global Variables                                                      */
/* ===================================================================== */
UINT64 CountSeen = 0;
UINT64 CountTaken = 0;
UINT64 CountCorrect = 0;

/* ===================================================================== */
/* TAGE Predictor                                                        */
/* ===================================================================== */

static const int NHIST = 128;

static UINT32 g_l1 = 4;
static UINT32 g_r = 2;
static UINT32 g_ntables = 6;
static UINT32 g_nentries = 4096;
static UINT32 g_pc_bits = 12;
static UINT32 g_tag_bits = 10;
static UINT32 g_prob = 3;

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
- T0: base predictor, 2-bit saturated counter
- T#: 3 values
    - pred (2-bit saturated counter)
    - tag
    - useful counter u (2-bit saturated counter)
       - when an entry is freshly allocated, u = 0

[The PC and the history are used for two separate purposes]:
- 1: index into the tagged predictor table
- 2: compare the tag stored in the table

[How to index into the tagged predictor tables]
- Index using both the branch PC and some global history bits
- History lengths follow L(i) = (int)(R^(i-1) * L(1) + 0.5)

[How to update an entry]
- If correct:
   - update prediction counter
   - update u: increment if provider correct and alternate wrong
- If incorrect:
   - update prediction counter
   - update u: decrement if provider wrong and alternate correct
   - try to allocate a new entry in a longer-history table
*/

// PART 1: Base and Tagged Predictor Tables
struct base_table_entry
{
    int pred; // 2-bit saturated counter: [0..3]
};

struct base_table
{
    std::vector<base_table_entry> entries;
};

struct tagged_table_entry
{
    int tag;
    int pred; // 2-bit saturated counter: [0..3]
    int u;    // 2-bit saturated counter: [0..3]
};

struct tagged_table
{
    std::vector<tagged_table_entry> entries;
    int n_hist;
};

// PART 2: Globals
base_table t0;
std::vector<tagged_table> tagged_tables;
__uint128_t hist = 0;

static int g_provider = -1; // -1 = base, otherwise index into tagged_tables
static bool g_prediction = false;
static bool g_altpred = false;

// PART 3: Helper Functions
static inline __uint128_t hist_mask(int n_hist)
// Compute a history mask based on n_hist, capping at 128 bits.
{
    if (n_hist <= 0)
        return (__uint128_t)0;
    if (n_hist >= 128)
        return ~(__uint128_t)0;
    return ((__uint128_t)1 << n_hist) - 1;
}

static int compress_history(__uint128_t h, int n_hist, int out_bits)
// Fold successive history chunks into out_bits width.
{
    int result = 0;
    int effective = std::min(n_hist, NHIST);

    for (int shift = 0; shift < effective; shift += out_bits)
    {
        result ^= (int)((h >> shift) & (((__uint128_t)1 << out_bits) - 1));
    }
    return result;
}

static inline int sat_inc(int v, int maxv) { return (v < maxv) ? v + 1 : maxv; }
static inline int sat_dec(int v) { return (v > 0) ? v - 1 : 0; }

static inline int table_index_mask()
// Since nentries must be a power of two, use masking for fast modulo.
{
    return (int)g_nentries - 1;
}

static int hash_index(int pc, __uint128_t h, int n_hist)
// Hash PC and folded history for table indexing.
{
    int pc_bits = pc & ((1 << g_pc_bits) - 1);
    int csr = compress_history(h, n_hist, g_pc_bits);
    return (pc_bits ^ csr);
}

static int hash_tag(int pc, __uint128_t h, int n_hist)
// Hash PC and folded history for tag comparison.
{
    int pc_bits = pc & ((1 << g_tag_bits) - 1);
    int csr1 = compress_history(h, n_hist, g_tag_bits);
    int csr2 = compress_history(h, n_hist, g_tag_bits - 1);
    return (pc_bits ^ csr1 ^ (csr2 << 1)) & ((1 << g_tag_bits) - 1);
}

static void validate_config()
{
    if (g_ntables < 2)
    {
        cerr << "Error: ntables must be at least 2 (base + one tagged table)\n";
        exit(1);
    }
    if (g_nentries == 0 || (g_nentries & (g_nentries - 1)) != 0)
    {
        cerr << "Error: nentries must be a nonzero power of 2\n";
        exit(1);
    }
    if (g_pc_bits == 0 || g_pc_bits > 30)
    {
        cerr << "Error: pcbits must be between 1 and 30\n";
        exit(1);
    }
    if (g_tag_bits < 2 || g_tag_bits > 30)
    {
        cerr << "Error: tagbits must be between 2 and 30\n";
        exit(1);
    }
    if (g_prob == 0)
    {
        cerr << "Error: prob must be >= 1\n";
        exit(1);
    }
    if (g_l1 == 0)
    {
        cerr << "Error: L1 must be >= 1\n";
        exit(1);
    }
    if (g_r == 0)
    {
        cerr << "Error: R must be >= 1\n";
        exit(1);
    }
}

// PART 4: Main Functions
void tage_init()
// Initialize TAGE.
{
    g_l1 = KnobL1.Value();
    g_r = KnobR.Value();
    g_ntables = KnobNTables.Value();
    g_nentries = KnobNEntries.Value();
    g_pc_bits = KnobPCBits.Value();
    g_tag_bits = KnobTagBits.Value();
    g_prob = KnobProb.Value();

    validate_config();

    hist = 0;
    g_provider = -1;
    g_prediction = false;
    g_altpred = false;

    // Base predictor table
    t0.entries.assign(g_nentries, base_table_entry{0});

    // Tagged predictor tables
    tagged_tables.clear();
    tagged_tables.resize(g_ntables - 1);

    for (UINT32 i = 0; i < g_ntables - 1; ++i)
    {
        int nh = (int)(std::pow((double)g_r, (double)i) * (double)g_l1 + 0.5);
        if (nh > NHIST)
            nh = NHIST;
        tagged_tables[i].n_hist = nh;
        tagged_tables[i].entries.assign(g_nentries, tagged_table_entry{-1, 0, 0});
    }
}

void tage_update(ADDRINT inst_ptr, bool taken)
// Update using TAGE.
{
    int pc = (int)(inst_ptr & ((1 << g_pc_bits) - 1));

    // STEP 1: Update the provider prediction counter (and u counter if tagged).
    if (g_provider == -1)
    {
        int idx = pc & table_index_mask();
        t0.entries[idx].pred = taken ? sat_inc(t0.entries[idx].pred, 3)
                                     : sat_dec(t0.entries[idx].pred);
    }
    else
    {
        int n_hist = tagged_tables[g_provider].n_hist;
        int idx = hash_index(pc, hist & hist_mask(n_hist), n_hist) & table_index_mask();
        tagged_table_entry &e = tagged_tables[g_provider].entries[idx];

        e.pred = taken ? sat_inc(e.pred, 3) : sat_dec(e.pred);

        bool provider_correct = (g_prediction == taken);
        bool alt_correct = (g_altpred == taken);

        if (provider_correct && !alt_correct)
        {
            e.u = sat_inc(e.u, 3);
        }
        else if (!provider_correct && alt_correct)
        {
            e.u = sat_dec(e.u);
        }
    }

    // STEP 2: If prediction was wrong, try allocation in longer-history tables.
    if (g_prediction != taken)
    {
        int start = (g_provider == -1) ? 0 : g_provider + 1;

        std::vector<int> candidates;
        for (int j = start; j < (int)tagged_tables.size(); ++j)
        {
            int n_hist = tagged_tables[j].n_hist;
            int idx = hash_index(pc, hist & hist_mask(n_hist), n_hist) & table_index_mask();
            if (tagged_tables[j].entries[idx].u == 0)
            {
                candidates.push_back(j);
            }
        }

        if (!candidates.empty())
        {
            int chosen = candidates[0];

            // Prefer shorter-history candidate with probability (prob-1)/prob.
            if (candidates.size() > 1 && (rand() % g_prob == 0))
            {
                chosen = candidates[1 + rand() % (candidates.size() - 1)];
            }

            int n_hist = tagged_tables[chosen].n_hist;
            int idx = hash_index(pc, hist & hist_mask(n_hist), n_hist) & table_index_mask();
            int tag = hash_tag(pc, hist & hist_mask(n_hist), n_hist);

            tagged_tables[chosen].entries[idx].tag = tag;
            tagged_tables[chosen].entries[idx].pred = taken ? 2 : 1; // weakly taken / weakly not taken
            tagged_tables[chosen].entries[idx].u = 0;
        }
        else
        {
            for (int j = start; j < (int)tagged_tables.size(); ++j)
            {
                int n_hist = tagged_tables[j].n_hist;
                int idx = hash_index(pc, hist & hist_mask(n_hist), n_hist) & table_index_mask();
                tagged_tables[j].entries[idx].u = sat_dec(tagged_tables[j].entries[idx].u);
            }
        }
    }

    // STEP 3: Shift global history.
    hist = ((hist << 1) | (taken ? (__uint128_t)1 : 0)) & hist_mask(NHIST);
}

bool tage_predict(ADDRINT inst_ptr)
// Predict using TAGE.
{
    int pc = (int)(inst_ptr & ((1 << g_pc_bits) - 1));

    // Default: base predictor
    g_provider = -1;
    g_prediction = (t0.entries[pc & table_index_mask()].pred >= 2);
    g_altpred = g_prediction;

    bool found_provider = false;

    // Walk longest => shortest tagged table.
    for (int i = (int)tagged_tables.size() - 1; i >= 0; --i)
    {
        int n_hist = tagged_tables[i].n_hist;
        int idx = hash_index(pc, hist & hist_mask(n_hist), n_hist) & table_index_mask();
        int tag = hash_tag(pc, hist & hist_mask(n_hist), n_hist);

        if (tagged_tables[i].entries[idx].tag == tag)
        {
            if (!found_provider)
            {
                g_provider = i;
                g_prediction = (tagged_tables[i].entries[idx].pred >= 2);
                found_provider = true;
            }
            else
            {
                g_altpred = (tagged_tables[i].entries[idx].pred >= 2);
                break;
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

    out << "L1: " << g_l1 << endl;
    out << "R: " << g_r << endl;
    out << "NTables: " << g_ntables << endl;
    out << "NEntries: " << g_nentries << endl;
    out << "PCBits: " << g_pc_bits << endl;
    out << "TagBits: " << g_tag_bits << endl;
    out << "Prob: " << g_prob << endl;

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
    CountSeen++;

    if (taken)
    {
        CountTaken++;
    }

    if (tage_predict(ins_ptr) == (bool)taken)
    {
        CountCorrect++;
    }

    tage_update(ins_ptr, (bool)taken);

    if (CountSeen == KnobBranchLimit.Value())
    {
        write_results(true);
        exit(0);
    }
}

VOID Instruction(INS ins, void *v)
{
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

    tage_init();

    INS_AddInstrumentFunction(Instruction, 0);
    PIN_AddFiniFunction(Fini, 0);

    PIN_StartProgram();

    return 0;
}

/* ===================================================================== */
/* eof */
/* ===================================================================== */