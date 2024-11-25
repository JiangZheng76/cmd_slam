#pragma once

#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <cstddef>
#include <iostream>
#include <vector>

#include "optimization/pcm_pgo/max_clique_finder/graphIO.h"

using namespace std;

#ifdef _DEBUG
int DEBUG = 1;
#endif

namespace FMC {

// Function Definitions
bool fexists(const char* filename);
double wtime();
void usage(char* argv0);
int getDegree(vector<int>* ptrVtx, int idx);
void print_max_clique(vector<int>& max_clique_data);

int maxClique(CGraphIO* gio, size_t l_bound, vector<int>* max_clique_data);
void maxCliqueHelper(CGraphIO* gio,
                     vector<int>* U,
                     size_t sizeOfClique,
                     size_t* maxClq,
                     vector<int>* max_clique_data_inter);

int maxCliqueHeu(CGraphIO* gio, vector<int>* max_clique_data);
void maxCliqueHelperHeu(CGraphIO* gio,
                        vector<int>* U,
                        size_t sizeOfClique,
                        size_t* maxClq,
                        vector<int>* max_clique_data_inter);

int maxCliqueHeuIncremental(CGraphIO* gio,
                            size_t num_new_lc,
                            size_t prev_maxclique_size,
                            vector<int>* max_clique_data);

}  // namespace FMC