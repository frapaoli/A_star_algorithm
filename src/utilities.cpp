#include "graph_gen_store_load.h"
#include "a_star.h"
#include "utilities.h"
#include "menu.h"

#include <iostream>
#include <typeinfo>
#include <signal.h>
#include <math.h>
#include <fcntl.h>



/*************************************/
/*                                   */
/*    Global variables definition    */
/*                                   */
/*************************************/

sem_t *barrier_1;       // semaphore used to implement a synchronization barrier
sem_t *barrier_2;       // semaphore used to implement a synchronization barrier
pthread_mutex_t *mb;    // mutex used to implement a synchronization barrier
unsigned int barrier_counter;   // counter used to implement a synchronization barrier
bool error_occurred;    // flag that gets set if an error occurred during any phase of threads' execution
pthread_t *tid_v;       // TIDs of all created threads
std::mutex m_abort;     // mutex locked by the thread that want to terminate all other threads and itself (because of some error that occurred)

/* time stamps to keep track of single-threaded and multi-threaded program performance */
std::chrono::steady_clock::time_point start_timer_graph_gen, stop_timer_graph_gen;
std::chrono::steady_clock::time_point start_timer_graph_read_from_file, stop_timer_graph_read_from_file;
std::chrono::steady_clock::time_point start_timer_a_star_sequential, stop_timer_a_star_sequential;
std::chrono::steady_clock::time_point start_timer_a_star_centralized, stop_timer_a_star_centralized;
std::chrono::steady_clock::time_point start_timer_a_star_decentralized, stop_timer_a_star_decentralized;



/********************************************************************/
/*                                                                  */
/*      Implementation of utility functions for A* algorithm        */
/*                                                                  */
/********************************************************************/

float distance_between_nodes(const Node& n1, const Node& n2) {

    if (typeid(n1.get_x()) != typeid(int) || typeid(n1.get_y()) != typeid(int) || typeid(n2.get_x()) != typeid(int) || typeid(n2.get_y()) != typeid(int))
        throw std::invalid_argument("in \'distance_between_nodes\' function the two arguments must be two instances of class Node.\n");

    // return Euclidean distance between nodes "n1" and "n2"
    return sqrt(pow(n1.get_x() - n2.get_x(), 2) + pow(n1.get_y() - n2.get_y(), 2));
}


void printf_graph(const std::vector<Node>& nodes_v) {

    // print the information about all the nodes belonging to the graph
    std::cout << "/**** GRAPH ****/" << std::endl << std::endl;
    for (const Node& node : nodes_v) {
        
        std::cout << node << std::endl;
    }
    std::cout << std::endl;
}


void print_best_path(graph_path_t& path) {

    if (path.path_cost.load() == UNDEFINED_COST) {
        std::cout << "No path exists between \"start\" and \"stop\" node." << std::endl;
    }
    else {
        Node n;

        std::cout << "/**** BEST PATH ****/" << std::endl << std::endl;
        for (int node_idx = path.path_num_nodes - 1; node_idx >= 0; --node_idx) {
            
            // print the i-th node of the path
            n = (*(path.path_ptr))[node_idx];
            std::cout << "ID: " << n.get_id() << ", x: " << n.get_x() << ", y: " << n.get_y();

            if (node_idx == (int)path.path_num_nodes - 1)
                std::cout << "    (start)";
            else if (node_idx == 0)
                std::cout << "    (stop)";

            std::cout << std::endl;
        }

        std::cout << std::endl << "Overall path cost: " << path.path_cost << std::endl << std::endl;
    }

}


void reset_path(graph_path_t& path) {

    // reset path status
    path.path_ptr = nullptr;
    path.path_num_nodes = 0;
    path.path_cost = UNDEFINED_COST;
}


void init_threads_synch(const unsigned int& num_threads_synch,
                        std::vector<ptr_to_atomic_flag>& threads_synch) {
                                
    for (unsigned int i = 0; i < num_threads_synch; ++i) {

        /* initialize atomic flags for threads synchronization */

        // NOTE: need to use "std::unique_ptr<std::atomic_flag>" because "std::atomic_flag" is not movable
        ptr_to_atomic_flag atomic_flag_ptr = std::make_unique<std::atomic_flag>();
        (*atomic_flag_ptr).clear();

        // store all synchronization objects in a std::vector
        threads_synch.emplace_back(std::move(atomic_flag_ptr));
    }
}


void abort_all_threads(const unsigned int& thread_id, const unsigned int& num_threads, bool& error_occurred) {

    // abort all threads (except the current one)
    for (unsigned int id = 0; id < num_threads; ++id) {
        if (id == thread_id)
            continue;
        
        // signal the i-th thread to make it terminate its execution
        pthread_kill(tid_v[id], SIGUSR1);
    }

    // notify the main thread that an error occurred
    error_occurred = true;

    // terminate current thread
    pthread_exit(NULL);
}


void abort_current_thread(int sig) {

    /* NOTE: sig is supposed to be == SIGUSR1, but if sig != SIGUSR1 then there would be an error somewhere, hence the process should be aborted anyway.
    Therefore, we avoid to perform the check "if (sig == SIGUSR1) ... else ..." */
    
    // terminate current thread
    pthread_exit(NULL);
}


unsigned int optimal_graph_axes_partit_size(const int& num_nodes) {

    if (num_nodes <= 0)
        throw std::invalid_argument("in \'optimal_graph_axes_partit_size\' function the number of nodes passed as parameter must be >=0.\n");

    /* NOTE: the following formulas were derived by taking into account the arbitrary choice made to generate the graph,
    and it returns the optimal size of each graph partition along X and Y coordinates in order to have the least overall number of links in the graph
    without excessively increase the overhead due to threads parallelism.
    This is just an implementation choice made in order to obtain a graph where it is “challenging” to find a path between
    two nodes and to reduce the time needed for the graph generation, but any choice > 0 of the graph partitions size is totally fine.*/

    int opt_graph_axes_partit_size = (sqrt(1 + 8*(num_nodes - 1)) + 1) / 4;

    return (unsigned int) min(max(opt_graph_axes_partit_size, 1), MAX_GRAPH_AXES_PARTIT);
}


int a_star_hash(const Node& n, const unsigned int& num_threads, const std::vector<int>& R, const int& hash_type, const unsigned int& AZ_hash_mask) {

    // check correctness of number of threads
    if (num_threads <= 0)
        return -1;

    // choose the hash function to be used
    switch (hash_type) {
        case MULTIPLICATIVE_HASH:
            return M_hash(n, num_threads);
        case ZOBRIST_HASH:
            return Z_hash(n, num_threads, R);
        case ABSTRACT_ZOBRIST_HASH:
            return AZ_hash(n, num_threads, R, AZ_hash_mask);
    }

    return -1;  // in case "hash_type" is none of the above hashing methods
}


int M_hash(const Node& n, const unsigned int& num_threads) {

    // golden ratio
    float A = (1 + sqrt(5)) / 2;

    // hashing the node's coordinates
    int hash_x = abs((int) std::hash<int>()(static_cast<int>(n.get_x() * n.get_y())));
    int hash_y = abs((int) std::hash<int>()(static_cast<int>(n.get_y() ^ n.get_x())));

    // key of multiplicative hashing (given by hashing the node)
    double k = (hash_x == 0 || hash_y == 0) ? (double)sqrt(hash_x + hash_y) : (double)hash_x / hash_y;

    // return the result of the Multiplicative hash function
    return ((int)(num_threads * (k*A - (int)(k*A)))) % num_threads;
}


int Z_hash(const Node& n, const unsigned int& num_threads, const std::vector<int>& R) {

    // get the node's x and y coordinates "shifted" by an amount that ensures that the resulting new x1 and x2 coordinates are both positive
    unsigned int x1 = (unsigned int) (n.get_x() + (max_graph_coord_span / 2) + 1);
    unsigned int x2 = (unsigned int) (n.get_y() + (max_graph_coord_span / 2) + 1);

    // ensure that x1 and x2 can be used as indexes to access the random-bit-strings table R
    x1 = x1 % R.size();
    x2 = x2 % R.size();

    // return the result of the Zobrist hash function
    return (R[x1] ^ R[x2]) % num_threads;
}


int AZ_hash(const Node& n, const unsigned int& num_threads, const std::vector<int>& R, const unsigned int& AZ_hash_mask) {

    // get the node's x and y coordinates "shifted" by an amount that ensures that the resulting new x1 and x2 coordinates are both positive
    unsigned int x1 = (unsigned int) (n.get_x() + (max_graph_coord_span / 2) + 1);
    unsigned int x2 = (unsigned int) (n.get_y() + (max_graph_coord_span / 2) + 1);

    // define the mask to be used in the feature-projection function A(x)
    unsigned int R_size = R.size();

    // apply the feature-projection function (useful to reduce communication overhead among threads)
    unsigned int A_x1 = x1 & AZ_hash_mask;
    unsigned int A_x2 = x2 & AZ_hash_mask;

    // ensure that A_x1 and A_x2 can be used as indexes to access the random-bit-strings table R
    A_x1 = A_x1 % R_size;
    A_x2 = A_x2 % R_size;

    // return the result of the Abstract Zobrist hash function
    return (R[A_x1] ^ R[A_x2]) % num_threads;
}


void debug_program_inputs(int& menu_option,
                        int& num_nodes_graph_gen,
                        int& num_threads_graph_gen,
                        int& num_threads_graph_read,
                        std::vector<bool>& a_star_versions_to_run,
                        int& num_threads_centr_a_star,
                        int& num_threads_decentr_a_star,
                        int& hash_type_decentr_a_star,
                        std::pair<int, int>& start_stop_nodes_idx) {
    
    /* the programmer can set DEBUG_MODE flag and directly modify program inputs from here,
    without the need of going through the A* menù every time */
    menu_option = 2;
    num_nodes_graph_gen = 10000;
    num_threads_graph_gen = 8;
    num_threads_graph_read = 8;
    a_star_versions_to_run.emplace_back(true);
    a_star_versions_to_run.emplace_back(true);
    a_star_versions_to_run.emplace_back(true);
    num_threads_centr_a_star = 4;
    num_threads_decentr_a_star = 8;
    hash_type_decentr_a_star = 2;
    start_stop_nodes_idx.first = -1;
    start_stop_nodes_idx.second = -1;

}
