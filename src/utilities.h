#ifndef UTILITIES_H
#define UTILITIES_H

#include <vector>
#include <random>
#include <math.h>
#include <sys/resource.h>

/* definition of aliases for Decentralized A* hashing methods (HDA*: Hash Distributed A*) */
#define MULTIPLICATIVE_HASH 1
#define ZOBRIST_HASH 2
#define ABSTRACT_ZOBRIST_HASH 3

/* if DEBUG_MODE is set then the program will NOT ask the user to insert inputs from the menù interface,
but it will take the inputs directly defined in the source code of debug_program_inputs function of "utilities.cpp"
(useful to debug quickly) */
#define DEBUG_MODE 0


/*************************************/
/*                                   */
/*    Global variables definition    */
/*                                   */
/*************************************/

extern sem_t *barrier_1;        // semaphore used to implement a synchronization barrier
extern sem_t *barrier_2;        // semaphore used to implement a synchronization barrier
extern pthread_mutex_t *mb;     // mutex used to implement a synchronization barrier
extern unsigned int barrier_counter;    // counter used to implement a synchronization barrier
extern bool error_occurred;     // flag that gets set if an error occurred during any phase of threads' execution
extern pthread_t *tid_v;        // TIDs of all created threads
extern std::mutex m_abort;      // mutex locked by the thread that want to terminate all other threads and itself (because of some error that occurred)

/* time stamps to keep track of single-threaded and multi-threaded program performance */
extern std::chrono::steady_clock::time_point start_timer_graph_gen, stop_timer_graph_gen;
extern std::chrono::steady_clock::time_point start_timer_graph_read_from_file, stop_timer_graph_read_from_file;
extern std::chrono::steady_clock::time_point start_timer_a_star_sequential, stop_timer_a_star_sequential;
extern std::chrono::steady_clock::time_point start_timer_a_star_centralized, stop_timer_a_star_centralized;
extern std::chrono::steady_clock::time_point start_timer_a_star_decentralized, stop_timer_a_star_decentralized;


/******************************/
/*                            */
/*    Functions prototypes    */
/*                            */
/******************************/

/*
 * name: random_in_range
 * purpose: generates a pseudo-random number in a given range.
 * parameters: lower bound; upper bound.
 * return value: generated number.
 */
template<typename T>
T random_in_range(T lb, T ub);

/*
 * name: max
 * purpose: computes the maximum value among the ones passed as parameters.
 * parameters: 1st value; 2nd value.
 * return value: maximum value among 1st and 2nd value.
 */
template<typename T>
T max(T a, T b);

/*
 * name: min
 * purpose: computes the minimum value among the ones passed as parameters.
 * parameters: 1st value; 2nd value.
 * return value: minimum value among 1st and 2nd value.
 */
template<typename T>
T min(T a, T b);

/*
 * name: distance_between_nodes
 * purpose: computes the distance between 2 nodes (in Euclidian norm).
 * parameters: 1st node; 2nd node.
 * return value: distance between 1st and 2nd node.
 */
float distance_between_nodes(const Node& n1, const Node& n2);

/*
 * name: printf_graph
 * purpose: prints a graph, i.e. vector of nodes (and the links that connect them).
 * parameters: vector of nodes to be printed.
 * return value: none.
 */
void printf_graph(const std::vector<Node>& nodes_v);

/*
 * name: print_best_path
 * purpose: prints the best graph's path (i.e. sequence of nodes) that connects start and stop nodes.
 * parameters: path to be printed.
 * return value: none.
 */
void print_best_path(graph_path_t& path);

/*
 * name: reset_path
 * purpose: resets a path variable.
 * parameters: path variable to reset.
 * return value: none.
 */
void reset_path(graph_path_t& best_path);

/*
 * name: init_threads_synch
 * purpose: initializes synchronization primitives in order to synchronize the threads.
 * parameters: number of synchronization primitives that we want to create; vector of structures used for threads synchronization.
 * return value: none.
 */
void init_threads_synch(const unsigned int& num_threads_synch,
                        std::vector<ptr_to_atomic_flag>& threads_synch);

/*
 * name: abort_all_threads
 * purpose: aborts the execution of every thread of the process to which the current thread belongs to (except the main thread of the process).
 * parameters: TID of current thread; number of threads that are running concurrently and must be aborted; flag that notifies to the main thread if any error occurred or not.
 * return value: none.
 */
void abort_all_threads(const unsigned int& thread_id, const unsigned int& num_threads, bool& error_occurred);

/*
 * name: abort_current_thread
 * purpose: aborts the execution of current thread.
 * parameters: ID of the signal received (which triggered the execution of 'abort_current_thread').
 * return value: none.
 */
void abort_current_thread(int sig);

/*
 * name: optimal_graph_axes_partit_size
 * purpose: compute the optimal size of each graph partition along X and Y coordinates in order to have the least overall number of links in the graph
 *          without excessively increase the overhead due to threads parallelism.
 * parameters: number of graph nodes.
 * return value: size of each graph partition along X and Y coordinates.
 */
unsigned int optimal_graph_axes_partit_size(const int& num_nodes);

/*
 * name: a_star_hash
 * purpose: given a graph's node, hash it in order to assign it to a running thread.
 * parameters: node to be hashed; number of threads running concurrently; random-bit-strings table; hashing method chosen by the user;
 *              mask for feature projection function of the Abstract Zobrist Hashing.
 * return value: thread ID obtained by hashing the node.
 */
int a_star_hash(const Node& n, const unsigned int& num_threads, const std::vector<int>& R, const int& hash_type, const unsigned int& AZ_hash_mask);

/*
 * name: M_hash
 * purpose: implements the Multiplicative hashing method.
 * parameters: node to be hashed; number of threads running concurrently.
 * return value: thread ID obtained by hashing the node.
 */
int M_hash(const Node& n, const unsigned int& num_threads);

/*
 * name: Z_hash
 * purpose: implements the Zobrist hashing method.
 * parameters: node to be hashed; number of threads running concurrently; random-bit-strings table.
 * return value: thread ID obtained by hashing the node.
 */
int Z_hash(const Node& n, const unsigned int& num_threads, const std::vector<int>& R);

/*
 * name: AZ_hash
 * purpose: implements the Abstract Zobrist hashing method.
 * parameters: node to be hashed; number of threads running concurrently; random-bit-strings table; mask for feature projection function of the Abstract Zobrist Hashing.
 * return value: thread ID obtained by hashing the node.
 */
int AZ_hash(const Node& n, const unsigned int& num_threads, const std::vector<int>& R, const unsigned int& AZ_hash_mask);

/*
 * name: debug_program_inputs
 * purpose: take program inputs directly from source code of this function, without the need of going through the A* menù every time (useful for debug).
 * parameters: program functionality to be run; number of nodes of generated graph; number of threads that have to concurrently generate the graph;
 *              number of threads that have to concurrently read the graph from file; A* versions that the user wants to run;
 *              number of threads that have to concurrently run Centralized A* version; number of threads that have to concurrently run Decentralized A* version;
 *              hashing method to be used in the Decentralized A* version; ID of "start" and "stop" nodes of A* algorithm.
 * return value: none.
 */
void debug_program_inputs(int& menu_option,
                        int& num_nodes_graph_gen,
                        int& num_threads_graph_gen,
                        int& num_threads_graph_read,
                        std::vector<bool>& a_star_versions_to_run,
                        int& num_threads_centr_a_star,
                        int& num_threads_decentr_a_star,
                        int& hash_type_decentr_a_star,
                        std::pair<int, int>& start_stop_nodes_idx);



/***********************************/
/*                                 */
/*    Functions implementations    */
/*                                 */
/***********************************/

/* NOTE: the following functions need to be implemented in .h file because they use templates */

template<typename T>
T random_in_range(T lb, T ub) {

    if (lb > ub)
        throw std::invalid_argument("in \'random_in_range\' function the lower bound must be lower or equal to the upper bound.\n");

    if (lb == ub)
        return lb;

    if (std::is_same<T, int>::value && /*lb >= 0*/ ub - lb > 0 && ub <= RAND_MAX)
        return static_cast<T>((rand() % static_cast<int>(ub - lb + 1)) + lb);

    return static_cast<T>((((double)rand() / (double)RAND_MAX) * ((double)ub - lb)) + lb);
}


template<typename T>
T max(T a, T b) {

    return (a >= b) ? a : b;
}


template<typename T>
T min(T a, T b) {

    return (a <= b) ? a : b;
}

#endif