#ifndef A_STAR_H
#define A_STAR_H

#include <unordered_map>
#include <utility>
#include <vector>
#include <queue>


/* UNDEFINED_COST is the value to which it is initialized the best-route cost at the beginning of the multithreaded A* algorithm.
If at the end of the algorithm the best-route is still equal to UNDEFINED_COST, then no best-route has been found. */
#define UNDEFINED_COST -1

/* UNDEFINED_THREAD represents a non-specified thread ID. */
#define UNDEFINED_THREAD -1

/* UNDEFINED_NODE represents a non-specified node ID. */
#define UNDEFINED_NODE -1

/* R_PARTIT_PER_THREAD is the number of partitions of the random-bit-strings table (used for HDA* algorithm)
that will be assigned (on average) to each thread during the generation phase of the table (to increase threads parallelism) */
#define R_PARTIT_PER_THREAD 3

/* FILL_R_IN_PARALLEL is a flag that the user can set in order to parallelize the filling-up procedure of the random-bit-strings table R
used in the Decentralized A* algorithm with Zobrist and Abstract-Zobrist hashing methods.
NOTE: tests conducted with different graph sizes showed that the non-parallel fill-up of R is consistently faster than the parellel version,
because of the limited size of the R data structure which doesn't justify the overhead introduced by parallelization. */
#define FILL_R_IN_PARALLEL 0

/* TEST_HIGH_NODE_EXPANSION_COST is a flag that, if set, simulates graphs where the expansion of a new graph's node is really expensive,
so that the user can actually see the effectiveness of the Centralized A* approach with respect to the Sequential and Decentralized A* in such cases.
The additional "cost" that gets added to the expansion of a node is determined by the macro NODE_EXPANSION_DELAY. */
#define TEST_HIGH_NODE_EXPANSION_COST 0

/* NODE_EXPANSION_DELAY represents the additional delay (in milliseconds) that gets added to the time needed for expanding a graph's node.
This additional delay gets added only if the TEST_HIGH_NODE_EXPANSION_COST flag is set. */
#define NODE_EXPANSION_DELAY 1



/*******************************************/
/*                                         */
/*      Aliases and data types for A*      */
/*                                         */
/*******************************************/

// tuple that, given a node 'N', stores f(N), h(N) and N itself
typedef std::tuple<double, double, Node> list_elem;

// list of nodes already discovered but still to be expanded by A* algorithm
typedef std::priority_queue<list_elem, std::vector<list_elem>, std::greater<list_elem>> open_list;

// list of nodes already expanded by A* algorithm (key of unordered_map is the node's ID)
typedef std::unordered_map<unsigned int, list_elem> closed_list;

// set of nodes in a graph's path (the order in which the nodes must be traversed is given by the key of the hash map)
typedef std::unordered_map<unsigned int, Node> path_umap;

// message exchanged between threads in multi-threaded Decentralized A* algorithm (the tuple contains node N, cost g(N) and parent(N))
typedef std::tuple<Node, double, Node> msg_t;

// buffer of messages
typedef std::queue<msg_t> msg_buffer_t;

/* parent request message (key-value are, respectively, the node of which it is requested to know the parent and the flag that represents
if a new parent request arrived or not) */
typedef std::pair<Node, bool> parent_request_t;

/* parent reply message (key-value are, respectively, the requested parent node and the flag that represents if a new parent reply is available or not) */
typedef std::pair<Node, bool> parent_reply_t;

// buffer of parent requests
typedef std::vector<parent_request_t> parent_request_buffer_t;

/* cumulative counter of sent and received messages for each thread. The first value of std::pair is the vector containing the counter associated with each
running thread, while the second value of std::pair represents the ID of the thread that now has to check the cumulative counter. */
typedef std::pair<std::vector<int>, std::atomic<unsigned int>> acc_msg_counter_t;


/******************** Path (i.e. sequence of nodes) of the graph ********************/
typedef struct graph_path_s {

    std::unique_ptr<path_umap> path_ptr;    // path itself
    unsigned int path_num_nodes;            // number of nodes contained in the path
    std::atomic<int> path_cost;             // path's overall cost

} graph_path_t;


/******************** Hash function for Node objects ********************/
namespace std {
    template <> struct hash<Node> {
        std::size_t operator()(const Node& node) const noexcept {
            return std::hash<int>()(abs(node.get_x()) ^ abs((node.get_y() << 16)));
        }
    };
}



/*************************************/
/*                                   */
/*    Global variables definition    */
/*                                   */
/*************************************/

extern unsigned int num_R_partit;   // number of partitions in which the random-bit-strings table gets divided (to allow efficient parallel operations on it)
extern std::mutex m_R;              // mutex used during the phase of filling up the random-bit-strings table


/******************************/
/*                            */
/*    Functions prototypes    */
/*                            */
/******************************/

/*
 * name: a_star_sequential
 * purpose: runs the Sequential A* (i.e.single-threaded A*) onto a graph.
 * parameters: graph onto which A* has to be run; start node; stop node; data structure where to store the best path found (if is there any).
 * return value: none.
 */
void a_star_sequential (std::vector<Node>& graph, Node start, Node stop, graph_path_t& best_path);

/*
 * name: a_star_centralized
 * purpose: setup and launch threads for the execution of the Centralized A* onto a graph.
 * parameters: graph onto which A* has to be run; start node; stop node; data structure where to store the best path found (if is there any).
 * return value: 0 on successful completion of Centralized A* algorithm, -1 in case of errors.
 */
int a_star_centralized (std::vector<Node>& graph, Node start, Node stop, unsigned int num_threads, graph_path_t& best_path);

/*
 * name: a_star_centralized_thread
 * purpose: single thread that runs the Centralized A* onto a graph.
 * parameters: graph onto which A* has to be run; start node; stop node; ID of stop node owner; number of threads to be launched;
 *              shared open list; shared from map; shared cost map; shared current solution cost; shared vector to determine end
 *              condition; shared lock m0 for the access synchronization of the "open" and "end" structures; shared lock m1 for
 *              the access synchronization of the "from" and "cost" maps; shared lock m2 for the access synchronization of the "closed" list.
 * return value: none.
 */
void a_star_centralized_thread(const std::vector<Node>& graph,
                            Node stop,
                            Node start,
                            unsigned int thread_num,
                            open_list& open,
                            closed_list& closed,
                            std::unordered_map<Node, Node>& from,
                            std::unordered_map<Node, unsigned int>& cost,
                            graph_path_t& best_path,
                            std::vector<int>& end,
                            std::mutex& m0,
                            std::mutex& m1,
                            std::mutex& m2);

/*
 * name: a_star_decentralized
 * purpose: setup and launch threads for the execution of the Decentralized A* onto a graph.
 * parameters: graph onto which A* has to be run; start node; stop node; hashing method; number of threads to be launched;
 *              data structure where to store the best path found (if is there any).
 * return value: 0 on successful completion of Decentralized A* algorithm, -1 in case of errors.
 */
int a_star_decentralized (std::vector<Node>& graph,
                            Node start,
                            Node stop,
                            int hash_type,
                            unsigned int num_threads,
                            graph_path_t& best_path);

/*
 * name: a_star_decentralized_thread
 * purpose: runs the Decentralized A* onto a graph.
 * parameters: graph onto which A* has to be run; start node; stop node; ID of stop node owner; hashing method; number of threads to be launched;
 *              ID of current thread; open list of current thread; random-bit-strings table; flag to notify that random-bit-strings table has been
 *              filled up; mask for feature projection function of the Abstract Zobrist Hashing; condition variable used during the random-bit-strings table
 *              fill up procedure; atomic flags used for synch; condition variables used for synch; mutexes used for synch; vector of message buffers;
 *              buffer of parent requests; parent reply; data structure where to store the best path found (if is there any); flag to notify that a thread
 *              found a new better path from start to stop node; seed for random numbers generation; cumulative message counter; flag that notifies the
 *              termination of the A* algorithm; ID of the thread that initiates the A* termination procedure.
 * return value: none.
 */
void a_star_decentralized_thread (std::vector<Node>& graph,
                                const Node& start,
                                const Node& stop,
                                int& stop_node_owner,
                                int hash_type,
                                unsigned int num_threads,
                                unsigned int thread_id,
                                open_list& open,
                                std::vector<int>& R,
                                bool& R_ready,
                                const unsigned int& AZ_hash_mask,
                                std::condition_variable& cv_R,
                                std::vector<ptr_to_atomic_flag>& threads_synch,
                                std::vector<ptr_to_cond_var>& cv_threads_v,
                                std::vector<ptr_to_mutex>& m_msg_v,
                                std::vector<msg_buffer_t>& msg_buffer_v,
                                parent_request_buffer_t& parent_request_buffer,
                                parent_reply_t& parent_reply,
                                graph_path_t& best_path,
                                std::atomic<bool>& found_new_best_path,
                                int rand_seed,
                                acc_msg_counter_t& acc_msg_counter,
                                std::atomic<bool>& algorithm_terminated,
                                std::atomic<int>& termination_starter_thread);

/*
 * name: rebuild_path_single_thread
 * purpose: rebuilds the path from "start" to "stop" node in case a single thread has all the information needed to do it (not possible in the Decentralized A*).
 * parameters: data structure where to store the best path found (if is there any); data structure containing parent-child node connections; start node; stop node.
 * return value: none.
 */
void rebuild_path_single_thread(graph_path_t& best_path,
                                std::unordered_map<Node, Node>& from,
                                const Node& start,
                                const Node& stop);

/*
 * name: calculate_h
 * purpose: computes the A* heuristic h(N1, N2) value, i.e. the Euclidean distance between nodes N1 and N2.
 * parameters: 1st node; 2nd node.
 * return value: the Euclidean distance between 1st and 2ns node.
 */
double calculate_h(const Node& n1, const Node& n2);

/*
 * name: add_node_if_worth_expanding
 * purpose: if a newly received node is worth to be expanded then add it to the open list of the current thread.
 * parameters: ID of current thread; number of threads concurrently executing A*; new node; new cost of new node; parent of new node; destionation node of A*;
 *              open list of current thread; closed list of current thread; data structure that relates each node N with its cost g(N) from the start node;
 *              data structure used to remember from which node (parent) a thread arrived in order to explore a certain node (child).
 * return value: none.
 */
void add_node_if_worth_expanding (unsigned int& thread_id,
                                unsigned int& num_threads,
                                Node& n_child,
                                double& g_child,
                                Node& n_parent,
                                const Node& stop,
                                open_list& open,
                                closed_list& closed,
                                std::unordered_map<Node, unsigned int>& cost,
                                std::unordered_map<Node, Node>& from);

#endif