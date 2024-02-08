#ifndef GRAPH_GEN_STORE_LOAD_H
#define GRAPH_GEN_STORE_LOAD_H

#include <unordered_map>
#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>
#include <atomic>
#include <semaphore.h>

/* MAX_GRAPH_AXES_PARTIT represents the maximum value (found through a trial and error procedure) of the graph partitions sizes along X and Y
coordinates that guarantees an increase of threads parallelism without introducing too much overhead due to threads communication.
NOTE: this value could change depending on the computational power of the hardware where this program gets executed. Having more computational
power available allows us to increase this value (and vice versa). */
#define MAX_GRAPH_AXES_PARTIT 300

/* MAX_ADJ_PARTIT_IN_2D_GRAPH represents the maximum number of partitions that could be adjacent to each graph partition,
which is always 4 (i.e. one on top, one on bottom, one left and one right) */
#define MAX_ADJ_PARTIT_IN_2D_GRAPH 4

/* UNDEFINED_PARTIT represents the ID of an undefined graph partition */
#define UNDEFINED_PARTIT -1

/* MAX_COEFF_WEIGHT_LINK and MIN_COEFF_WEIGHT_LINK are the upper and lower bounds of the random coefficient that will be multiplied by
the weights assigned to each link, respectively. */
#define MAX_COEFF_WEIGHT_LINK 2
#define MIN_COEFF_WEIGHT_LINK 1

/* PRINT_GRAPH_AFTER_GEN is a flag that the user can set in order to print all the graph's nodes and links after its generation */
#define PRINT_GRAPH_AFTER_GEN 0

/* PRINT_GRAPH_AFTER_GEN is a flag that the user can set in order to print all the graph's nodes and links after reading it from file */
#define PRINT_GRAPH_AFTER_READ 0



/********************************************/
/*                                          */
/*    Aliases and data types definitions    */
/*                                          */
/********************************************/

// x-y coordinates of graph's nodes
typedef std::pair<int, int> coordinate;

// set of links (and corresponding weights) that connect a node with its neighbors
typedef std::unordered_map<unsigned int, unsigned int> link_weight_umap;

// pointer to an atomic
typedef std::unique_ptr<std::atomic_flag> ptr_to_atomic_flag;

// pointer to a mutex
typedef std::unique_ptr<std::mutex> ptr_to_mutex;

// pointer to a condition variable
typedef std::unique_ptr<std::condition_variable> ptr_to_cond_var;

/******************** Graph's nodes structure ********************/
class Node {

    public:
        /* copy control features definition */
        Node ();                                    // default constructor
        Node (const Node& other);                   // copy constructor
        Node (Node&& other) noexcept;               // move constructor
        Node& operator= (const Node& other);        // copy assignment
        Node& operator= (Node&& other) noexcept;    // move assignment
        ~Node ();                                   // destructor

        /* overloaded operators */
        bool operator==(const Node& rhs) const;
        bool operator!=(const Node& rhs) const;
        bool operator<(const Node& rhs) const;
        bool operator>(const Node& rhs) const;
        friend std::ostream& operator<<(std::ostream& os, const Node& node);

        /* getters and setters */
        unsigned int get_id() const;
        int get_x() const;
        int get_y() const;
        link_weight_umap get_links() const;
        unsigned int get_link_weight(const unsigned int& node_id) const;
        void set_id(const unsigned int& id);
        void set_x(const int& x);
        void set_y(const int& y);
        void set_links(const link_weight_umap& umap);
        void set_link(const unsigned int& node_id, const unsigned int& weight);

    private:
        unsigned int id;    // node's ID
        int x;              // node's coordinate along x axis
        int y;              // node's coordinate along y axis
        std::unique_ptr<link_weight_umap> neighbor;     // node's links to neighbor nodes
};


/******************** Hash function for std::pair objects ********************/
typedef struct hash_pair_s {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {

        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        // if h1 != h2 then return their XOR
        if (h1 != h2)
            return h1 ^ h2;

        // if h1 == h2 then return just h1 (their XOR is zero)
        return h1;
    }
} hash_pair_t;



/*************************************/
/*                                   */
/*    Global variables definition    */
/*                                   */
/*************************************/

extern unsigned int num_graph_partit;      // number of partitions in which the graph gets divided (to allow efficient parallel operations on it)
extern unsigned int tot_num_links_graph;   // total number of graph's links
extern unsigned int max_graph_coord_span;  // dimension of x-y coordinates values range
extern unsigned int graph_axes_partit;     // size of graph partitions along each graph's axis (i.e. X and Y axes)


/******************************/
/*                            */
/*    Functions prototypes    */
/*                            */
/******************************/

/*
 * name: graph_generation
 * purpose: creates the threads that will generate the graph and store it on disk.
 * parameters: number of nodes to be generated; number of threads that concurrently have to generate the graph; path of the file on disk where to store the graph.
 * return value: 0 if graph generation and storing on disk were successfull, -1 otherwise.
 */
int graph_generation(unsigned int num_nodes,
                    unsigned int num_threads,
                    std::vector<Node>& nodes_v,
                    char *path_file_graph);

/*
 * name: nodes_links_generation
 * purpose: makes (multiple) threads run concurrently and generates the nodes and the links of the graph, which is then stored on disk (i.e. written on a file).
 * parameters: ID of the current thread; number of nodes to be generated; number of threads that concurrently have to generate the graph;
 *             vector to be filled with all generated nodes; vector of structures used for threads synchronization; vector to be filled with the number of links that has
 *             been created for each graph partition; cumulative sum of the number of links that has been created for each graph partition;
 *             path of the file on disk where to store the graph; seed for random numbers generation of the current thread.
 * return value: none.
 */
void nodes_links_generation(unsigned int thread_id,
                            unsigned int num_nodes,
                            unsigned int num_threads,
                            std::vector<Node>& nodes_v,
                            std::vector<ptr_to_atomic_flag>& threads_synch,
                            std::vector<unsigned int>& links_gen_per_partit,
                            std::vector<unsigned int>& cum_links_gen_per_partit,
                            char *path_file_graph,
                            int rand_seed);

/*
 * name: graph_read_from_file
 * purpose: creates the threads that will read the graph from file.
 * parameters: number of threads that concurrently have to read the graph from file; vector to be filled with all graph's nodes;
 *              path of the file on disk where the graph is stored.
 * return value: 0 if graph reading from file was successfull, -1 otherwise.
 */
int graph_read_from_file(unsigned int num_threads,
                        std::vector<Node>& nodes_v,
                        char *path_file_graph);

/*
 * name: nodes_links_read_from_file
 * purpose: makes (multiple) threads run concurrently and read the nodes and the links of the graph from file.
 * parameters: ID of the current thread; number of graph's nodes; number of graph's links; number of threads that concurrently have to read the graph;
 *             vector to be filled with all graph's nodes; vector of structures used for threads synchronization; vector to be filled with the number of links that
 *             has been created for each graph partition; cumulative sum of the number of links that has been created for each graph partition;
 *             path of the file on disk where the graph is stored.
 * return value: none.
 */
void nodes_links_read_from_file(unsigned int thread_id,
                                unsigned int num_nodes,
                                unsigned int num_links,
                                unsigned int num_threads,
                                std::vector<Node>& nodes_v,
                                std::vector<ptr_to_atomic_flag>& threads_synch,
                                std::vector<unsigned int>& links_gen_per_partit,
                                std::vector<unsigned int>& cum_links_gen_per_partit,
                                char *path_file_graph);

#endif