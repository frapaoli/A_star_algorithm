#include "graph_gen_store_load.h"
#include "a_star.h"
#include "utilities.h"
#include "menu.h"

#include <iostream>
#include <exception>
#include <chrono>
#include <shared_mutex>
#include <memory>
#include <cstring>
#include <utility>
#include <type_traits>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <fcntl.h>
#include <math.h>
#include <sys/mman.h>
#include <pthread.h>
#include <signal.h>


/********************************/
/*                              */
/*      Global variables        */
/*                              */
/********************************/

unsigned int num_graph_partit;      // number of partitions in which the graph gets divided (to allow efficient parallel operations on it)
unsigned int tot_num_links_graph;   // total number of graph's links
unsigned int max_graph_coord_span;  // dimension of x-y coordinates values range
unsigned int graph_axes_partit;     // size of graph partitions along each graph's axis (i.e. X and Y axes)


/********************************************************************/
/*                                                                  */
/*      Copy control features implementation for graph's nodes      */
/*                                                                  */
/********************************************************************/

// default constructor
Node::Node () {
    this->neighbor = std::make_unique<link_weight_umap>();  // allocate memory for hash map containing links of the node
};

// copy constructor
Node::Node (const Node& other) {
    this->id = other.id;
    this->x = other.x;
    this->y = other.y;
    this->neighbor = std::make_unique<link_weight_umap>(other.get_links());
}

// move constructor
Node::Node (Node&& other) noexcept {
    this->id = other.id;
    this->x = other.x;
    this->y = other.y;
    this->neighbor = std::make_unique<link_weight_umap>(other.get_links());

    // leave the temporary object in a reliable state
    other.neighbor.reset();
    other.neighbor = nullptr;
};
// copy assignment
Node& Node::operator= (const Node& other) {
    this->id = other.id;
    this->x = other.x;
    this->y = other.y;
    this->neighbor = std::make_unique<link_weight_umap>(other.get_links());

    return *this;
}
// move assignment
Node& Node::operator= (Node&& other) noexcept {
    if (this == &other)
        return *this;

    this->id = other.id;
    this->x = other.x;
    this->y = other.y;
    this->neighbor = std::make_unique<link_weight_umap>(other.get_links());

    // leave the temporary object in a reliable state
    other.neighbor.reset();
    other.neighbor = nullptr;

    return *this;
}
// destructor
Node::~Node () {
    this->neighbor.reset();
    this->neighbor = nullptr;
};


/**************************************/
/*                                    */
/*      Nodes operators overload      */
/*                                    */
/**************************************/

bool Node::operator==(const Node& rhs) const {
    return (this->x == rhs.x) && (this->y == rhs.y);
}

bool Node::operator!=(const Node& rhs) const {
    return !(*this == rhs);
}

bool Node::operator<(const Node& rhs) const {
    return this->id < rhs.id;
}

bool Node::operator>(const Node& rhs) const {
    return this->id > rhs.id;
}

std::ostream& operator<<(std::ostream& os, const Node& node) {
    os << "ID = " << node.get_id() << ", x = " << node.get_x() << ", y = " << node.get_y() << std::endl;
    os << "Neighbors:" << std::endl;
    for (auto& link : node.get_links()) {
        os << "ID = " << link.first << ", weight = " << link.second << std::endl;
    }
    os << std::endl;

    return os;
}


/********************************/
/*                              */
/*      Getters and setters     */
/*                              */
/********************************/

unsigned int Node::get_id() const {
    return this->id;
}

int Node::get_x() const {
    return this->x;
}

int Node::get_y() const {
    return this->y;
}

link_weight_umap Node::get_links() const {
    return *(this->neighbor);
}

unsigned int Node::get_link_weight(const unsigned int& node_id) const {
    return (*(this->neighbor))[node_id];
}

void Node::set_id(const unsigned int& id) {
    this->id = id;
}

void Node::set_x(const int& x) {
    this->x = x;
}

void Node::set_y(const int& y) {
    this->y = y;
}

void Node::set_links(const link_weight_umap& umap) {
    this->neighbor = std::make_unique<link_weight_umap>(umap);
}

void Node::set_link(const unsigned int& node_id, const unsigned int& weight) {
    (*(this->neighbor))[node_id] = weight;
}



/****************************************************************************************************/
/*                                                                                                  */
/*      Implementation of functions for graph generation, storing on disk and loading from disk     */
/*                                                                                                  */
/****************************************************************************************************/

int graph_generation(unsigned int num_nodes,
                    unsigned int num_threads,
                    std::vector<Node>& nodes_v,
                    char *path_file_graph) {

    /* NOTE: this function gets called when we already checked that
    num_nodes > 0, num_threads > 0 and path_file_graph != NULL */

    // declaration of variables
    std::vector<std::thread> threads_v;
    std::vector<ptr_to_atomic_flag> threads_synch;
    std::vector<unsigned int> links_gen_per_partit; // number of links that has been generated for each graph partition
    std::vector<unsigned int> cum_links_gen_per_partit; // cumulative sum of the number of links that has been generated for each graph partition

    // set random seed for main thread
    srand(time(NULL));

    num_graph_partit = pow(graph_axes_partit, 2);
    if (num_graph_partit > num_nodes)
        num_graph_partit = num_nodes;

    try {
        // set size of nodes vector
        nodes_v.resize(num_nodes);

        // set size of vector containing the number of generated links per graph partition and their cumulative sum
        links_gen_per_partit.resize(num_graph_partit, 0);
        cum_links_gen_per_partit.resize(num_graph_partit, 0);
    }
    catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    // set the initial number of links in the graph
    tot_num_links_graph = 0;
    
    // no error occurred yet
    error_occurred = false;
    
    // initialize the synchronization primitive (to synchronize threads' access to critical sections)
    init_threads_synch(num_graph_partit, threads_synch);

    // initialize barrier synchronization for threads
    barrier_1 = (sem_t *) malloc(sizeof(sem_t));
    barrier_2 = (sem_t *) malloc(sizeof(sem_t));
    mb = (pthread_mutex_t *) malloc(sizeof(pthread_mutex_t));
    sem_init(barrier_1, 0, 0);
    sem_init(barrier_2, 0, 0);
    pthread_mutex_init(mb, NULL);
    barrier_counter = 0;

    // allocate memory for TIDs of threads
    tid_v = (pthread_t *) malloc(num_threads * sizeof(pthread_t));
    
    // generate threads that will build the graph and store it on disk
    for (unsigned int thread_id = 0; thread_id < num_threads; ++thread_id) {
        threads_v.emplace_back(&nodes_links_generation, thread_id, num_nodes, num_threads, std::ref(nodes_v),
                                std::ref(threads_synch), std::ref(links_gen_per_partit), std::ref(cum_links_gen_per_partit), std::ref(path_file_graph), rand());
    }
    // wait for the completion of all threads
    for (std::thread& t : threads_v) {
        t.join();
    }

    // free memory allocated dynamically
    free(barrier_1);
    free(barrier_2);
    free(mb);
    free(tid_v);
    barrier_1 = NULL;
    barrier_2 = NULL;
    mb = NULL;
    tid_v = NULL;

    // check if any error occurred
    if (error_occurred)
        return -1;

    return 0;
}


void nodes_links_generation(unsigned int thread_id,
                            unsigned int num_nodes,
                            unsigned int num_threads,
                            std::vector<Node>& nodes_v,
                            std::vector<ptr_to_atomic_flag>& threads_synch,
                            std::vector<unsigned int>& links_gen_per_partit,
                            std::vector<unsigned int>& cum_links_gen_per_partit,
                            char *path_file_graph,
                            int rand_seed) {

    /* NOTE: this function gets called when we already checked that
    num_nodes > 0, num_threads > 0 and path_file_graph != NULL */

    // set random seed for i-th thread
    srand(rand_seed);

    // store TID of i-th thread
    tid_v[thread_id] = pthread_self();

    // ensure that, if necessary, each thread can be aborted by every other thread
    signal(SIGUSR1, abort_current_thread);

    // declare variables for nodes generation
    int x, y;
    coordinate coord;
    int lb_x_graph, ub_x_graph;
    int lb_y_graph, ub_y_graph;
    unsigned int coord_span_per_partit;
    std::unordered_map<coordinate, unsigned int, hash_pair_t> coord_umap;

    // declare variables for links generation
    unsigned int generated_links;       // number of links generated by the current thread
    unsigned int idx;       // general purpose index
    unsigned int nodes_per_partition;   // number of nodes per graph partition
    unsigned int num_outgoing_links;
    unsigned int gen_links_count_curr_thread;   // total number of links generated by the current thread
    float nodes_distance;   // distance between 2 given nodes
    std::vector<int> adjacent_partit;  // contains the IDs of graph partitions that are adjacent to the currently considered partition

    // declare variables for both nodes and links generation
    unsigned int lb_idx, ub_idx;
    unsigned int lb_idx_adj_partit, ub_idx_adj_partit;
    unsigned int prev_umap_size;
    unsigned int gen_nodes_count_curr_partit;

    // set the max coordinates span length of each graph partition
    coord_span_per_partit = (unsigned int) ((max_graph_coord_span - 1) / graph_axes_partit);
    coord_span_per_partit += (coord_span_per_partit == 0) ? 1 : 0;

    // number of nodes per partition (except for the last partition, in which there could be less nodes)
    nodes_per_partition = (unsigned int) ((num_nodes - 1) / num_graph_partit) + 1;

    // set correct size of adjacent partition IDs
    try {
        adjacent_partit.resize(MAX_ADJ_PARTIT_IN_2D_GRAPH);
    }
    catch (std::exception& e) {
        // ensure that only one thread executes the "abort all threads" procedure
        std::unique_lock<std::mutex> lock{m_abort};

        // notify that an exception was raised and abort all concurrent threads
        std::cout << "Exception raised: " << e.what() << std::endl;
        abort_all_threads(thread_id, num_threads, error_occurred);
    }


    // wait for all running threads to finish the setup for graph generation and to insert their TID in the array "tid_v"
    pthread_mutex_lock(mb);
    ++barrier_counter;
    if (barrier_counter == num_threads) {

        // reset barrier counter (for next barrier)
        barrier_counter = 0;

        // release barrier for all threads
        for (unsigned int i = 0; i < num_threads; ++i) {
            sem_post(barrier_1);
        }
    }
    pthread_mutex_unlock(mb);
    sem_wait(barrier_1);



    /************************************/
    /*                                  */
    /*      Graph nodes generation      */
    /*                                  */
    /************************************/

    /* each thread tries to concurrently access every partition (each partition will be taken by only one thread), and the nodes inside each partition
    get generated by the thread that acquires the partition */
    for (unsigned int i = 0; i < num_graph_partit; i++) {
        
        // get synchronization variables of i-th partition
        ptr_to_atomic_flag& af_ptr = threads_synch[i];

        // if i-th partition has been already taken by another thread, then go to next partition
        if ((*af_ptr).test_and_set())
            continue;


        // set lower and upper bound of node IDs to be generated in i-th graph partition
        lb_idx = i * nodes_per_partition;
        ub_idx = lb_idx + nodes_per_partition - 1;
        if (ub_idx > num_nodes - 1)     // make sure to not exceed the last graph node's index
            ub_idx = num_nodes - 1;

        // ensure that we don't exceed graph's size
        if (ub_idx < lb_idx)
            continue;

        // set x lower and upper bound of i-th graph partition
        lb_x_graph = - (int) (max_graph_coord_span / 2) - 1 + (i % graph_axes_partit) * coord_span_per_partit;
        ub_x_graph = lb_x_graph + coord_span_per_partit - 1;
        if (ub_x_graph > (int) (max_graph_coord_span / 2))     // make sure to not exceed the graph size
            ub_x_graph = (int) (max_graph_coord_span / 2);

        // set y lower and upper bound of i-th graph partition
        lb_y_graph = - (int) (max_graph_coord_span / 2) - 1 + ((unsigned int)(i / graph_axes_partit)) * coord_span_per_partit;
        ub_y_graph = lb_y_graph + coord_span_per_partit - 1;
        if (ub_y_graph > (int) (max_graph_coord_span / 2))     // make sure to not exceed the graph size
            ub_y_graph = (int) (max_graph_coord_span / 2);

        // reset the counter of nodes generated for the current graph partition
        gen_nodes_count_curr_partit = 0;

        /* generate random nodes within the boudaries of the i-th partition */
        while (gen_nodes_count_curr_partit <= ub_idx - lb_idx) {
        
            try {
                // generate a pair of random x-y coordinates
                x = random_in_range<int>(lb_x_graph, ub_x_graph);
                y = random_in_range<int>(lb_y_graph, ub_y_graph);
            }
            catch (std::exception& e) {
                // ensure that only one thread executes the "abort all threads" procedure
                std::unique_lock<std::mutex> lock{m_abort};

                // notify that an exception was raised and abort all concurrent threads
                std::cout << "Exception raised: " << e.what() << std::endl;
                abort_all_threads(thread_id, num_threads, error_occurred);
            }

            // save the just generated x-y pair
            coord = std::make_pair(x, y);            
            coord_umap[coord] = gen_nodes_count_curr_partit;

            /* if no previously generated node has the same x-y coordinates of the just generated one
            then add this new node to the graph, otherwise discard it and generate a new x-y pair. */
            if (coord_umap.size() > gen_nodes_count_curr_partit) {

                Node& node = nodes_v[lb_idx + gen_nodes_count_curr_partit];
                node.set_id(lb_idx + gen_nodes_count_curr_partit);
                node.set_x(x);
                node.set_y(y);

                ++gen_nodes_count_curr_partit;
            }
        }
    }



    // wait for all running threads to finish generating nodes
    pthread_mutex_lock(mb);
    ++barrier_counter;
    if (barrier_counter == num_threads) {

        // allocate memory for the umap containing the links of the i-th node
        for (Node& node : nodes_v) {
            link_weight_umap umap;      // empty hash map that will contain all links of the i-th node
            node.set_links(umap);
            // here "umap" gets automatically deleted
        }

        // notify the user that all graph's nodes have been generated
        std::cout << "Graph nodes generated." << std::endl;

        // reset barrier counter (for next barrier)
        barrier_counter = 0;

        // reset synch variables
        for (unsigned int i = 0; i < num_graph_partit; i++) {
            ptr_to_atomic_flag& af_ptr = threads_synch[i];
            (*af_ptr).clear();
        }

        // release barrier for all threads
        for (unsigned int i = 0; i < num_threads; ++i) {
            sem_post(barrier_2);
        }
    }
    pthread_mutex_unlock(mb);
    sem_wait(barrier_2);


    /************************************/
    /*                                  */
    /*      Graph links generation      */
    /*                                  */
    /************************************/

    // reset the counter of links generated by the current thread
    gen_links_count_curr_thread = 0;

    /* each thread tries to concurrently access every partition (each partition will be taken by only one thread), and the links inside each partition
    get generated by the thread that acquires the partition */
    for (unsigned int i = 0; i < num_graph_partit; i++) {
        
        // get synchronization variables of i-th partition
        ptr_to_atomic_flag& af_ptr = threads_synch[i];

        // if i-th partition has been already taken by another thread, then go to next partition
        if ((*af_ptr).test_and_set())
            continue;


        // set lower and upper bound of node IDs to be generated in i-th graph partition
        lb_idx = i * nodes_per_partition;
        ub_idx = lb_idx + nodes_per_partition - 1;
        if (ub_idx > num_nodes - 1)     // make sure to not exceed the last graph node's index
            ub_idx = num_nodes - 1;
        
        // ensure that we don't exceed graph's size
        if (ub_idx < lb_idx)
            continue;

        // at the beginning there are no nodes with incoming links within the current graph partition
        std::vector<bool> node_has_incoming_link(ub_idx - lb_idx + 1, false);


        /******************** Generate links ********************/

        // reset the counter of generated links coming from the current graph partition
        generated_links = 0;

        /* 1st phase: generate links within the current graph partition (if there are at least 2 nodes inside it) */
        if (ub_idx - lb_idx > 0) {

            for (unsigned int node_idx = lb_idx; node_idx <= ub_idx; ++node_idx) {
                
                // get the reference to the current node
                Node& node = nodes_v[node_idx];

                /* generate the links */
                try {

                    /* add an incoming link to the current node (at least one link to guarantee that the overall graph will be strongly connected) */
                    if (!node_has_incoming_link[node_idx - lb_idx]) {

                        // avoid self loops in the graph
                        do {
                            idx = random_in_range<int>(lb_idx, ub_idx);
                        } while (idx == node_idx);
                        
                        // distance between current node and the node to which we are creating the link
                        nodes_distance = distance_between_nodes(node, nodes_v[idx]);

                        // generate the link (with proper weight)
                        nodes_v[idx].set_link(node_idx, (unsigned int) (nodes_distance * random_in_range<float>(MIN_COEFF_WEIGHT_LINK, MAX_COEFF_WEIGHT_LINK)) + 1);

                        // notify that the current node now has at least one incoming link
                        node_has_incoming_link[node_idx - lb_idx] = true;

                        // update the counter of generated links
                        ++generated_links;
                    }


                    /* set the number of outgoing links to generate from current node (at least one link to guarantee that
                    the overall graph will be strongly connected) */
                    num_outgoing_links = 1 + random_in_range<unsigned int>(0, (unsigned int)log2(ub_idx - lb_idx));

                    /* add outgoing links from the current node */
                    for (unsigned int links_count = 0; links_count < num_outgoing_links; ++links_count) {

                        do {
                            // avoid self loops in the graph
                            do {
                                idx = random_in_range<int>(lb_idx, ub_idx);
                            } while (idx == node_idx);
                            
                            // distance between current node and the node to which we are creating the link
                            nodes_distance = distance_between_nodes(node, nodes_v[idx]);

                            // generate the link (with proper weight) ensuring that it doesn't already exist
                            prev_umap_size = node.get_links().size();
                            if (prev_umap_size >= ub_idx - lb_idx)   // if we already reached the max number of outgoing links for the current node then exit the loop
                                break;
                            
                            // set the link
                            node.set_link(idx, (unsigned int) (nodes_distance * random_in_range<float>(MIN_COEFF_WEIGHT_LINK, MAX_COEFF_WEIGHT_LINK)) + 1);
                        
                        } while (node.get_links().size() == prev_umap_size);    // if a link already existed between the two selected nodes then repeat the process

                        // if we actually added a new outgoing link from the current node then ...
                        if (node.get_links().size() > prev_umap_size) {
                            
                            // notify the existence of a new incoming link (coming from the current node)
                            node_has_incoming_link[idx - lb_idx] = true;

                            // update the counter of generated links
                            ++generated_links;
                        }
                    }
                    
                }
                catch (std::exception& e) {

                    // ensure that only one thread executes the "abort all threads" procedure
                    std::unique_lock<std::mutex> lock{m_abort};

                    // notify that an exception was raised and abort all concurrent threads
                    std::cout << "Exception raised: " << e.what() << std::endl;
                    abort_all_threads(thread_id, num_threads, error_occurred);
                }
            }
        }
        

        /* 2nd phase: generate links that connect the current graph partition with all adjacent partitions */

        // get IDs of adjacent partitions
        adjacent_partit[0] = ((int) (i + graph_axes_partit) >= (int) num_graph_partit) ? UNDEFINED_PARTIT : (int) i + graph_axes_partit;  // top partition
        adjacent_partit[1] = ((int) (i + 1) % graph_axes_partit == 0) ? UNDEFINED_PARTIT : (int) (i + 1);           // right partition
        adjacent_partit[2] = ((int) i - graph_axes_partit < 0) ? UNDEFINED_PARTIT : (int) i - graph_axes_partit;    // bottom partition
        adjacent_partit[3] = ((int) (i + graph_axes_partit) % graph_axes_partit == 0) ? UNDEFINED_PARTIT : (int) (i - 1);   // left partition


        // generate links among adjacent partitions
        for (unsigned int partit_count = 0; partit_count < MAX_ADJ_PARTIT_IN_2D_GRAPH; ++partit_count) {
            
            // if the currently considered partition doesn't actually exist then go to the next iteration
            if (adjacent_partit[partit_count] == UNDEFINED_PARTIT)
                continue;
            
            // set lower and upper bound of node IDs of adjacent partition
            lb_idx_adj_partit = adjacent_partit[partit_count] * nodes_per_partition;
            ub_idx_adj_partit = lb_idx_adj_partit + nodes_per_partition - 1;
            if (ub_idx_adj_partit > num_nodes - 1)     // make sure to not exceed the last graph node's index
                ub_idx_adj_partit = num_nodes - 1;

            // ensure that we don't exceed graph's size
            if (ub_idx < lb_idx || ub_idx_adj_partit < lb_idx_adj_partit)
                continue;

            // set number of outgoing links from current partition to adjacent partition
            num_outgoing_links = (unsigned int) sqrt(ub_idx_adj_partit - lb_idx_adj_partit + 1);

            // generate links to adjacent partition
            try {
                for (unsigned int links_count = 0; links_count < num_outgoing_links; ++links_count) {

                    do {
                        // choose randomly the source and destination nodes of the link
                        idx = random_in_range<int>(lb_idx, ub_idx);
                        unsigned int node_idx_adj_partit = random_in_range<int>(lb_idx_adj_partit, ub_idx_adj_partit);
                        
                        // distance between the two selected nodes
                        nodes_distance = distance_between_nodes(nodes_v[idx], nodes_v[node_idx_adj_partit]);

                        // generate the link (with proper weight) ensuring that it doesn't already exist
                        prev_umap_size = nodes_v[idx].get_links().size();
                        nodes_v[idx].set_link(node_idx_adj_partit, (unsigned int) (nodes_distance * random_in_range<float>(MIN_COEFF_WEIGHT_LINK, MAX_COEFF_WEIGHT_LINK)) + 1);
                    
                    } while (nodes_v[idx].get_links().size() == prev_umap_size);    // if a link already existed between the two selected nodes then repeat the process
                
                    // update the counter of generated links
                    ++generated_links;
                }
            }
            catch (std::exception& e) {

                // ensure that only one thread executes the "abort all threads" procedure
                std::unique_lock<std::mutex> lock{m_abort};

                // notify that an exception was raised and abort all concurrent threads
                std::cout << "Exception raised: " << e.what() << std::endl;
                abort_all_threads(thread_id, num_threads, error_occurred);
            }
        }

        // update the overall number of generated links
        gen_links_count_curr_thread += generated_links;
        links_gen_per_partit[i] += generated_links;
    }


    /********************************************************/
    /*                                                      */
    /*      Store on disk the number of nodes and links     */
    /*                                                      */
    /********************************************************/

    int fd_graph;   // file descriptor of the file where graph has to be stored
    int nW;         // number of written characters (useful for correctness double-checks)
    struct flock flk;   // structure for file locking
    unsigned int flk_start, flk_len;    // starting point and length of the locked file region, respectively

    // wait for all running threads to finish generating links
    pthread_mutex_lock(mb);
    ++barrier_counter;
    tot_num_links_graph += gen_links_count_curr_thread;

    // open the file where the graph will be stored
    if ((fd_graph = open(path_file_graph, O_WRONLY | O_CREAT | O_TRUNC, 0666)) == -1) {
        // ensure that only one thread executes the "abort all threads" procedure
        std::unique_lock<std::mutex> lock{m_abort};

        // notify that an exception was raised and abort all concurrent threads
        std::cerr << "Thread " << thread_id << " failed to open the file " << path_file_graph << std::endl;
        abort_all_threads(thread_id, num_threads, error_occurred);
    }

    if (barrier_counter == num_threads) {

        // notify the user that all graph's links have been generated
        std::cout << "Graph links generated." << std::endl << std::endl;

        // compute the cumulative sums of links for all graph partitions
        cum_links_gen_per_partit[0] = links_gen_per_partit[0];              // NOTE: we are sure that the size of "links_gen_per_partit" is > 0
        for (unsigned int i = 1; i < links_gen_per_partit.size(); ++i)
            cum_links_gen_per_partit[i] = cum_links_gen_per_partit[i-1] + links_gen_per_partit[i];

        // store on disk the number of overall nodes and links that have been generated
        nW = write(fd_graph, &num_nodes, sizeof(unsigned int));
        nW += write(fd_graph, &tot_num_links_graph, sizeof(unsigned int));

        // store on disk the number of graph partitions and the number of links for each partition
        nW += write(fd_graph, &num_graph_partit, sizeof(unsigned int));
        for (unsigned int i = 0; i < num_graph_partit; ++i)
            nW += write(fd_graph, &links_gen_per_partit[i], sizeof(unsigned int));

        // check that we actually wrote to the file the correct number of characters
        if (nW != (int) (3 * sizeof(unsigned int) + links_gen_per_partit.size() * sizeof(unsigned int))) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to write to the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

        // notify the user about the number of generated nodes and links
        std::cout << "Number of generated nodes: " << num_nodes << std::endl;
        std::cout << "Number of generated links: " << tot_num_links_graph << std::endl << std::endl;

        // reset synch variables
        for (unsigned int i = 0; i < num_graph_partit; i++) {
            ptr_to_atomic_flag& af_ptr = threads_synch[i];
            (*af_ptr).clear();
        }

        // reset barrier counter (for next barrier)
        barrier_counter = 0;

        // release barrier for all threads
        for (unsigned int i = 0; i < num_threads; ++i) {
            sem_post(barrier_1);
        }
    }
    pthread_mutex_unlock(mb);
    sem_wait(barrier_1);


    /****************************************/
    /*                                      */
    /*      Store graph's nodes on disk     */
    /*                                      */
    /****************************************/

    // notify the user that the graph storage on file has begun
    if (thread_id == 0)
        std::cout << "Graph storage started..." << std::endl;

    unsigned int node_id;   // ID of a graph's node

    /* storage of nodes on disk */
    for (unsigned int i = 0; i < num_graph_partit; i++) {
        
        // get synchronization variables of i-th partition
        ptr_to_atomic_flag& af_ptr = threads_synch[i];

        // if i-th partition has been already taken by another thread, then go to next partition
        if ((*af_ptr).test_and_set())
            continue;

        // choose proper indexes of nodes of which the current thread has to generate the links
        lb_idx = i * nodes_per_partition;
        ub_idx = lb_idx + nodes_per_partition - 1;
        if (ub_idx > num_nodes - 1)     // make sure to not exceed the last graph node's index
            ub_idx = num_nodes - 1;

        // if i-th partition doesn't contain any node then go to next partition
        if (lb_idx > ub_idx)
            continue;

        
        /* setup the flock struct in order to lock the partition in write mode */

        // take into account the initial bytes containing informations regarding the number of nodes and links for each graph partition
        flk_start = 3 * sizeof(unsigned int) + links_gen_per_partit.size() * sizeof(unsigned int);
        
        // each node has one ID (unsigned int) and two coordinates (int)
        flk_start += lb_idx * (sizeof(unsigned int) + 2 * sizeof(int));
        flk_len = (ub_idx - lb_idx + 1) * (sizeof(unsigned int) + 2 * sizeof(int));

        // setup the lock in write mode
        flk.l_type = F_WRLCK;
        flk.l_whence = SEEK_SET;
        flk.l_start = flk_start;
        flk.l_len = flk_len;

        // lock the partition
        if (fcntl(fd_graph, F_SETLK, &flk) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to lock a partition of the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

        // move to the beginning of the locked file's partition
        if (lseek(fd_graph, flk_start, SEEK_SET) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to move the pointer to the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

        /* store nodes of i-th partition */
        for (unsigned int node_idx = lb_idx; node_idx <= ub_idx; ++node_idx) {

            /* get node's information */
            Node& node = nodes_v[node_idx];

            node_id = node.get_id();
            x = node.get_x();
            y = node.get_y();

            /* write node's information into the file */
            nW = write(fd_graph, &node_id, sizeof(unsigned int));
            nW += write(fd_graph, &x, sizeof(int));
            nW += write(fd_graph, &y, sizeof(int));

            // check the correctness of the number of written characters
            if (nW != sizeof(unsigned int) + 2 * sizeof(int)) {
                // ensure that only one thread executes the "abort all threads" procedure
                std::unique_lock<std::mutex> lock{m_abort};

                // notify that an exception was raised and abort all concurrent threads
                std::cerr << "Thread " << thread_id << " failed to write to the file " << path_file_graph << std::endl;
                abort_all_threads(thread_id, num_threads, error_occurred);
            }
        }

        // unlock the file's partition
        flk.l_type = F_UNLCK;
        if (fcntl(fd_graph, F_SETLK, &flk) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to unlock a partition of the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }
    }

    // wait for all running threads to finish storing the nodes
    pthread_mutex_lock(mb);
    ++barrier_counter;
    if (barrier_counter == num_threads) {
        
        // reset synch variables
        for (unsigned int i = 0; i < num_graph_partit; i++) {
            ptr_to_atomic_flag& af_ptr = threads_synch[i];
            (*af_ptr).clear();
        }

        // notify the user that all graph's nodes have been stored on disk
        std::cout << "Graph nodes stored in the file." << std::endl;

        // reset barrier counter (for next barrier)
        barrier_counter = 0;

        // release barrier for all threads
        for (unsigned int i = 0; i < num_threads; ++i) {
            sem_post(barrier_2);
        }
    }
    pthread_mutex_unlock(mb);
    sem_wait(barrier_2);


    /****************************************/
    /*                                      */
    /*      Store graph's links on disk     */
    /*                                      */
    /****************************************/

    /* storage of links on disk */
    for (unsigned int i = 0; i < num_graph_partit; i++) {

        // in case the generated graph contains no links, skip this section
        if (tot_num_links_graph == 0)
            break;
        
        // get synchronization variables of i-th partition
        ptr_to_atomic_flag& af_ptr = threads_synch[i];

        // if i-th partition has been already taken by another thread, then go to next partition
        if ((*af_ptr).test_and_set())
            continue;

        // choose proper indexes of nodes of which the current thread has to generate the links
        lb_idx = i * nodes_per_partition;
        ub_idx = lb_idx + nodes_per_partition - 1;
        if (ub_idx > num_nodes - 1)     // make sure to not exceed the last graph node's index
            ub_idx = num_nodes - 1;
        
        // if i-th partition doesn't contain any node then go to next partition
        if (lb_idx > ub_idx)
            continue;

        
        /* setup the flock struct in order to lock the partition in write mode */

        // take into account the initial bytes containing informations regarding the number of nodes and links for each graph partition
        flk_start = 3 * sizeof(unsigned int) + links_gen_per_partit.size() * sizeof(unsigned int);

        // take into account the nodes of the graph previously written in the file
        flk_start += num_nodes * (sizeof(unsigned int) + 2 * sizeof(int));
        
        // take into accout the size of all previous links
        if (i > 0)
            flk_start += cum_links_gen_per_partit[i-1] * (3 * sizeof(unsigned int));

        // each link has one source ID (unsigned int), one destination ID (unsigned int) and one weight (unsigned int)
        flk_len = links_gen_per_partit[i] * (3 * sizeof(unsigned int));

        // setup the lock in write mode
        flk.l_type = F_WRLCK;
        flk.l_whence = SEEK_SET;
        flk.l_start = flk_start;
        flk.l_len = flk_len;

        // lock the partition
        if (fcntl(fd_graph, F_SETLK, &flk) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to lock a partition of the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

        // move to the beginning of the locked file's partition
        if (lseek(fd_graph, flk_start, SEEK_SET) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to move the pointer to the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

        /* store links of i-th partition */
        for (unsigned int node_idx = lb_idx; node_idx <= ub_idx; ++node_idx) {

            // get the j-th node of the i-th partition
            Node& node = nodes_v[node_idx];
            
            /* get all the information about the links of the j-th node and write it into the file */
            for (auto& link : node.get_links()) {
                nW = write(fd_graph, &node_idx, sizeof(unsigned int));
                nW += write(fd_graph, &link.first, sizeof(unsigned int));
                nW += write(fd_graph, &link.second, sizeof(unsigned int));

                // check the correctness of the number of written characters
                if (nW != 3 * sizeof(unsigned int)) {
                    // ensure that only one thread executes the "abort all threads" procedure
                    std::unique_lock<std::mutex> lock{m_abort};

                    // notify that an exception was raised and abort all concurrent threads
                    std::cerr << "Thread " << thread_id << " failed to write to the file " << path_file_graph << std::endl;
                    abort_all_threads(thread_id, num_threads, error_occurred);
                }
            }

        }

        // unlock the file's partition
        flk.l_type = F_UNLCK;
        if (fcntl(fd_graph, F_SETLK, &flk) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to unlock a partition of the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

    }

    // close the file that stores the graph
    close(fd_graph);

    // notify the user that all graph's links have been stored on disk
    if (thread_id == 0)
        std::cout << "Graph links stored in the file." << std::endl << std::endl;
}


int graph_read_from_file(unsigned int num_threads,
                        std::vector<Node>& nodes_v,
                        char *path_file_graph) {

    /* NOTE: this function gets called when we already checked that
    num_threads > 0 and path_file_graph != NULL */

    // declaration of variables
    int fd_graph;   // file descriptor of the file where graph has to be stored
    int nR;         // number of written characters (useful for correctness double-checks)
    unsigned int num_nodes;     // number of graph nodes
    unsigned int num_links;     // number of graph links
    std::vector<unsigned int> links_gen_per_partit;     // number of links generated in each partition
    std::vector<unsigned int> cum_links_gen_per_partit; // cumulative sum of the number of links generated in each partition
    std::vector<std::thread> threads_v;     // threads pool
    std::vector<ptr_to_atomic_flag> threads_synch;  // structure for threads synchronization

    // open the file containing the graph
    if ((fd_graph = open(path_file_graph, O_RDONLY)) == -1) {
        std::cerr << "Main thread failed to open the file " << path_file_graph << std::endl;
        return -1;
    }

    // get overall number of nodes, links and partitions contained in the graph
    nR = read(fd_graph, &num_nodes, sizeof(unsigned int));
    nR += read(fd_graph, &num_links, sizeof(unsigned int));
    nR += read(fd_graph, &num_graph_partit, sizeof(unsigned int));

    // check for read errors
    if (nR != 3 * sizeof(unsigned int)) {
        std::cerr << "Main thread failed to read from file " << path_file_graph << std::endl;
        return -1;
    }

    try {
        // set correct size of nodes vector
        nodes_v.resize(num_nodes);

        // set size of vector containing the number of links per graph partition and their cumulative sum
        links_gen_per_partit.resize(num_graph_partit, 0);
        cum_links_gen_per_partit.resize(num_graph_partit, 0);
    }
    catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    // get number of links of each graph partition
    nR = read(fd_graph, &links_gen_per_partit[0], sizeof(unsigned int));    // NOTE: we are sure that the size of "links_gen_per_partit" is > 0
    for (unsigned int i = 1; i < num_graph_partit; ++i)
        nR += read(fd_graph, &links_gen_per_partit[i], sizeof(unsigned int));
    
    // check for read errors
    if (nR != (int) (num_graph_partit * sizeof(unsigned int))) {
        std::cerr << "Main thread failed to read from file " << path_file_graph << std::endl;
        return -1;
    }


    // compute the cumulative sums of links for all graph partitions
    cum_links_gen_per_partit[0] = links_gen_per_partit[0];              // NOTE: we are sure that the size of "links_gen_per_partit" is > 0
    for (unsigned int i = 1; i < links_gen_per_partit.size(); ++i)
        cum_links_gen_per_partit[i] = cum_links_gen_per_partit[i-1] + links_gen_per_partit[i];


    // no error occurred yet
    error_occurred = false;

    // initialize thread synchronization primitives
    init_threads_synch(num_graph_partit, threads_synch);

    // initialize barrier synchronization for threads
    barrier_1 = (sem_t *) malloc(sizeof(sem_t));
    barrier_2 = (sem_t *) malloc(sizeof(sem_t));
    mb = (pthread_mutex_t *) malloc(sizeof(pthread_mutex_t));
    sem_init(barrier_1, 0, 0);
    sem_init(barrier_2, 0, 0);
    pthread_mutex_init(mb, NULL);
    barrier_counter = 0;

    // allocate memory for TIDs of threads
    tid_v = (pthread_t *) malloc(num_threads * sizeof(pthread_t));

    // generate threads that will build the graph and store it on disk
    for (unsigned int thread_id = 0; thread_id < num_threads; ++thread_id) {
        threads_v.emplace_back(&nodes_links_read_from_file, thread_id, num_nodes, num_links, num_threads, std::ref(nodes_v),
                                std::ref(threads_synch), std::ref(links_gen_per_partit), std::ref(cum_links_gen_per_partit), std::ref(path_file_graph));
    }
    // wait for the completion of all threads
    for (std::thread& t : threads_v) {
        t.join();
    }

    // free memory allocated dynamically
    free(barrier_1);
    free(barrier_2);
    free(mb);
    free(tid_v);
    barrier_1 = NULL;
    barrier_2 = NULL;
    mb = NULL;
    tid_v = NULL;

    if (error_occurred)
        return -1;

    return 0;
}


void nodes_links_read_from_file(unsigned int thread_id,
                                unsigned int num_nodes,
                                unsigned int num_links,
                                unsigned int num_threads,
                                std::vector<Node>& nodes_v,
                                std::vector<ptr_to_atomic_flag>& threads_synch,
                                std::vector<unsigned int>& links_gen_per_partit,
                                std::vector<unsigned int>& cum_links_gen_per_partit,
                                char *path_file_graph) {

    /* NOTE: this function gets called when we already checked that
    num_threads > 0 and path_file_graph != NULL */
    
    // store TID of i-th thread
    tid_v[thread_id] = pthread_self();

    // ensure that, if necessary, each thread can be aborted by every other thread
    signal(SIGUSR1, abort_current_thread);

    // declaration of variables                                
    int fd_graph;   // file descriptor of the file where graph has to be stored
    int nR;         // number of written characters (useful for correctness double-checks)
    unsigned int nodes_per_partition;   // number of nodes for each partition (except for the last few partitions that could have less nodes)
    unsigned int lb_idx, ub_idx;    // lower and upper bounds for the nodes' ID of each partition
    struct flock flk;   // structure for file locking
    unsigned int flk_start, flk_len;    // starting point and length of the locked file region, respectively
    unsigned node_id, node_x, node_y;   // node IDs
    unsigned int src_node_id, dest_node_id;     // ID of the source and destination nodes for each graph link
    unsigned int link_weight;   // weight of a graph link

    // number of nodes per partition (except for the last partition, in which there could be less nodes)
    nodes_per_partition = (unsigned int) ((num_nodes - 1) / num_graph_partit) + 1;
    
    // open the file where the graph will be stored
    if ((fd_graph = open(path_file_graph, O_RDONLY)) == -1) {
        // ensure that only one thread executes the "abort all threads" procedure
        std::unique_lock<std::mutex> lock{m_abort};

        // notify that an exception was raised and abort all concurrent threads
        std::cerr << "Thread " << thread_id << " failed to open the file " << path_file_graph << std::endl;
        abort_all_threads(thread_id, num_threads, error_occurred);
    }


    // wait for all running threads to finish the setup for graph reading and to insert their TID in the array "tid_v"
    pthread_mutex_lock(mb);
    ++barrier_counter;
    if (barrier_counter == num_threads) {

        // reset barrier counter (for next barrier)
        barrier_counter = 0;

        // release barrier for all threads
        for (unsigned int i = 0; i < num_threads; ++i) {
            sem_post(barrier_1);
        }
    }
    pthread_mutex_unlock(mb);
    sem_wait(barrier_1);


    /********************************************/
    /*                                          */
    /*      Read graph's nodes from file        */
    /*                                          */
    /********************************************/

    /* read nodes from file */
    for (unsigned int i = 0; i < num_graph_partit; i++) {
        
        // get synchronization variables of i-th partition
        ptr_to_atomic_flag& af_ptr = threads_synch[i];

        // if i-th partition has been already taken by another thread, then go to next partition
        if ((*af_ptr).test_and_set())
            continue;

        // choose proper indexes of nodes of which the current thread has to generate the links
        lb_idx = i * nodes_per_partition;
        ub_idx = lb_idx + nodes_per_partition - 1;
        if (ub_idx > num_nodes - 1)     // make sure to not exceed the last graph node's index
            ub_idx = num_nodes - 1;
        
        // if i-th partition doesn't contain any node then go to next partition
        if (lb_idx > ub_idx)
            continue;
        

        /* setup the flock struct in order to lock the partition in read mode */

        // take into account the initial bytes containing informations regarding the number of nodes and links for each graph partition
        flk_start = 3 * sizeof(unsigned int) + links_gen_per_partit.size() * sizeof(unsigned int);
        
        // each node has one ID (unsigned int) and two coordinates (int)
        flk_start += lb_idx * (sizeof(unsigned int) + 2 * sizeof(int));
        flk_len = (ub_idx - lb_idx + 1) * (sizeof(unsigned int) + 2 * sizeof(int));

        // setup the lock in read mode
        flk.l_type = F_RDLCK;
        flk.l_whence = SEEK_SET;
        flk.l_start = flk_start;
        flk.l_len = flk_len;


        // lock the partition
        if (fcntl(fd_graph, F_SETLK, &flk) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to lock a partition of the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

        // move to the beginning of the locked file's partition
        if (lseek(fd_graph, flk_start, SEEK_SET) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to move the pointer to the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

        /* read nodes of i-th partition */
        for (unsigned int node_idx = lb_idx; node_idx <= ub_idx; ++node_idx) {

            // read information about the j-th node of the i-th partition
            nR = read(fd_graph, &node_id, sizeof(unsigned int));
            nR += read(fd_graph, &node_x, sizeof(int));
            nR += read(fd_graph, &node_y, sizeof(int));
                
            // check for file reading errors
            if (nR != sizeof(unsigned int) + 2 * sizeof(int)) {
                // ensure that only one thread executes the "abort all threads" procedure
                std::unique_lock<std::mutex> lock{m_abort};

                // notify that an exception was raised and abort all concurrent threads
                std::cerr << "Thread " << thread_id << " failed to read a node from file " << path_file_graph << std::endl;
                abort_all_threads(thread_id, num_threads, error_occurred);
            }

            // save the information of the retrieved node into memory
            Node& node = nodes_v[node_id];
            node.set_id(node_id);
            node.set_x(node_x);
            node.set_y(node_y);
        }

        // unlock the file's partition
        flk.l_type = F_UNLCK;
        if (fcntl(fd_graph, F_SETLK, &flk) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to unlock a partition of the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

    }

    // wait for all running threads to finish reading the nodes
    pthread_mutex_lock(mb);
    ++barrier_counter;
    if (barrier_counter == num_threads) {

        // allocate memory for the umap containing the links of the i-th node
        for (Node& node : nodes_v) {
            link_weight_umap umap;      // empty hash map that will contain all links of the i-th node
            node.set_links(umap);
            // here "umap" gets automatically deleted
        }
        
        // reset synch variables
        for (unsigned int i = 0; i < num_graph_partit; i++) {
            ptr_to_atomic_flag& af_ptr = threads_synch[i];
            (*af_ptr).clear();
        }

        // reset barrier counter (for next barrier)
        barrier_counter = 0;

        // release barrier for all threads
        for (unsigned int i = 0; i < num_threads; ++i) {
            sem_post(barrier_2);
        }
    }
    pthread_mutex_unlock(mb);
    sem_wait(barrier_2);


    /********************************************/
    /*                                          */
    /*      Read graph's links from file        */
    /*                                          */
    /********************************************/
    
    /* read links from file */
    for (unsigned int i = 0; i < num_graph_partit; i++) {

        // in case the graph contains no links, skip this section
        if (num_links == 0)
            break;
        
        // get synchronization variables of i-th partition
        ptr_to_atomic_flag& af_ptr = threads_synch[i];

        // if i-th partition has been already taken by another thread, then go to next partition
        if ((*af_ptr).test_and_set())
            continue;

        // choose proper indexes of nodes of which the current thread has to generate the links
        lb_idx = 0;
        if (i > 0)
            lb_idx += cum_links_gen_per_partit[i-1];
        ub_idx = cum_links_gen_per_partit[i] - 1;


        // if i-th partition doesn't contain any link then go to next partition
        if (lb_idx > ub_idx)
            continue;


        /* setup the flock struct in order to lock the partition in read mode */

        // take into account the initial bytes containing informations regarding the number of nodes and links for each graph partition
        flk_start = 3 * sizeof(unsigned int) + links_gen_per_partit.size() * sizeof(unsigned int);

        // take into account the nodes of the graph previously written in the file
        flk_start += num_nodes * (sizeof(unsigned int) + 2 * sizeof(int));
        
        // take into accout the size of all previous links
        if (i > 0)
            flk_start += cum_links_gen_per_partit[i-1] * (3 * sizeof(unsigned int));

        // each link has one source ID (unsigned int), one destination ID (unsigned int) and one weight (unsigned int)
        flk_len = links_gen_per_partit[i] * (3 * sizeof(unsigned int));

        // setup the lock in read mode
        flk.l_type = F_RDLCK;
        flk.l_whence = SEEK_SET;
        flk.l_start = flk_start;
        flk.l_len = flk_len;
        

        // lock the partition
        if (fcntl(fd_graph, F_SETLK, &flk) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to lock a partition of the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

        // move to the beginning of the locked file's partition
        if (lseek(fd_graph, flk_start, SEEK_SET) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to move the pointer to the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }

        /* store links of i-th partition */
        for (unsigned int link_idx = lb_idx; link_idx <= ub_idx; ++link_idx) {
            
            // read the information about the j-th link of the i-th partition
            nR = read(fd_graph, &src_node_id, sizeof(unsigned int));
            nR += read(fd_graph, &dest_node_id, sizeof(unsigned int));
            nR += read(fd_graph, &link_weight, sizeof(unsigned int));

            // check for file reading errors
            if (nR != 3 * sizeof(unsigned int)) {
                // ensure that only one thread executes the "abort all threads" procedure
                std::unique_lock<std::mutex> lock{m_abort};

                // notify that an exception was raised and abort all concurrent threads
                std::cerr << "Thread " << thread_id << " failed to read a link from file " << path_file_graph << std::endl;
                abort_all_threads(thread_id, num_threads, error_occurred);
            }

            // save the retrieved link into memory
            Node& node = nodes_v[src_node_id];
            node.set_link(dest_node_id, link_weight);
            
        }

        // unlock the file's partition
        flk.l_type = F_UNLCK;
        if (fcntl(fd_graph, F_SETLK, &flk) == -1) {
            // ensure that only one thread executes the "abort all threads" procedure
            std::unique_lock<std::mutex> lock{m_abort};

            // notify that an exception was raised and abort all concurrent threads
            std::cerr << "Thread " << thread_id << " failed to unlock a partition of the file " << path_file_graph << std::endl;
            abort_all_threads(thread_id, num_threads, error_occurred);
        }
    }

    // close the file descriptor of the file containing the graph
    close(fd_graph);
}

