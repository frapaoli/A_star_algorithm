#include "graph_gen_store_load.h"
#include "a_star.h"
#include "utilities.h"
#include "menu.h"

#include <memory>
#include <algorithm>
#include <stdio.h>
#include <tuple>
#include <climits>
#include <iostream>
#include <math.h>
#include <fcntl.h>
#include <signal.h>


/*************************************/
/*                                   */
/*    Global variables definition    */
/*                                   */
/*************************************/

unsigned int num_R_partit;  // number of partitions in which the random-bit-strings table gets divided (to allow efficient parallel operations on it)
std::mutex m_R;             // mutex used during the phase of filling up the random-bit-strings table


/*******************************************************/
/*                                                     */
/*      Implementation of A* algorithm functions       */
/*                                                     */
/*******************************************************/

void a_star_sequential (std::vector<Node>& graph, Node start, Node stop, graph_path_t& best_path) {

    /* variables definition */
    open_list open;         // list of nodes already discovered but still to be expanded
    closed_list closed;     // list of nodes already expanded
    std::unordered_map<Node, Node> from;    // used to remember from which node (parent) a thread arrived in order to explore a certain node (child)
    std::unordered_map<Node, unsigned int> cost;    // used to keep track of the cost to arrive to each node
    list_elem node_in_open_list;    // element of open list

    /* record infos about the starting node */
    open.emplace(calculate_h(start, stop), calculate_h(start, stop), start);
    cost[start] = 0;
    from[start] = start;


    /****************************/
    /*                          */
    /*    Sequential A* loop    */
    /*                          */
    /****************************/

    do {
        // expand the node in the open list that has the lower cost
        node_in_open_list = open.top();
        Node& curr = std::get<2>(node_in_open_list);
        open.pop();

        // check that the current node has NOT been already expanded. If it is, then skip to next iteration
        if (closed.find(curr.get_id()) != closed.end())
            continue;
        
        // add the expanded node to the closed list
        closed[curr.get_id()] = node_in_open_list;

        // if the current node is the "stop" node, then we arrived to destination
        if (curr == stop) {

            // rebuild the path from "stop" up to "start" node
            return rebuild_path_single_thread(best_path, from, start, stop);
        }

        // discover the neighbors of the current node and eventually add them to the open list
        for (auto link : curr.get_links()) {

            Node& n = graph[link.first];
            unsigned int curr_cost = cost[curr] + link.second;   // "new cost" = "old cost" + "weight of new link"

            // if the i-th neighbor has never been seen before or it has been seen with a worst cost w.r.t. the current one, then add it to the open list
            if (cost.find(n) == cost.end() || cost[n] > curr_cost) {
                cost[n] = curr_cost;
                from[n] = curr;
                double h = calculate_h(n, stop);
                double f = curr_cost + h;
                open.emplace(f, h, n);
            }
        }
    }
    while (!open.empty());  // loop while there are nodes to be expanded

}


int a_star_centralized (std::vector<Node>& graph, Node start, Node stop, unsigned int num_threads, graph_path_t& best_path) {
    
    /* NOTE: this function gets called when we already checked that num_threads > 0 */

    /******************** Variables definition ********************/

    /* definition of shared variables */
    open_list open;     // shared open list
    closed_list closed; // shared closed list
    std::unordered_map<Node, Node> from;    // shared structure used to remember from which node (parent) a thread arrived in order to explore a certain node (child)
    std::unordered_map<Node, unsigned int> cost;    // shared structure that relates each node N with its cost g(N) from the start node
    std::vector<int> end;   // used for the algorithm termination
    std::mutex m0, m1, m2;  // used to synchronize the threads' access to the shared structures

    // thread pool (used to join the threads created by the main thread)
    std::vector<std::thread> threadPool;


    /******************** Variables initialization ********************/

    /* record infos about the starting node */
    open.emplace(0, 0, start);
    cost[start] = 0;
    from[start] = start;
    
    // no best path has been found yet
    end.resize(num_threads, 1);
    best_path.path_cost.store(INT_MAX);


    /******************** Launch threads that will run Centralized A* algorithm ********************/

    for (unsigned int i = 0; i < num_threads; ++i) {
        threadPool.emplace_back(&a_star_centralized_thread, std::ref(graph), std::ref(stop), std::ref(start), i, std::ref(open), std::ref(closed), std::ref(from), std::ref(cost), std::ref(best_path), std::ref(end), std::ref(m0), std::ref(m1), std::ref(m2));
    }

    // wait for the completion of all threads
    for (auto& t : threadPool) {
        t.join();
    }

    return 0;
}


void a_star_centralized_thread(const std::vector<Node>& graph,
                            Node stop,
                            Node start,
                            unsigned int thread_id,
                            open_list& open,
                            closed_list& closed,
                            std::unordered_map<Node, Node>& from,
                            std::unordered_map<Node, unsigned int>& cost,
                            graph_path_t& best_path,
                            std::vector<int>& end,
                            std::mutex& m0,
                            std::mutex& m1,
                            std::mutex& m2) {
                
    /*****************************/
    /*                           */
    /*    Centralized A* loop    */
    /*                           */
    /*****************************/
    while (true) {

        /* if the open list is empty or the lowest f(N) value in it is not lower than the cost of the
        overall best path found until now (if there is any), then the algorithm can potentially terminate. */
        std::unique_lock l0{m0};
        if (open.empty() || std::get<0>(open.top()) >= best_path.path_cost.load()) {
            end[thread_id] = 0;     // set the current thread as "done"

            /* if all threads are done then quit the program, otherwise continue. */
            bool terminate = std::all_of(end.begin(), end.end(), [](int flag) { return flag == 0; });

            if (terminate) {
                break;
            }
            continue;
        }
        end[thread_id] = 1;         // reset the "end" flag of the current thread

        /* expand a node: get from open list the node N with lowest f(N) and revome it from open list. */
        list_elem top = open.top();
        open.pop();
        l0.unlock();

        Node curr = std::get<2>(top);   // retrieve the node from the list entry

        /* check that the current node has NOT been already expanded with a better cost. If it is, then skip to next iteration, otherwise expand the node. */
        std::unique_lock l2{m2};
        auto exist = closed.find(curr.get_id());
        if (exist != closed.end()) {
            int f = std::get<0>(top);
            int h = std::get<1>(top);
            int g = f - h;

            /* if we previously expanded the same node with a higher cost than expand again the node with the new (better) cost,
            otherwise go to the next iteration. */
            std::unique_lock l1{m1};
            if (g > (int) cost[curr]) {
                continue;
            }
        }
        
        // add the new node to the closed list
        closed[(curr).get_id()] = top;
        l2.unlock();

        /* if the node just expanded is the goal node and its overall cost is lower than the best solution found previously, then substitute it. */
        if (curr == stop) {
            std::unique_lock l1{m1};
            if ((int) cost[curr] < best_path.path_cost.load()) {
                rebuild_path_single_thread(best_path, from, start, stop);   // rebuild the new best path from "start" to "stop" node
            }
        }
        /* for each neighbor of the expanded node, check if it is worth to expand it and, if yes, then add it to the shared open list. */
        for (auto link : curr.get_links()) {
            
            // take information regarding the i-th neighbor of the current node
            Node n = graph.at(link.first);
            std::unique_lock l1{m1};
            double curr_cost = cost[curr] + link.second;

            // if it is worth to expand it then add its information to the shared data structures
            if (cost.find(n) == cost.end() || cost[n] > curr_cost) {

                cost[n] = curr_cost;
                from[n] = curr;
                l1.unlock();
                double h = calculate_h(n, stop);
                double f = curr_cost + h;
                std::unique_lock l0{m0};
                open.emplace(f, h, n);
            }
        }
    }
}


int a_star_decentralized (std::vector<Node>& graph, Node start, Node stop, int hash_type, unsigned int num_threads, graph_path_t& best_path) {

    /* NOTE: this function gets called when we already checked that num_threads > 0 */

    /***********************************************/
    /*                                             */
    /*      Decentralized A* initialization        */
    /*                                             */
    /***********************************************/

    /******************** Variables definition ********************/

    std::vector<std::thread> threads_v;     // vector of all thread instances that will run A*
    std::atomic<bool> found_new_best_path;  // flag to notify that A* found a new best path
    std::vector<open_list> open_list_v;     // vector of threads' open lists
    std::vector<int> R;             // random-bit-strings table (containing a random 4-bytes integer for each entry)
    std::condition_variable cv_R;   // condition variable used during the phase of filling up the random-bit-strings table
    bool R_ready;           // flag to notify that the random-bit-strings table has been filled up
    unsigned int AZ_hash_mask;      // mask used to implement the feature projection function of the Abstract Zobrist Hashing
    int start_node_owner;   // thread ID obtained by hashing the "start" node (i.e. ID of the thread that owns "start")
    int stop_node_owner;    // thread ID obtained by hashing the "stop" node (i.e. ID of the thread that owns "stop")
    std::vector<ptr_to_atomic_flag> threads_synch;  // vector of pointers to atomic flag (used for threads synchronization)
    std::vector<msg_buffer_t> msg_buffer_v;         // vector of threads' message buffers
    parent_request_buffer_t parent_request_buffer;  // buffer of requests to know the parent of a certain node
    parent_reply_t parent_reply;        // when i-th thread asks to j-th thread which node is the parent of node N, the j-th thread replies by putting parent(N) inside parent_reply
    std::vector<ptr_to_cond_var> cv_threads_v;      // vector of pointers to condition variables (used for threads synchronization)
    std::vector<ptr_to_mutex> m_msg_v;              // vector of pointers to mutexes (used for threads communication)
    std::atomic<bool> algorithm_terminated;         // flag to notify to all threads that the Decentralized A* terminated
    std::atomic<int> termination_starter_thread;    // ID of the first thread that verifies the algorithm termination condition
    acc_msg_counter_t acc_msg_counter;  // stores the cumulative counter of sent and received messages for each thread (used to detect algorithm termination)


    /******************** Variables initialization ********************/

    // we didn't find any path from "start" to "stop" node yet
    found_new_best_path.store(false);

    // initially the random-bit-strings table is not ready yet
    R_ready = false;

    // reset the algorithm termination conditions
    algorithm_terminated.store(false);
    termination_starter_thread.store(UNDEFINED_THREAD);


    /******************** Set size of Decentralized A* data structures ********************/

    try {
        // resize the vector of open lists
        open_list_v.resize(num_threads);

        // resize the random-bit-strings table
        R.resize(max_graph_coord_span + 1);

        // resize the vector of message buffers where are being sent the new graph's nodes to be potentially expanded
        msg_buffer_v.resize(num_threads);

        // resize vector of message buffers to request to another thread the parent of a node that it owns
        parent_request_buffer.resize(num_threads);
        for (unsigned int idx = 0; idx < num_threads; ++idx)
            parent_request_buffer[idx].second = false;   // initially set all parent requests as "not yet arrived"

        // initially set all parent requests as "not yet answered"
        parent_reply.second = false;

        // resize the vector of condition variables
        cv_threads_v.resize(num_threads);
        for (unsigned int cv_idx = 0; cv_idx < num_threads; ++cv_idx)
            cv_threads_v[cv_idx] = std::make_unique<std::condition_variable>();

        // resize the vectors of mutexes
        m_msg_v.resize(num_threads);
        for (unsigned int m_idx = 0; m_idx < num_threads; ++m_idx)
            m_msg_v[m_idx] = std::make_unique<std::mutex>();

        // resize and reset the cumulative counter of sent and received messages for each thread
        acc_msg_counter.first.resize(num_threads, 0);

        // set the thread with ID == 0 to be the first owner of acc_msg_counter
        acc_msg_counter.second.store(0);

        // if the chosen hashing method is the Abstract Zobrist then initialize its feature projection function
        if (hash_type == ABSTRACT_ZOBRIST_HASH) {

            unsigned int next_power_of_two;     // lowest power of 2 that is higher than the size of the random-bit-strings table
            unsigned int N;                     // power exponent
            unsigned int sub_from_mask;         // value to be subtracted to the mask 0xFFFFFFFF (i.e. mask full of ones)

            // compute the lowest power of 2 that is higher than the size of the random-bit-strings table
            next_power_of_two = R.size();
            next_power_of_two |= next_power_of_two >> 1;
            next_power_of_two |= next_power_of_two >> 2;
            next_power_of_two |= next_power_of_two >> 4;
            next_power_of_two |= next_power_of_two >> 8;
            next_power_of_two |= next_power_of_two >> 16;
            next_power_of_two++;
            
            // compute N such that 2^(2*N - 1) == next_power_of_two (where "2^(2*N - 1)" gets approximated to an integer)
            N = (unsigned int)((log2(next_power_of_two) + 1)/2.0f);
            
            // compute the value to be subtracted to the mask 0xFFFFFFFF
            sub_from_mask = pow(2, N-1) - 1;
            
            // compute Abstract Zobrist projection
            AZ_hash_mask = 0xFFFFFFFF - sub_from_mask;
        }
    }
    catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    // no error occurred yet
    error_occurred = false;

    /* set the number of partitions in which we divide the random-bit-strings table, so that, in order to fill the entire table, we assign partitions to the
    running threads and they will fill the partition */
    num_R_partit = num_threads * R_PARTIT_PER_THREAD;
    if (num_R_partit > max_graph_coord_span)
        num_R_partit = max_graph_coord_span;

    // initialize the synchronization primitives used by the threads to fill the random-bit-strings table
    init_threads_synch(num_R_partit, threads_synch);

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
    

    /***********************************************/
    /*                                             */
    /*      Decentralized A* threads launch        */
    /*                                             */
    /***********************************************/

    /******************** Launch threads that will run Decentralized A* algorithm ********************/
    
    for (unsigned int thread_id = 0; thread_id < num_threads; ++thread_id) {
        threads_v.emplace_back(&a_star_decentralized_thread, std::ref(graph), std::ref(start), std::ref(stop), std::ref(stop_node_owner), hash_type,
                                num_threads, thread_id, std::ref(open_list_v[thread_id]), std::ref(R), std::ref(R_ready), std::ref(AZ_hash_mask), std::ref(cv_R),
                                std::ref(threads_synch), std::ref(cv_threads_v), std::ref(m_msg_v), std::ref(msg_buffer_v),
                                std::ref(parent_request_buffer), std::ref(parent_reply), std::ref(best_path), std::ref(found_new_best_path), rand(),
                                std::ref(acc_msg_counter), std::ref(algorithm_terminated), std::ref(termination_starter_thread));
    }
  
    // wait for the other threads to generate the random-bit-strings table
    std::unique_lock<std::mutex> lock_R{m_R};
    while (!R_ready) {
        cv_R.wait(lock_R);
    }

    /* add 'start' to the appropriate open list (according to the hash function chosen by the user) */
    if ((start_node_owner = a_star_hash(start, num_threads, R, hash_type, AZ_hash_mask)) == UNDEFINED_THREAD ||
        (stop_node_owner = a_star_hash(stop, num_threads, R, hash_type, AZ_hash_mask)) == UNDEFINED_THREAD) {

        // if hashing the node failed then free memory allocated dynamically and return error
        free(barrier_1);
        free(barrier_2);
        free(mb);
        free(tid_v);
        barrier_1 = NULL;
        barrier_2 = NULL;
        mb = NULL;
        tid_v = NULL;

        // terminate all created threads
        for (unsigned int id = 0; id < num_threads; ++id) {
            // signal the i-th thread to make it terminate its execution
            pthread_kill(tid_v[id], SIGUSR1);
        }

        return -1;
    }

    // if hashing the node succeded then continue
    open_list_v[start_node_owner].emplace(calculate_h(start, stop), calculate_h(start, stop), start);
    lock_R.unlock();

    // release barrier for all threads
    for (unsigned int i = 0; i < num_threads; ++i) {
        sem_post(barrier_1);
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

    return 0;
}


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
                                std::atomic<int>& termination_starter_thread) {

    // set random seed for i-th thread
    srand(rand_seed);

    // store TID of i-th thread
    tid_v[thread_id] = pthread_self();

    // ensure that, if necessary, each thread can be aborted by every other thread
    signal(SIGUSR1, abort_current_thread);

    
    /******************** Thread-local variables definition ********************/

    unsigned int R_entries_per_partition;   // number of entries in each random-bit-strings table partition
    unsigned int R_lb_idx, R_ub_idx;        // lower and upper bound indexes of each random-bit-strings table partition
    std::condition_variable& cv_thread = *(cv_threads_v[thread_id]);     // alias for condition variable of current thread
    std::mutex& m_msg = *(m_msg_v[thread_id]);       // alias for message mutex of current thread
    msg_buffer_t& msg_buffer = msg_buffer_v[thread_id];      // alias for message buffer of current thread
    parent_request_t& parent_request = parent_request_buffer[thread_id];     // alias for parent request of current thread
    msg_t msg;  // message to be sent/received to/from other threads
    bool need_to_wait_for_msg;      // flag that gets set when current thread has no node that is worth expanding, hence it needs to wait for new messages 
    bool need_to_sleep;             // flag that gets set when current thread can go to sleep (avoids useless loop of the program)
    closed_list closed;             // list of nodes already expanded
    double f, g, g_child, h;        // graph costs
    std::unordered_map<Node, Node> from;    // used to remember from which node (parent) a thread arrived in order to explore a certain node (child)
    std::unordered_map<Node, unsigned int> cost;    // relates each node N with its cost g(N) from the start node
    bool need_to_set_best_path;     // flag that gets set when the current thread has to update the graph's best path from start to stop node
    int hashed_thread_id;           // thread ID obtained by hashing a node
    int best_path_cost;             // temporary variable for best path's cost
    Node n, n_child, n_parent;      // nodes
    list_elem node_in_open_list;    // element of open list
    bool node_expanded;         // flag that gets set when, in a given A* iteration, the current thread expanded a new node
    std::vector<int> msg_counter;   // thread-local counter of messages being sent and received by current thread
    bool acc_msg_counter_not_yet_added_up;  // flag that gets set when the cumulative messages counter 'acc_msg_counter' gets added to the message counter local to the current thread
    bool owner_of_new_neighbor;     // flag that gets set when, after expanding a node, the thread finds a neighbor of that node of which it is the owner


    /**********************************************/
    /*                                            */
    /*    Fill up the random-bit-strings table    */
    /*                                            */
    /**********************************************/

    // number of entries of the random-bit-strings table per partition (except for the last partition, in which there could be less entries)
    R_entries_per_partition = (unsigned int) ((max_graph_coord_span + 1) / num_R_partit) + 1;
    
    for (unsigned int i = 0; i < num_R_partit; ++i) {

        /* depending on the flag FILL_R_IN_PARALLEL set by the user, let the random-bit-strings table R be filled-up
        by either a single thread or multiple threads. */
        if (FILL_R_IN_PARALLEL) {

            // get synchronization variables of i-th partition
            ptr_to_atomic_flag& af_ptr = threads_synch[i];

            // if i-th partition has been already taken by another thread, then go to next partition
            if ((*af_ptr).test_and_set())
                continue;
        }
        else {
            
            // let just one thread fill-up the random-bit-strings table R
            if (thread_id != 0)
                break;
        }

        // choose proper indexes of R entries to be filled with random 4-bytes strings
        R_lb_idx = i * R_entries_per_partition;
        R_ub_idx = R_lb_idx + R_entries_per_partition - 1;
        if (R_ub_idx > max_graph_coord_span)     // make sure to not exceed the last R entry index
            R_ub_idx = max_graph_coord_span;
        
        // generate random entries of R
        for (unsigned int R_idx = R_lb_idx; R_idx <= R_ub_idx; ++R_idx) {
            R[R_idx] = random_in_range<int>(INT_MIN, INT_MAX);
        }

    }


    /* wait for all running threads to finish filling up the random-bit-strings table, insert the 'start' node into the appropriate open list and
    to insert their TID in the array "tid_v" */
    pthread_mutex_lock(mb);
    ++barrier_counter;
    if (barrier_counter == num_threads) {

        // reset barrier counter (for next barrier)
        barrier_counter = 0;

        // reset synch variables
        for (unsigned int i = 0; i < num_R_partit; i++) {
            ptr_to_atomic_flag& af_ptr = threads_synch[i];
            (*af_ptr).clear();
        }

        // notify the main thread that the random-bit-strings table is ready
        std::unique_lock<std::mutex> lock_R{m_R};
        R_ready = true;
        cv_R.notify_one();
    }
    pthread_mutex_unlock(mb);
    sem_wait(barrier_1);


    /* if the current thread owns the "start" node, then initialize its cost and its parent */
    if((hashed_thread_id = a_star_hash(start, num_threads, R, hash_type, AZ_hash_mask)) == UNDEFINED_THREAD) {
        // ensure that only one thread executes the "abort all threads" procedure
        std::unique_lock<std::mutex> lock{m_abort};

        // notify that an exception was raised and abort all concurrent threads
        std::cerr << "Thread " << thread_id << " failed to retrieve a thread ID by hashing a graph's node" << std::endl;
        abort_all_threads(thread_id, num_threads, error_occurred);
    }
    if (hashed_thread_id == (int)thread_id) {
        cost[start] = 0;
        from[start] = start;
    }


    /******************** Decentralized A* initial conditions ********************/

    need_to_wait_for_msg = false;
    need_to_set_best_path = false;
    acc_msg_counter_not_yet_added_up = true;
    try {
        msg_counter.resize(num_threads, 0);
    }
    catch (std::exception& e) {
        // ensure that only one thread executes the "abort all threads" procedure
        std::unique_lock<std::mutex> lock{m_abort};

        // notify that an exception was raised and abort all concurrent threads
        std::cerr << "Thread " << thread_id << " failed to allocate memory for its own message counter" << std::endl;
        std::cerr << "Error: " << e.what() << std::endl;
        abort_all_threads(thread_id, num_threads, error_occurred);
    }


    /*******************************/
    /*                             */
    /*    Decentralized A* loop    */
    /*                             */
    /*******************************/

    // no neighbor node has been found by the current thread yet
    owner_of_new_neighbor = false;

    while (!algorithm_terminated.load()) {
        
        // no neighbor node has been found by the current thread yet
        owner_of_new_neighbor = false;

        /******************** Check if current thread can sleep ********************/

        while (need_to_wait_for_msg && !found_new_best_path.load()) {

            std::unique_lock<std::mutex> lock_sleep{m_msg};
            need_to_sleep = (msg_buffer.empty() && !algorithm_terminated.load() && acc_msg_counter.second.load() != thread_id);

            if (need_to_sleep) {
                
                // there's no node that is worth to be expanded, so just go to sleep and wait for new messages from other threads
                cv_thread.wait(lock_sleep);
            }
            else
                need_to_wait_for_msg = false;
        }
        

        /******************** Check messages from other threads ********************/
        
        while (true) {

            /* check if current thread received any message from other threads. If so, then get their content and process them */
            std::unique_lock<std::mutex> lock_check_msg_buffer{m_msg};
            if (!msg_buffer.empty()) {

                // extract the new node from the message
                msg = msg_buffer.front();
                msg_buffer.pop();
            }
            else
                break;
            lock_check_msg_buffer.unlock();

            // get message content, i.e. a new node N to be expanded, its cost g(N) and its parent N'
            n_child = std::get<0>(msg);
            g_child = std::get<1>(msg);
            n_parent = std::get<2>(msg);


            /******************** Check if received node is worth expanding ********************/

            add_node_if_worth_expanding (thread_id, num_threads, n_child, g_child, n_parent, stop,
                                        std::ref(open), std::ref(closed), std::ref(cost), std::ref(from));

            // update the counter of messages received by current thread
            msg_counter[thread_id]--;
        }
        

        /******************** Expand a node in the open list ********************/

        node_expanded = false;

        /* NOTE: if the following statement is true then another thread found a new better path from start to stop node and it is
        trying to rebuild the path, hence the current thread must help it. */
        if (!found_new_best_path.load()) {
            
            // check the cost of current best path
            best_path_cost = best_path.path_cost.load();

            /* if open list of current thread is empty or the lowest f(N) value in it is not lower than the cost of the
            overall best path found until now (if there is any), then go to next iteration. */
            if (open.empty() || (best_path_cost != UNDEFINED_COST && std::get<0>(open.top()) >= best_path_cost)) {

                need_to_wait_for_msg = true;
            }

            if (!need_to_wait_for_msg) {
                
                /* expand a node: get from open list the node N with lowest f(N), revome it from open list and add it to closed list. */
                node_expanded = true;

                node_in_open_list = open.top();
                f = std::get<0>(node_in_open_list);
                h = std::get<1>(node_in_open_list);
                g = f - h;
                n = std::get<2>(node_in_open_list);
                open.pop();

                /* check that the current node has NOT been already expanded. If it is, then skip to next iteration, otherwise expand the node. */
                if (closed.find(n.get_id()) != closed.end())
                    continue;
                
                closed[n.get_id()] = node_in_open_list;
            
                // if the node N just expanded is the goal node and its overall cost is lower than the best solution found previously, then substitute it
                if (n == stop) {
                    best_path_cost = best_path.path_cost.load();
                    need_to_set_best_path = (best_path_cost == UNDEFINED_COST || f < best_path_cost);
                }
            }
        }


        /******************** Help to rebuild the path from START to STOP node ********************/

        /* NOTE: the following code can be executed by all threads except the one that owns the "stop" node */
        while (found_new_best_path.load()) {

            bool& parent_request_arrived = parent_request.second;

            std::unique_lock<std::mutex> lock_check_parent_requests{m_msg};

            /* if another thread requested to know the parent of a node N of which the current thread is the owner, then the
            current thread must reply with parent(N), otherwise current thread can sleep. */
            if (!parent_request_arrived)
                cv_thread.wait(lock_check_parent_requests);

            if (!parent_request_arrived) {      // check for spurious wakeups (just to be sure)
                continue;
            }

            lock_check_parent_requests.unlock();


            /******************** Reply to the parent request ********************/

            // reset the "parent request arrived" flag
            parent_request_arrived = false;

            // extract a node N from the requests buffer and reply by telling which node is the parent of N
            n_child = parent_request.first;
            parent_reply = std::make_pair(from[n_child], true);

            // notify the thread that requested to know which node is the parent of N
            (*(cv_threads_v[stop_node_owner])).notify_one();
        }


        /******************** Update best path ********************/
        
        /* NOTE: the following code can be executed only by the thread that owns the "stop" node */
        if (need_to_set_best_path) {

            // notify all other threads that a new best path has just been found
            found_new_best_path.store(true);

            // reset the best path (except for its cost, which will be updated at the end of the path rebuilding procedure)
            best_path.path_ptr = std::make_unique<path_umap>();
            best_path.path_num_nodes = 0;
            best_path_cost = 0;


            /******************** Rebuild the path from START to STOP node ********************/

            /* NOTE: the nodes of the best path found until now will be saved with reverse indexes
            (i.e. "stop" node has index == 0, whereas "start" node has index == K-1, where K is the
            total number of nodes of which the best path is composed) */

            n_child = n;
            n_parent = from[n];

            while (true) {
                
                // add current node (and its cost) to the best path
                (*(best_path.path_ptr))[best_path.path_num_nodes] = n_child;
                best_path.path_num_nodes++;
                if (n_child == n_parent)
                    break;
                best_path_cost += n_parent.get_link_weight(n_child.get_id());

                
                /******************** Request parent of current node ********************/

                n_child = n_parent;
                
                /* get the parent of the new n_child by the thread that owns it */
                if((hashed_thread_id = a_star_hash(n_child, num_threads, R, hash_type, AZ_hash_mask)) == UNDEFINED_THREAD) {
                    // ensure that only one thread executes the "abort all threads" procedure
                    std::unique_lock<std::mutex> lock{m_abort};

                    // notify that an exception was raised and abort all concurrent threads
                    std::cerr << "Thread " << thread_id << " failed to retrieve a thread ID by hashing a graph's node" << std::endl;
                    abort_all_threads(thread_id, num_threads, error_occurred);
                }


                /* if n_child is owned by the current thread then there's no need to ask to any other thread, otherwise we need to
                ask to n_child's owner which is the parent of n_child */
                if (hashed_thread_id == (int)thread_id) {
                    
                    // n_child is owned by the current thread
                    n_parent = from[n_child];
                }
                else {
                    
                    // n_child is owned by some other thread, hence we need to ask it to send us parent(n_child)
                    parent_request_t& parent_request_node_owner = parent_request_buffer[hashed_thread_id];

                    // make the parent request
                    std::unique_lock<std::mutex> lock_request_node_parent{*(m_msg_v[hashed_thread_id])};
                    parent_request_node_owner = std::make_pair(n_child, true);

                    // notify n_child's owner that the parent request has been sent
                    (*(cv_threads_v[hashed_thread_id])).notify_one();
                    lock_request_node_parent.unlock();

                    // wait for the reply
                    bool& parent_request_answered = parent_reply.second;
                    std::unique_lock<std::mutex> lock_get_parent{m_msg};
                    while (!parent_request_answered) {
                        
                        // the reply isn't arrived yet, hence current thread can sleep
                        cv_thread.wait(lock_get_parent);
                    }

                    // get parent of n_child
                    n_parent = parent_reply.first;
                    parent_request_answered = false;
                }

                /* when arrived here, the current thread knows for sure the parent of n_child */
            }


            /******************** Update the cost of the best path ********************/

            best_path.path_cost.store(best_path_cost);

            // reset the best path flags
            need_to_set_best_path = false;
            found_new_best_path.store(false);

            // notify all other threads that the best path has been updated
            for (unsigned int id = 0; id < num_threads; ++id)
                (*(cv_threads_v[id])).notify_one();
        }

        
        /* NOTE: this if-statement addresses the algorithm termination problem in the special case of Decentralized A* being executed by just 1 thread.
        Since the Euler Norm heuristic is both admissible (i.e. it never overestimates the actual distance to the goal node) and consistent
        (i.e. it fulfills the triangle inequality), it is guaranteed that, in case of single-thread execution, the first solution found is
        also the optimal one. */
        if (num_threads == 1 && best_path.path_cost.load() != UNDEFINED_COST)
            break;

        
        /******************** Send current node neighbors to their owners ********************/

        /* for each neighbor N' of the expanded node N, compute g(N') and send the tuple [N', g(N'), N] to the thread that owns N'
        (based on the chosen hashing method). */
        if (node_expanded && n != stop) {
            
            for (auto link : n.get_links()) {

                // collect i-th child node N' and its cost g(N')
                n_child = graph[link.first];
                if (n_child == from[n])
                    continue;
                g_child =  g + link.second;

                /* find the ID of the thread that owns N' */
                if((hashed_thread_id = a_star_hash(n_child, num_threads, R, hash_type, AZ_hash_mask)) == UNDEFINED_THREAD) {

                    // ensure that only one thread executes the "abort all threads" procedure
                    std::unique_lock<std::mutex> lock{m_abort};

                    // notify that an exception was raised and abort all concurrent threads
                    std::cerr << "Thread " << thread_id << " failed to retrieve a thread ID by hashing a graph's node" << std::endl;
                    abort_all_threads(thread_id, num_threads, error_occurred);
                }


                if (hashed_thread_id == (int)thread_id) {
                    
                    /* current thread is the owner of N', so there is no need to send any message */
                    
                    // add N' to the open list of current thread if it is worth to expand that node
                    add_node_if_worth_expanding (thread_id, num_threads, n_child, g_child, n, stop,
                                                std::ref(open), std::ref(closed), std::ref(cost), std::ref(from));
                    
                    // the current thread just found a neighbor node of which it is the owner
                    owner_of_new_neighbor = true;
                }
                else {

                    /* current thread is NOT the owner of N', so send the tuple [N', g(N'), N] to the thread that actually owns N' */
                    
                    // create message
                    msg_buffer_t& msg_buffer_node_owner = msg_buffer_v[hashed_thread_id];
                    msg = std::make_tuple(n_child, g_child, n);

                    // send message
                    std::unique_lock<std::mutex> lock_send_msg{*(m_msg_v[hashed_thread_id])};
                    msg_buffer_node_owner.push(msg);
                    
                    // update the counter of messages sent to thread with ID == hashed_thread_id
                    msg_counter[hashed_thread_id]++;

                    // notify the thread owning N' that the current thread sent a message with the tuple [N', g(N'), N]
                    (*(cv_threads_v[hashed_thread_id])).notify_one();
                }
            }
        }


        /******************** Update algorithm termination conditions ********************/

        /* check if the cumulative messages counter 'acc_msg_counter' needs to be checked by the current thread. If yes then add the
        cumulative messages counter to the local message counter of the current thread, otherwise go to next iteration. */

        if (acc_msg_counter.second.load() != thread_id || num_threads == 1)
            continue;   // go to next iteration


        // if cumulative and thread-local message counters did add up yet...
        if (acc_msg_counter_not_yet_added_up) {
            
            // ... then add them up
            for (unsigned int id = 0; id < num_threads; ++id)
                msg_counter[id] += acc_msg_counter.first[id];

            acc_msg_counter_not_yet_added_up = false;
        }

        // check if the new thread-local message counter contains 0 in every entry
        bool msg_counter_all_zeros = std::all_of(msg_counter.begin(), msg_counter.end(), [](int count) { return count == 0; });


        /******************** Check termination conditions ********************/

        if (termination_starter_thread.load() != UNDEFINED_THREAD && (!msg_counter_all_zeros || node_expanded || owner_of_new_neighbor)) {
            
            // reset termination condition
            termination_starter_thread.store(UNDEFINED_THREAD);
        }
        else if (termination_starter_thread.load() == (int)thread_id && msg_counter_all_zeros) {
           
            // termination condition verified: wake up all threads and exit the Decentralized A* loop
            algorithm_terminated.store(true);
            for (unsigned int id = 0; id < num_threads; ++id)
                (*(cv_threads_v[id])).notify_one();
        }
        else if (msg_counter[thread_id] <= 0) {

            if (termination_starter_thread.load() == UNDEFINED_THREAD && msg_counter_all_zeros) {
                
                /* the current thread is the first to see that the cumulative messages counter is full of 0. If the cumulative message counter traverses
                all other threads and comes back to this thread remaining always full of 0, then the algorithm can terminate (i.e. either there are no
                solutions better than the one already found or there is no solution at all). */
                termination_starter_thread.store(thread_id);
            }

            // update cumulative messages counter
            acc_msg_counter.first = msg_counter;

            // reset local messages counter of current thread
            std::fill(msg_counter.begin(), msg_counter.end(), 0);

            // get ID of next thread
            int next_thread_id = (thread_id + 1) % num_threads;

            /* notify the next thread that it is its time to check the cumulative messages counter */
            acc_msg_counter.second.store(next_thread_id);
            std::unique_lock<std::mutex> lock_msg_counter{*(m_msg_v[next_thread_id])};
            (*(cv_threads_v[next_thread_id])).notify_one();
            acc_msg_counter_not_yet_added_up = true;
        }
    }

}


void rebuild_path_single_thread(graph_path_t& best_path, std::unordered_map<Node, Node>& from, const Node& start, const Node& stop) {
    
    Node n_child, n_parent;

    // begin rebuilding the path by starting from "stop" node
    n_child = stop;
    n_parent = from[n_child];
    best_path.path_num_nodes = 0;
    int path_cost = 0;
    best_path.path_ptr = std::make_unique<path_umap>();

    // rebuild the entire path going backward from "stop" to "start" node
    while (true) {

        // record current node of the path
        (*(best_path.path_ptr))[best_path.path_num_nodes] = n_child;
        best_path.path_num_nodes++;

        // if the current node is equal to its parent then the current node is "start" node, hence the path rebuilding process is finished
        if (n_child == n_parent)
            break;

        // update the overall path cost
        path_cost += n_parent.get_link_weight(n_child.get_id());

        // go to next child-parent couple
        n_child = n_parent;
        n_parent = from[n_child];
    }

    // update the cost of the best path structure
    best_path.path_cost = path_cost;
}


double calculate_h(const Node& n1, const Node& n2) {

    /* if the TEST_HIGH_NODE_EXPANSION_COST flag is set then the program simulates graphs where the expansion of a new graph's node
    is really expensive, so that the user can actually see the effectiveness of the Centralized A* approach with respect to the
    Sequential and Decentralized A* in such cases. The TEST_HIGH_NODE_EXPANSION_COST and NODE_EXPANSION_DELAY macros can be set
    by the user in the "a_star.h" file. */
    if (TEST_HIGH_NODE_EXPANSION_COST)
        std::this_thread::sleep_for(std::chrono::milliseconds(NODE_EXPANSION_DELAY));

    // return the Euclidean distance between "n1" and "n2" nodes
    return sqrt(pow(n1.get_x() - n2.get_x(), 2) + pow(n1.get_y() - n2.get_y(), 2));
}


void add_node_if_worth_expanding (unsigned int& thread_id, unsigned int& num_threads, Node& n_child, double& g_child, Node& n_parent, const Node& stop,
                                open_list& open, closed_list& closed, std::unordered_map<Node, unsigned int>& cost, std::unordered_map<Node, Node>& from) {

    // current costs of the node 'n_child' for which it has been found a new path
    double f, g, h;     // NOTE: f(N) = g(N) + h(N) for every node N

    // element of closed list
    list_elem node_in_closed_list;

    // represents if the new path for a given node is better than the one found previously or not
    bool new_path_is_worth = false;

    // check if node is already in closed list
    if (closed.find(n_child.get_id()) != closed.end()) {

        // the new node is in closed list. Now check if we need to remove it from there and add it to the open list of the current thread
        node_in_closed_list = closed[n_child.get_id()];
        f = std::get<0>(node_in_closed_list);
        h = std::get<1>(node_in_closed_list);
        g = f - h;

        if (g_child < g) {

            /* remove the new node from the closed list and add it to the open list */
            if (!closed.erase(n_child.get_id())) {
                // ensure that only one thread executes the "abort all threads" procedure
                std::unique_lock<std::mutex> lock{m_abort};

                // notify that an exception was raised and abort all concurrent threads
                std::cerr << "Thread " << thread_id << " failed to erase a node from its closed list" << std::endl;
                abort_all_threads(thread_id, num_threads, error_occurred);
            }
            
            new_path_is_worth = true;
        }
    }
    else {

        // the new node is NOT in closed list. Now check if we need to add it to the open list of the current thread
        if (cost.find(n_child) == cost.end()) {
            
            // the new node is NOT in the open list, so add it into the open list
            new_path_is_worth = true;
        }
        else {

            // new node is already in the open list, so retrieve it and compare the new value g(N) with the old one
            g = cost[n_child];
            
            if (g_child < g) {

                // the new path for the node has a lower cost with respect to the old path, so substitute it
                new_path_is_worth = true;
            }
        }
    }

    // check if the new path received by message is better than the previous one and, if so, substitute it
    if (new_path_is_worth) {
        
        h = calculate_h(n_child, stop);
        f = g_child + h;
        open.emplace(f, h, n_child);
        cost[n_child] = g_child;
        from[n_child] = n_parent;
    }
}
















