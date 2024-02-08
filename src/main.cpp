#include "graph_gen_store_load.h"
#include "a_star.h"
#include "utilities.h"
#include "menu.h"

#include <iostream>
#include <cstring>


int main(int argc, char *argv[]) {

    // make the stdout stream unbuffered
    setbuf(stdout, 0);

    // set random seed for main thread
    srand(time(NULL));

    // check for input arguments
    if (argc != 2) {
        fprintf(stderr, "The program expects to receive as argument the path of the file where the graph has to be written/read to/from\n");
        exit(-1);
    }

    // check for macros correctness (just to be sure)
    if (MIN_COEFF_WEIGHT_LINK > MAX_COEFF_WEIGHT_LINK || MIN_COEFF_WEIGHT_LINK <= 0 || MAX_COEFF_WEIGHT_LINK <= 0 ||
        MAX_ADJ_PARTIT_IN_2D_GRAPH != 4 ||
        (TEST_HIGH_NODE_EXPANSION_COST == 1 && NODE_EXPANSION_DELAY <= 0)) {

        fprintf(stderr, "Macros defined in \'graph_gen_store_load.h\' assume illegal values:\n");
        fprintf(stderr, "1. MIN_COEFF_WEIGHT_LINK and MAX_COEFF_WEIGHT_LINK must be positive numbers such that MIN_COEFF_WEIGHT_LINK < MAX_COEFF_WEIGHT_LINK.\n");
        fprintf(stderr, "2. MAX_ADJ_PARTIT_IN_2D_GRAPH must be equal to 4.\n");
        fprintf(stderr, "3. If TEST_HIGH_NODE_EXPANSION_COST is set, NODE_EXPANSION_DELAY must be > 0.\n");
        return -1;
    }


    /******************** Variables definition ********************/

    // variables to be initialized by user's inputs
    char *path_file_graph;      // path name of the file where to store/load the graph 
    int menu_option;            // program functionality to be run
    int num_nodes_graph_gen;    // number of nodes that the generated graph should contain
    int num_threads_graph_gen;  // number of threads that have to concurrently generate the graph
    int num_threads_graph_read; // number of threads that have to concurrently read the graph from file
    std::vector<bool> a_star_versions_to_run;   // A* versions that the user wants to run
    int num_threads_centr_a_star;               // number of threads that have to concurrently run Centralized A* version
    int num_threads_decentr_a_star;             // number of threads that have to concurrently run Decentralized A* version (HDA*: Hash Distributed A*)
    int hash_type_decentr_a_star;               // hashing method to be used in the Decentralized A* version
    std::pair<int, int> start_stop_nodes_idx;   // ID of "start" and "stop" nodes of A* algorithm

    // variables to be initialized by the program execution
    std::vector<Node> nodes_v;  // graph containing all nodes (and their links)
    unsigned int num_nodes;     // number of nodes in the graph
    graph_path_t best_path;     // best path found by A*


    /******************** Acquire user's inputs for A* execution ********************/

    // get file's path
    path_file_graph = argv[1];

    // check for file's path correctness
    if (path_file_graph == NULL || strlen(path_file_graph) <= 0) {
        fprintf(stderr, "Input argument do not satisfy acceptance requirements: file's path must not be NULL\n");
        exit(-1);
    }


    if (!DEBUG_MODE) {
        
        // ask the user which functionalities of the program have to be executed
        a_star_menu(menu_option, num_nodes_graph_gen, num_threads_graph_gen, num_threads_graph_read, std::ref(a_star_versions_to_run),
                    num_threads_centr_a_star, num_threads_decentr_a_star, hash_type_decentr_a_star, std::ref(start_stop_nodes_idx));
    }
    else {

        // take inputs directly defined in the source code of debug_program_inputs function of "utilities.cpp" (useful to debug quickly)
        debug_program_inputs(menu_option, num_nodes_graph_gen, num_threads_graph_gen, num_threads_graph_read, std::ref(a_star_versions_to_run),
                            num_threads_centr_a_star, num_threads_decentr_a_star, hash_type_decentr_a_star, std::ref(start_stop_nodes_idx));
        

        // check that inputs inserted through the debug mode are actually correct
        if ((menu_option != 1 && menu_option != 2) || num_nodes_graph_gen <= 0 || num_threads_graph_gen <= 0 || num_threads_graph_read <= 0 ||
            (a_star_versions_to_run[0] != true && a_star_versions_to_run[0] != false) || (a_star_versions_to_run[1] != true && a_star_versions_to_run[1] != false) ||
            (a_star_versions_to_run[2] != true && a_star_versions_to_run[2] != false) || num_threads_centr_a_star <= 0 || num_threads_decentr_a_star <= 0 ||
            (hash_type_decentr_a_star != MULTIPLICATIVE_HASH && hash_type_decentr_a_star != ZOBRIST_HASH && hash_type_decentr_a_star != ABSTRACT_ZOBRIST_HASH) ||
            start_stop_nodes_idx.first < UNDEFINED_NODE || start_stop_nodes_idx.second < UNDEFINED_NODE) {
            
            // notify the user that the debug mode inputs are not correct
            std::cerr << "The inputs inserted through the debug mode are not coherent with what expected." << std::endl;
            return -1;
        }
    }


    if (menu_option == 1) {

        /**************************************************/
        /*                                                */
        /*    Menù option 1: generate a graph of nodes    */
        /*                                                */
        /**************************************************/

        std::cout << "/**************************/" << std::endl;
        std::cout << "/*    Graph generation    */" << std::endl;
        std::cout << "/**************************/" << std::endl << std::endl;

        /* arbitrary choice: we set both the width and height of the graph's X-Y grid equal to the number of nodes to be generated (a.k.a. "NUM_NODES"),
        so that the X-Y coordinates of each graph's node will be placed onto a grid having "NUM_NODES ^ 2" cells in total. */
        max_graph_coord_span = num_nodes_graph_gen;

        /* arbitrary choice: compute the optimal size of each graph partition along X and Y coordinates in order to have the
        least overall number of links in the graph to be generated.
        NOTE: this is just an implementation choice made in order to obtain a graph where it is “challenging” to find a path between
        two nodes and to reduce the time needed for the graph generation, but any choice of "graph_axes_partit" > 0 is totally fine. */
        try {
            graph_axes_partit = optimal_graph_axes_partit_size(num_nodes_graph_gen);
        }
        catch (std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return -1;
        }


        std::cout << "Graph generation started..." << std::endl;
        start_timer_graph_gen = std::chrono::steady_clock::now();   // record the time instant at which the graph generation started
        
        /* generate the graph */
        if (graph_generation(num_nodes_graph_gen, num_threads_graph_gen, nodes_v, path_file_graph) == -1) {
            std::cout << "Graph generation ended with an error. The program is going to shut down..." << std::endl;
            return -1;
        }
        
        stop_timer_graph_gen = std::chrono::steady_clock::now();    // record the time instant at which the graph generation ended
        std::cout << "Graph generation completed successfully" << std::endl << std::endl;

        // print the elapsed time for graph generation
        auto delta_t_graph_gen = std::chrono::duration_cast<std::chrono::microseconds>(stop_timer_graph_gen - start_timer_graph_gen).count();
        std::cout << "Elapsed time for graph generation: " << delta_t_graph_gen / 1000000.0 << " seconds" << std::endl << std::endl;

        /* after generating the graph, we can optionally print it on stdout */
        if (PRINT_GRAPH_AFTER_GEN)
            printf_graph(nodes_v);

    }
    else if (menu_option == 2) {

        /**********************************************************/
        /*                                                        */
        /*    Menù option 2: run the A* algorithm onto a graph    */
        /*                                                        */
        /**********************************************************/

        std::cout << "/**********************/" << std::endl;
        std::cout << "/*    A* algorithm    */" << std::endl;
        std::cout << "/**********************/" << std::endl << std::endl;

        /******************** Read file from file ********************/

        std::cout << "Graph reading from disk started..." << std::endl;
        start_timer_graph_read_from_file = std::chrono::steady_clock::now();    // record the time instant at which the graph read from file started

        /* read the graph from file */
        if (graph_read_from_file(num_threads_graph_read, nodes_v, path_file_graph) == -1) {
            std::cout << "Graph reading from disk ended with an error. The program is going to shut down..." << std::endl;
            return -1;
        }

        stop_timer_graph_read_from_file = std::chrono::steady_clock::now();     // record the time instant at which the graph read from file ended
        std::cout << "Graph has been successfully read from disk" << std::endl << std::endl;

        // print the elapsed time for reading the graph from file
        auto delta_t_graph_read_from_file = std::chrono::duration_cast<std::chrono::microseconds>(stop_timer_graph_read_from_file - start_timer_graph_read_from_file).count();
        std::cout << "Elapsed time for reading graph from file: " << delta_t_graph_read_from_file / 1000000.0 << " seconds" << std::endl << std::endl;

        /* after reading the graph from file, we can optionally print in on stdout */
        if (PRINT_GRAPH_AFTER_READ)
            printf_graph(nodes_v);
        
        // get number of nodes in the graph
        num_nodes = nodes_v.size();

        /* arbitrary choice: we set both the width and height of the graph's X-Y grid equal to the number of nodes to be generated (a.k.a. "NUM_NODES"),
        in order to represent that the X-Y coordinates of each graph's node are placed onto a grid having "NUM_NODES ^ 2" cells in total. */
        max_graph_coord_span = num_nodes;

        
        /******************** Select start and stop nodes ********************/

        // check that the graph has at least a "start" and a "stop" node for running the A* algorithm
        if (num_nodes <= 1) {

            std::cout << "A* algorithm can't be run onto a graph with less than 2 nodes." << std::endl;
            return 0;
        }

        // choose "start" and "stop" nodes (either randomly or according to user's input arguments)
        if (start_stop_nodes_idx.first == UNDEFINED_NODE || start_stop_nodes_idx.first >= (int)num_nodes)
            start_stop_nodes_idx.first = random_in_range<int>(0, num_nodes - 1);

        if (start_stop_nodes_idx.second == UNDEFINED_NODE || start_stop_nodes_idx.second >= (int)num_nodes) {
            do {
                start_stop_nodes_idx.second = random_in_range<int>(0, num_nodes - 1);
            } while (start_stop_nodes_idx.second == start_stop_nodes_idx.first);
        }

        /******************** Run Sequential A* algorithm ********************/

        if (a_star_versions_to_run[0]) {

            // initially set the best path as undefined (we didn't find it yet)
            reset_path(std::ref(best_path));
            
            std::cout << "Sequential A* started..." << std::endl;
            start_timer_a_star_sequential = std::chrono::steady_clock::now();   // record the time instant at which Sequential A* started

            // run sequential A*
            a_star_sequential(nodes_v, nodes_v[start_stop_nodes_idx.first], nodes_v[start_stop_nodes_idx.second], std::ref(best_path));

            stop_timer_a_star_sequential = std::chrono::steady_clock::now();    // record the time instant at which Sequential A* ended
            std::cout << "Sequential A* completed successfully" << std::endl << std::endl;

            // print the path found (if is there any)
            print_best_path(std::ref(best_path));
        }

        /******************** Run Centralized A* algorithm ********************/

        if (a_star_versions_to_run[1]) {

            // initially set the best path as undefined (we didn't find it yet)
            reset_path(std::ref(best_path));
            
            std::cout << "Centralized A* started..." << std::endl;
            start_timer_a_star_centralized = std::chrono::steady_clock::now();   // record the time instant at which Centralized A* started

            // run centralized A*
            a_star_centralized(nodes_v, nodes_v[start_stop_nodes_idx.first], nodes_v[start_stop_nodes_idx.second],
                                num_threads_centr_a_star, std::ref(best_path));

            stop_timer_a_star_centralized = std::chrono::steady_clock::now();    // record the time instant at which Centralized A* ended
            std::cout << "Centralized A* completed successfully" << std::endl << std::endl;

            // print the path found (if is there any)
            print_best_path(std::ref(best_path));
        }

        /******************** Run Decentralized A* algorithm ********************/

        if (a_star_versions_to_run[2]) {
            
            // initially set the best path as undefined (we didn't find it yet)
            reset_path(std::ref(best_path));
            
            std::cout << "Decentralized A* started..." << std::endl;
            start_timer_a_star_decentralized = std::chrono::steady_clock::now();   // record the time instant at which Decentralized A* started

            // run decentralized A*
            a_star_decentralized(nodes_v, nodes_v[start_stop_nodes_idx.first], nodes_v[start_stop_nodes_idx.second],
                                hash_type_decentr_a_star, num_threads_decentr_a_star, std::ref(best_path));

            stop_timer_a_star_decentralized = std::chrono::steady_clock::now();    // record the time instant at which Decentralized A* ended
            std::cout << "Decentralized A* completed successfully" << std::endl << std::endl;

            // print the path found (if is there any)
            print_best_path(std::ref(best_path));
        }

        std::cout << std::endl;


        /******************** Print elapsed time for A* execution ********************/

        std::cout << "/*************************/" << std::endl;
        std::cout << "/*    A* elapsed time    */" << std::endl;
        std::cout << "/*************************/" << std::endl << std::endl;

        if (a_star_versions_to_run[0]) {
            auto delta_t_a_star_sequential = std::chrono::duration_cast<std::chrono::microseconds>(stop_timer_a_star_sequential - start_timer_a_star_sequential).count();
            std::cout << "Elapsed time for Sequential A*: " << delta_t_a_star_sequential / 1000000.0 << " seconds" << std::endl << std::endl;
        }
        if (a_star_versions_to_run[1]) {
            auto delta_t_a_star_centralized = std::chrono::duration_cast<std::chrono::microseconds>(stop_timer_a_star_centralized - start_timer_a_star_centralized).count();
            std::cout << "Elapsed time for Centralized A*: " << delta_t_a_star_centralized / 1000000.0 << " seconds" << std::endl << std::endl; 
        }
        if (a_star_versions_to_run[2]) {
            auto delta_t_a_star_decentralized = std::chrono::duration_cast<std::chrono::microseconds>(stop_timer_a_star_decentralized - start_timer_a_star_decentralized).count();
            std::cout << "Elapsed time for Decentralized A*";

            if (hash_type_decentr_a_star == MULTIPLICATIVE_HASH)
                std::cout << " (MHDA*)";
            else if (hash_type_decentr_a_star == ZOBRIST_HASH)
                std::cout << " (ZHDA*)";
            else if (hash_type_decentr_a_star == ABSTRACT_ZOBRIST_HASH)
                std::cout << " (AZHDA*)";

            std::cout << ": " << delta_t_a_star_decentralized / 1000000.0 << " seconds" << std::endl << std::endl;
        }

    }
    else {

        // report the occurrence of an error
        std::cerr << "An error occurred when choosing the program functionality to be executed" << std::endl;
        return -1;
    }
    std::cout << std::endl;


    /******************** Print memory usage ********************/
    struct rusage ram_usage;    // structure to be filled with the statistics concerning the RAM usage

    if (getrusage(RUSAGE_SELF, &ram_usage)) {

        // report the occurrence of an error
        std::cerr << "An error occurred when trying to retrieve RAM usage statistics of the current process" << std::endl;
        return -1;
    }

    // print the RAM usage statistics
    std::cout << "Maximum amount of RAM (in MB) that the process used simultaneously: " << ((float) ram_usage.ru_maxrss) / 1024.0 << std::endl << std::endl;

    return 0;
}