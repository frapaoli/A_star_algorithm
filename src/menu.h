#ifndef MENU_H
#define MENU_H

#include <iostream>


/******************************/
/*                            */
/*    Functions prototypes    */
/*                            */
/******************************/

/*
 * name: a_star_menu
 * purpose: menù from which the user can make its own choices regarding the program functionalities to be run.
 * parameters: program functionality to be run; number of nodes of generated graph; number of threads that have to concurrently generate the graph;
 *              number of threads that have to concurrently read the graph from file; A* versions that the user wants to run;
 *              number of threads that have to concurrently run Centralized A* version; number of threads that have to concurrently run Decentralized A* version;
 *              hashing method to be used in the Decentralized A* version; ID of "start" and "stop" nodes of A* algorithm.
 * return value: none.
 */
void a_star_menu(int& menu_option,
                int& num_nodes_graph_gen,
                int& num_threads_graph_gen,
                int& num_threads_graph_read,
                std::vector<bool>& a_star_versions_to_run,
                int& num_threads_centr_a_star,
                int& num_threads_decentr_a_star,
                int& hash_type_decentr_a_star,
                std::pair<int, int>& start_stop_nodes_idx);


#endif