#include "graph_gen_store_load.h"
#include "a_star.h"
#include "utilities.h"
#include "menu.h"

#include <iostream>
#include <algorithm>
#include <climits>


/*****************************************/
/*                                       */
/*      Implementation of A* menù        */
/*                                       */
/*****************************************/

void a_star_menu(int& menu_option,
                int& num_nodes_graph_gen,
                int& num_threads_graph_gen,
                int& num_threads_graph_read,
                std::vector<bool>& a_star_versions_to_run,
                int& num_threads_centr_a_star,
                int& num_threads_decentr_a_star,
                int& hash_type_decentr_a_star,
                std::pair<int, int>& start_stop_nodes_idx) {
    
    
    // variables declaration/initialization
    bool condition;         // used to check if user's keyboard inputs are coherent with what is being asked in the A* menù
    bool contains_only_digits;  // used to check if a string contains only digits
    std::string answer;     // used to store yes/no user's answers


    /*****************************************************************************************/
    /*                                                                                       */
    /*    Take user's choices regarding which A* algorithm functionalities have to be run    */
    /*                                                                                       */
    /*****************************************************************************************/

    std::cout << "/***************************/" << std::endl;
    std::cout << "/*    A* algorithm menù    */" << std::endl;
    std::cout << "/***************************/" << std::endl << std::endl;

    // take user's choice regarding which functionality of the program should be used
    do {
        std::cout << "Select one of the following functionalities:" << std::endl;
        std::cout << "1) Generate a graph of nodes." << std::endl;
        std::cout << "2) Run the A* algorithm onto a graph." << std::endl;

        getline(std::cin, answer);
        std::cout << std::endl;

        try {
            // check if user's input contains only digits
            contains_only_digits = std::all_of(answer.begin(), answer.end(), ::isdigit);
            if (!contains_only_digits)
                throw std::invalid_argument("input must contain only digits.");
            
            // check if user's input can be represented as an integer
            if (std::stol(answer) > INT_MAX)
                throw std::invalid_argument("insert a smaller value, so that it can be represented as an integer.");

            // NOTE: this can raise std::invalid_argument exception
            menu_option = std::stoi(answer);

            condition = menu_option == 1 || menu_option == 2;
            if (!condition)
                throw std::invalid_argument("insert a value among the proposed options.");
                
        }
        catch(std::exception& e) {
            std::cout << "Input is not valid: " << e.what() << std::endl << std::endl;
            condition = false;
        }
    } while (!condition);


    if (menu_option == 1) {

        /******************** Menù option 1: generate a graph of nodes ********************/
        
        // take user's choice regarding how many nodes the graph should contain
        do {
            std::cout << "How many nodes do you want the graph to contain?" << std::endl;

            getline(std::cin, answer);
            std::cout << std::endl;
            
            try {
                // check if user's input contains only digits
                contains_only_digits = std::all_of(answer.begin(), answer.end(), ::isdigit);
                if (!contains_only_digits)
                    throw std::invalid_argument("input must contain only digits.");
                
                // check if user's input can be represented as an integer
                if (std::stol(answer) > INT_MAX)
                    throw std::invalid_argument("insert a smaller value, so that it can be represented as an integer.");

                // NOTE: this can raise std::invalid_argument exception
                num_nodes_graph_gen = std::stoi(answer);

                condition = num_nodes_graph_gen >= 2;
                if (!condition)
                    throw std::invalid_argument("the graph must contain at least 2 nodes in order to run the A* algorithm on it.");
                    
            }
            catch(std::exception& e) {
                std::cout << "Input is not valid: " << e.what() << std::endl << std::endl;
                condition = false;
            }
        } while (!condition);

        // take user's choice regarding how many threads should concurrently run in order to generate the graph
        do {
            std::cout << "How many threads do you want to parallely generate the graph?" << std::endl;

            getline(std::cin, answer);
            std::cout << std::endl;

            try {
                // check if user's input contains only digits
                contains_only_digits = std::all_of(answer.begin(), answer.end(), ::isdigit);
                if (!contains_only_digits)
                    throw std::invalid_argument("input must contain only digits.");
                
                // check if user's input can be represented as an integer
                if (std::stol(answer) > INT_MAX)
                    throw std::invalid_argument("insert a smaller value, so that it can be represented as an integer.");
                
                // NOTE: this can raise std::invalid_argument exception
                num_threads_graph_gen = std::stoi(answer);

                condition = num_threads_graph_gen >= 1;
                if (!condition)
                    throw std::invalid_argument("insert a positive number of threads.");
                    
            }
            catch(std::exception& e) {
                std::cout << "Input is not valid: " << e.what() << std::endl << std::endl;
                condition = false;
            }
        } while (!condition);

    }
    else {

        /******************** Menù option 2: run the A* algorithm onto a graph ********************/

        // take user's choice regarding how many threads should concurrently read the graph from the file where it is stored
        do {
            std::cout << "How many threads do you want to parallely read the graph from file?" << std::endl;

            getline(std::cin, answer);
            std::cout << std::endl;

            try {
                // check if user's input contains only digits
                contains_only_digits = std::all_of(answer.begin(), answer.end(), ::isdigit);
                if (!contains_only_digits)
                    throw std::invalid_argument("input must contain only digits.");
                
                // check if user's input can be represented as an integer
                if (std::stol(answer) > INT_MAX)
                    throw std::invalid_argument("insert a smaller value, so that it can be represented as an integer.");
                
                // NOTE: this can raise std::invalid_argument exception
                num_threads_graph_read = std::stoi(answer);

                condition = num_threads_graph_read >= 1;
                if (!condition)
                    throw std::invalid_argument("insert a positive number of threads.");
                    
            }
            catch(std::exception& e) {
                std::cout << "Input is not valid: " << e.what() << std::endl << std::endl;
                condition = false;
            }
        } while (!condition);

        /******************** Sequential A* version ********************/

        // ask the user if Sequential A* version has to be run
        do {
            std::cout << "Do you want to run Sequential A* algorithm on the graph? [yes/y] or [no/n]" << std::endl;
            
            getline(std::cin, answer);
            std::cout << std::endl;

            condition = !answer.compare("yes") || !answer.compare("y") || !answer.compare("no") || !answer.compare("n");
            if (!condition) {
                std::cout << "Input is not valid. The answer must be either \"yes\", \"y\", \"no\", \"n\"." << std::endl << std::endl;
            }
        } while (!condition);

        // record user's choice of either running the Sequential A* version or not
        a_star_versions_to_run.emplace_back((!answer.compare("yes") || !answer.compare("y")) ? true : false);

        /******************** Centralized A* version ********************/

        // ask the user if Centralized A* version has to be run
        do {
            std::cout << "Do you want to run Centralized A* algorithm on the graph? [yes/y] or [no/n]" << std::endl;
            
            getline(std::cin, answer);
            std::cout << std::endl;

            condition = !answer.compare("yes") || !answer.compare("y") || !answer.compare("no") || !answer.compare("n");
            if (!condition) {
                std::cout << "Input is not valid. The answer must be either \"yes\", \"y\", \"no\", \"n\"." << std::endl << std::endl;
            }
        } while (!condition);

        // record user's choice of either running the Sequential A* version or not
        a_star_versions_to_run.emplace_back((!answer.compare("yes") || !answer.compare("y")) ? true : false);

        // if Centralized A* version has to be actually run, ask the user some details about its execution
        if (a_star_versions_to_run.back()) {

            // take user's choice regarding how many threads should concurrently run the Centralized A* version
            do {
                std::cout << "How many threads do you want to parallely run the Centralized A*?" << std::endl;

                getline(std::cin, answer);
                std::cout << std::endl;

                try {
                    // check if user's input contains only digits
                    contains_only_digits = std::all_of(answer.begin(), answer.end(), ::isdigit);
                    if (!contains_only_digits)
                        throw std::invalid_argument("input must contain only digits.");
                    
                    // check if user's input can be represented as an integer
                    if (std::stol(answer) > INT_MAX)
                        throw std::invalid_argument("insert a smaller value, so that it can be represented as an integer.");
                    
                    // NOTE: this can raise std::invalid_argument exception
                    num_threads_centr_a_star = std::stoi(answer);

                    condition = num_threads_centr_a_star >= 1;
                    if (!condition)
                        throw std::invalid_argument("insert a positive number of threads.");
                        
                }
                catch(std::exception& e) {
                    std::cout << "Input is not valid: " << e.what() << std::endl << std::endl;
                    condition = false;
                }
            } while (!condition);
        }

        /******************** Decentralized A* version ********************/

        // ask the user if Decentralized A* version has to be run
        do {
            std::cout << "Do you want to run Decentralized A* algorithm on the graph? [yes/y] or [no/n]" << std::endl;
            
            getline(std::cin, answer);
            std::cout << std::endl;

            condition = !answer.compare("yes") || !answer.compare("y") || !answer.compare("no") || !answer.compare("n");
            if (!condition) {
                std::cout << "Input is not valid. The answer must be either \"yes\", \"y\", \"no\", \"n\"." << std::endl << std::endl;
            }
        } while (!condition);

        // record user's choice of either running the Sequential A* version or not
        a_star_versions_to_run.emplace_back((!answer.compare("yes") || !answer.compare("y")) ? true : false);

        // if Decentralized A* version has to be actually run, ask the user some details about its execution
        if (a_star_versions_to_run.back()) {

            // take user's choice regarding how many threads should concurrently run the Decentralized A* version
            do {
                std::cout << "How many threads do you want to parallely run the Decentralized A*?" << std::endl;

                getline(std::cin, answer);
                std::cout << std::endl;

                try {
                    // check if user's input contains only digits
                    contains_only_digits = std::all_of(answer.begin(), answer.end(), ::isdigit);
                    if (!contains_only_digits)
                        throw std::invalid_argument("input must contain only digits.");
                    
                    // check if user's input can be represented as an integer
                    if (std::stol(answer) > INT_MAX)
                        throw std::invalid_argument("insert a smaller value, so that it can be represented as an integer.");
                    
                    // NOTE: this can raise std::invalid_argument exception
                    num_threads_decentr_a_star = std::stoi(answer);

                    condition = num_threads_decentr_a_star >= 1;
                    if (!condition)
                        throw std::invalid_argument("insert a positive number of threads.");
                        
                }
                catch(std::exception& e) {
                    std::cout << "Input is not valid: " << e.what() << std::endl << std::endl;
                    condition = false;
                }
            } while (!condition);

            // take user's choice regarding which functionality of the program should be used
            do {
                std::cout << "Select the hashing method to be used in the Decentralized A*:" << std::endl;
                std::cout << "1) Multiplicative Hash (MHDA*)" << std::endl;
                std::cout << "2) Zobrist Hash (ZHDA*)" << std::endl;
                std::cout << "3) Abstract Zobrist Hash (AZHDA*)" << std::endl;
                
                getline(std::cin, answer);
                std::cout << std::endl;

                try {
                    // check if user's input contains only digits
                    contains_only_digits = std::all_of(answer.begin(), answer.end(), ::isdigit);
                    if (!contains_only_digits)
                        throw std::invalid_argument("input must contain only digits.");
                    
                    // check if user's input can be represented as an integer
                    if (std::stol(answer) > INT_MAX)
                        throw std::invalid_argument("insert a smaller value, so that it can be represented as an integer.");
                    
                    // NOTE: this can raise std::invalid_argument exception
                    hash_type_decentr_a_star = std::stoi(answer);

                    condition = hash_type_decentr_a_star == 1 || hash_type_decentr_a_star == 2 || hash_type_decentr_a_star == 3;
                    if (!condition)
                        throw std::invalid_argument("insert a value among the proposed options.");
                        
                }
                catch(std::exception& e) {
                    std::cout << "Input is not valid: " << e.what() << std::endl << std::endl;
                    condition = false;
                }
            } while (!condition);

        }

        /******************** Start and Stop nodes choice ********************/

        // take user's choice regarding which should be the "start" node of the A* algorithm
        std::cout << "Insert the IDs of the START and STOP nodes between which the A* algorithm has to find the best path?" << std::endl;
        std::cout << "NOTES:" << std::endl;
        std::cout << "- If you want the START and/or STOP node to be chosen randomly, please insert \"-1\"" << std::endl;
        std::cout << "- If an inserted ID does not correspond to a graph's node, it will be chosen randomly" << std::endl << std::endl;

        do {
            std::cout << "Start node ID: ";
            
            getline(std::cin, answer);
            std::cout << std::endl;

            // if user inserted "-1", the start node of A* will be chosen randomly
            if (!answer.compare("-1")) {
                start_stop_nodes_idx.first = -1;
                break;
            }

            try {
                // check if user's input contains only digits
                contains_only_digits = std::all_of(answer.begin(), answer.end(), ::isdigit);
                if (!contains_only_digits)
                    throw std::invalid_argument("input must contain only digits.");
                
                // check if user's input can be represented as an integer
                if (std::stol(answer) > INT_MAX)
                    throw std::invalid_argument("insert a smaller value, so that it can be represented as an integer.");
                
                // NOTE: this can raise std::invalid_argument exception
                start_stop_nodes_idx.first = std::stoi(answer);

                condition = typeid(start_stop_nodes_idx.first) == typeid(int);
                if (!condition)
                    throw std::invalid_argument("insert an integer value.");
                    
            }
            catch(std::exception& e) {
                std::cout << "Input is not valid: " << e.what() << std::endl << std::endl;
                condition = false;
            }
        } while (!condition);

        do {
            std::cout << "Stop node ID: ";
            
            getline(std::cin, answer);
            std::cout << std::endl;

            // if user inserted "-1", the stop node of A* will be chosen randomly
            if (!answer.compare("-1")) {
                start_stop_nodes_idx.second = -1;
                break;
            }

            try {
                // check if user's input contains only digits
                contains_only_digits = std::all_of(answer.begin(), answer.end(), ::isdigit);
                if (!contains_only_digits)
                    throw std::invalid_argument("input must contain only digits.");
                
                // check if user's input can be represented as an integer
                if (std::stol(answer) > INT_MAX)
                    throw std::invalid_argument("insert a smaller value, so that it can be represented as an integer.");
                
                // NOTE: this can raise std::invalid_argument exception
                start_stop_nodes_idx.second = std::stoi(answer);

                condition = typeid(start_stop_nodes_idx.second) == typeid(int);
                if (!condition)
                    throw std::invalid_argument("insert an integer value.");
                    
            }
            catch(std::exception& e) {
                std::cout << "Input is not valid: " << e.what() << std::endl << std::endl;
                condition = false;
            }
        } while (!condition);

        // if "start" and/or "stop" node ID is negative then it will be chosen randomly
        if(start_stop_nodes_idx.first < 0)
            start_stop_nodes_idx.first = UNDEFINED_NODE;

        if(start_stop_nodes_idx.second < 0)
            start_stop_nodes_idx.second = UNDEFINED_NODE;
    }

    std::cout << std::endl;
}


