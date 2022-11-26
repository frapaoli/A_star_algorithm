# A* algorithm
Single and multi-threaded implementation of A* algorithm for optimal path planning.



# 1. Introduction

A* is a path search algorithm for finding the optimal-cost path that connects any `start` node to any `stop` node of a directed, weighted graph (if such path exists).
The following documentation aims to guide the user through the C/C++ implementation of single-thread and multi-thread versions of A* algorithm, highlighting the main design choices that have been made and the experimental results that have been achieved.

# 2. Data structures and algorithms design

For a better understanding of the documentation, the main notations used to describe the algorithm design procedure have been reported below:
- `graph`: set of nodes connected between them by directed, weighted links.
- `Node`: data structure representing the graph nodes (more details on this later).
- `start` and `stop`: source and destination nodes of the A*, respectively.
- `neighbors` of a node: nodes to which it is connected through a directed, weighted link.
- `g_cost` of a node: cumulative cost (i.e., weight) of all the links that have to be traversed from `start` in order to arrive at that node.
- `h_cost` of a node: estimation of the cost of the links that must be traversed from that node to arrive at `stop`, which is computed by an heuristic (more details on this later).
- `f_cost` of a node: `g_cost` + `h_cost` of that node.

## 2.1 Graph structures

The first implementation choice was to represent a graph as a `std::vector` structure of `Node` objects, whose main features are reported below:
```c++
// set of links (and corresponding weights) that connect a node with its neighbors
typedef std::unordered_map<unsigned int, unsigned int> link_weight_umap;

class Node {
    unsigned int id;    // node's ID
    int x;              // node's coordinate along x axis
    int y;              // node's coordinate along y axis
    std::unique_ptr<std::unordered_map<unsigned int, unsigned int>> neighbor;     // node's links to neighbor nodes
    …
};
```
where:
- `id` is the unique identifier of each node on the graph.
- `x` and `y` are the coordinates of the node on the 2D grid onto which the graph is built.
- `neighbor` is a pointer to a `std::unordered_map` structure containing a set of key-value pairs representing the `id` of the nodes to which the `neighbor` structure owner is connected and the cost of the corresponding links, respectively.


Each `Node` is characterized by an unique ID and two coordinates that define the node position.
The links are represented by listing, for every node, all the nodes it is linked to. This is done with an unordered map `neighbor` with all the linked nodes as a key and the cost of the link as value.
The node in `neighbor` is represented as an unsigned integer and represents the position of the corresponding node in the graph vector.


## Graph generation
Since the graph can have up to millions of nodes and therefore the graph size can become quite big, we decide to implement the graph generation (and loading) in a multithreaded way. 
…

## Graph loading
…


## Algorithms
We decided to implement 3 different type of A* algorithms, each one with different parallelization level:
- `Sequential A*` that is the classical algorithm and runs on a single thread,
- `Centralized A*` that runs on multiple threads but all the data structures are shared between them and protected by locks
- `Decentralized A*` that is a more optimized parallel algorithm where locks and shared resources are reduced at a minimum

All the algorithms have at least 3 inputs that are the same:
- The `graph` object that represent the graph in which we want to find the best path
- The `start` and `end` nodes between which we want to find the path in the graph
- The `best_path` object where the solution will be stored.

The `best_path` object contains:
- an unordered map that has nodes as keys and values and it stores the path in order from start to end
- the total cost of the found path
- the number of nodes contained in the path


### Sequential A*

The sequential A*, as the name indicates, runs on a single thread.

Internally, it uses 4 data structures:
- `open` is a priority queue storing the nodes that still need to be visited together with its g cost and f cost. Since the open list needs fast insertion and fast removal of the lower cost node, we chose the priority queue because the nodes are always ordered by g (and f in case g is the same). Because the std implementation of priority queues doesn't allow the edit of nodes already inserted, nodes inside the queue can be duplicated (but with different costs). The node with lower cost is always picked up first and any other duplicate will be discarded when it's their turn to be removed from the open list.
- `closed` is an unordered map used to keep track of nodes that have been already visited. This structure is used to discard duplicates nodes in the open list like described above
- `from` is an unordered map having as key and value a node and it is used to keep track of the current best path from start to end. It will be read at the end to reconstruct the whole path
- `cost` is an unordered map having as key a node and as value an integer representing the current best cost to arrive at the specified node. We chose to use an unordered map as the algorithm only requirement is a data structure that is easy to loop through.

The sequential algorithm code is pretty simple. As a first step, we add the start node to the open list and then we do in while loop (until the open set is not empty) the following steps:
- We get the top node from the open list and check if we already visited it and therefore needs to be discarded
- If the node is the last of the path, we reconstruct the path and the algorithm ends
- We loop through the neighbors and, if they are not present in the cost map or the current cost is less than the map cost, we add it to the cost, from and open data structures.

If we exit the while loop because the open list is empty but no solution has been found, it means that the `start` and `end` nodes are not connected.

### Centralized A*

Centralized A* is a simple attempt to parallelize the sequential algorithm.
It uses the same data structures of the sequential algorithm (plus a vector `end` described below) and it shares them between multiple threads.
The code run by each thread is pretty similar to the sequential case but with 2 main differences.
First, since data structures can now be accessed concurrently by multiple threads, they need to be protected. For this reason, we have to add 2 locks:
- m0 for the open set, closed set and the end vector
- m1 for all the others

We tried to limit the number of locks as much as possible to reduce the associated overhead while still trying to have some kind of parallelism (?)


Second, unlike the sequential algorithm, in this case finding a path from start to end does not guarantee it is the shortest. For this reason, we have to continue to open new nodes until either we don't have any nodes left in the open list or the cost of the opened nodes exceeds the cost of the current best path.
Those conditions must be true for all the threads at the same time. For this reason, there is the need of an algorithm for threads to collectively decide when the computation is finished and the best path has been found.
In the case of the centralized A*, the termination algorithm is very simple. It uses a shared vector of bool `end` of N elements where N is the number of threads. When all conditions are met, a thread sets `end[thread_id]` to true and to false otherwise. When all elements in `end` are true, the algorithm terminates and all threads exit.

### Decentralized A*

 Decentralized A* is a parallel implementation of the algorithm that aims to be more efficient than the centralized version by removing the overhead of the locks and the use of shared data structures.

 To achieve this goal, each thread has its own open and closed lists and it runs the A* star algorithm locally on those structures. 

The work is split evenly between threads thanks to the use of a deterministic hash function. When a thread visits a new node and explores its neighbors, it calculates the hash of each of them and sends them to the respective thread.
Since the hash is deterministic, the same node will always be sent to the same thread.

We have 3 kind of hashing algorithms:
-
-
-

The algorithm implementation is composed by the following steps, which are being executed in parallel by each thread:
1. Fill up the random-bit-strings table R (spiegare brevemente perché serve)
2. Check if the current thread has no useful task to be accomplished other than sleeping on a condition variable (to save CPU computation)
3. Check if there is any message to be read by the current thread, which would eventually contain the info about a new node to be potentially expanded
4. Expand a node in the open list (if is there any that is actually worth expanding)




## Experimental evaluation




