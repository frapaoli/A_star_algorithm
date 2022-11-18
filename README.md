# A* algorithm
Single and multi-threaded implementation of A* algorithm for optimal path planning.





# Design

## Graph

A graph is a set of nodes connected by a weighted link.

In code, it is represented by a vector of `Node`.
```c++
class Node {
    unsigned int id;    // node's ID
    int x;              // node's coordinate along x axis
    int y;              // node's coordinate along y axis
    std::unique_ptr<std::unordered_map<unsigned int, unsigned int>> neighbor;     // node's links to neighbor nodes
};
```

Each `Node` is characterized by an unique id and two integers for the coordinates that define the node position.
The links are represented by listing, for every node, all the nodes it is linked to. This is done with an unordered map `neighbor` with all the linked nodes as a key and the cost of the link as value.
The node in `neighbor` is represented as an unsigned integer and represents the position of the corresponding node in the graph vector.


## Graph generation
Since the graph can have up to millions of nodes and therefore the graph size can become quite big, we decide to implement the graph generation (and loading) in a multithreaded way. 


## Graph loading



## Algorithms
We decided to implement 3 different type of A* star algorithms, each one with different parallelization level:
- `Sequential A*` that is the classical algorithm and runs on a single thread,
- `Centralized A*` that runs on multiple threads but all the data structures are shared between them and protected by locks
- `Decentralized A*` that is a more optimized parallel algorithm where locks and shared resources are reduced at a minimum

All the algorithms have at least 3 inputs that are the same:
- The `graph` object that represent the graph in which we want to find the best path and it is a vector of nodes
- The `start` and `end` nodes between which we want to find the path in the graph
- The `best_path` object where the solution will be stored.

The `best_path` object contains:
- an unordered map that has nodes as keys and values and it stores the path in order from start to end
- the total cost of the found path
- the number of nodes contained in the path


### Sequential A* star

The sequential A start, as the name indicates, runs on a single thread.

Internally, it uses 4 data structures:
- `open` is a priority queue storing the nodes that still need to be visited together with its g cost and f cost. The nodes are ordered by g (and f) so that the node with lower g (or f in case g is the same) is always on top. Because the std implementation of priority queues doesn't allow the edit of nodes already inserted, nodes inside the queue can be duplicated (but with different costs). The node with lower cost is always picked up first and any other duplicate will be discarded when it's their turn to be removed from the open list.
- `closed` is an unordered map used to keep track of nodes that have been already visited. This structure is used to discard duplicates nodes in the open list like described above
- `from` is an unordered map having as key and value a node and it is used to keep track of the current best path from start to end. It will be read at the end to reconstruct the whole path
- `cost` is an unordered map having as key a node and as value an integer representing the current best cost to arrive at the specified node.

The code is pretty straight forward. We just pop the node from the open list (with lower cost g) and check if the node has already been visited. If it was, we just discard it, otherwise we check if it is the end node.
If it is, we are guaranteed we have found the best path and therefore we reconstruct the path using `from` and return.


### Centralised A* star

Centralized A* star is a simple attempt to parallelize the sequential algorithm.
It uses the same data structures of the sequential algorithm (plus a vector `end` described below) and it shares them between multiple threads.
The code run by each thread is pretty similar to the sequential case but with 2 main differences.
First, since data structures can now be accessed concurrently by multiple threads, they need to be protected. For this reason, we have to add 2 locks:
- m0 for the open set, closed set and the end vector
- m1 for all the others


Second, unlike the sequential algorithm, in this case finding a path from start to end does not guarantee it is the shortest. For this reason, we have to continue to open new nodes until either we don't have any nodes left in the open list or the cost of the opened nodes exceeds the cost of the current best path.
Those conditions must be true for all the threads at the same time. For this reason, there is the need of an algorithm for threads to collectively decide when the computation is finished and the best path has been found.
In the case of the centralized A* star, the termination algorithm is very simple. It uses a shared vector of bool `end` of N elements where N is the number of threads. When all conditions are met, a thread sets `end[thread_id]` to true and to false otherwise. When all elements in `end` are true, the algorithm terminates and all threads exit.

### Decentralized A* star

 Decentralized A* star is a parallel implementation of the algorithm that aims to be more efficient than the centralized version by removing the overhead of the locks and the use of shared data structures.

 To achieve this goal, each thread has its own open and closed lists and it runs the A* star algorithm locally on those structures. 
When visiting 

The work is split evenly between threads thanks to the use of a deterministic hash function. When a thread visits a new node and explores its neighbors, it calculates the hash of each of them and then…



## Experimental evaluation



