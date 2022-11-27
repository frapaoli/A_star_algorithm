# A* algorithm project

A* is a path search algorithm for finding the optimal-cost path that connects any `start` node to any `stop` node of a directed, weighted graph (if such path exists).
The following documentation aims to guide the user through the C/C++ implementation of single-thread and multi-thread versions of A* algorithm, highlighting the main design choices that have been made and the experimental results that have been achieved.

# 1. Graph data structures and algorithms design

For a better understanding of the documentation, the main notations used to describe the algorithm design procedure have been reported below:
- `graph`: set of nodes connected between them by directed, weighted links.
- `Node`: data structure representing the graph nodes (more details on this later).
- `start` and `stop`: source and destination nodes of the A*, respectively.
- `neighbors` of a node: nodes to which it is connected through a directed, weighted link.
- `g_cost` of a node: cumulative cost (i.e., weight) of all the links that have to be traversed from `start` in order to arrive at that node.
- `h_cost` of a node: estimation of the cost of the links that must be traversed from that node to arrive at `stop`, which is computed by an heuristic (more details on this later).
- `f_cost` of a node: `g_cost` + `h_cost` of that node.
- `num_nodes`: number of graph nodes
- `num_threads`: number of threads that are running concurrently.

## 1.1 Graph data structures

The first implementation choice was to represent a graph as a `std::vector` structure of `Node` objects, whose main features are reported below:
```c++
// set of links (and corresponding weights) that connect a node with its neighbors
typedef std::unordered_map<unsigned int, unsigned int> link_weight_umap;

/******************** Graph's nodes structure ********************/
class Node {

    unsigned int id;    // node's ID
    int x;              // node's coordinate along x axis
    int y;              // node's coordinate along y axis
    std::unique_ptr<link_weight_umap> neighbor;     // node's links to neighbor nodes
    …
};
```
where:
- `id` is the unique identifier of each node on the graph.
- `x` and `y` are the coordinates of the node on the 2D grid onto which the graph is built.
- `neighbor` is a pointer to a `std::unordered_map` structure containing a set of key-value pairs representing the `id` of the nodes to which the `neighbor` structure owner is connected and the cost of the corresponding links, respectively.

_**Highlighted implementation choices**_:
- `neighbor` is a pointer to a `std::unordered_map` and not a `std::unordered_map` itself because we wanted each `Node` object to be of constant size.
- Each `Node` has been provided with full copy control features (i.e., constructor, copy/move constructor, copy/move assignment, destructor), overloaded operators and attributes getters/setters, which have been implemented in `graph_gen_store_load.cpp/.h` files.

## 1.2 Graph generation and storage
The first task was to implement a graph generation algorithm in order to generate a graph (having up to millions of nodes and links) and to store it in a file, from which it can then be retrieved whenever needed.
Considering the potentially huge size of the graph, it has been decided to implement the algorithm in a parallel way, so that it could be executed both in single-thread and multi-thread.
Through the user interface provided by `menu.cpp/.h` files, the user can choose to run the program in order to generate a random graph of a given size (i.e., number of nodes), as well as the number of threads that should concurrently generate the graph’s nodes and links and store them in a long-term memory file.

### 1.2.1 Details about the graph generation procedure
The function `graph_generation` called by the main thread initializes all the data structures and synchronization primitives needed for the graph generation and then, for each thread that has to run concurrently, an instance of the function `nodes_links_generation` gets launched in order to generate the graph in parallel. Both above mentioned functions are in `graph_gen_store_load.cpp/.h`.

For efficient threads synchronization, the following primitives have been employed:
- `std::mutex` structures to manage critical code sections that need to be mutually exclusive.
- `pthread_mutex_t` and `sem_t` structures to implement barriers that allow all threads to “re-align” in a given point in code execution before proceeding further.
- `std::atomic_flag` to allow threads to lock certain graph partitions that divide the original graph in order to guarantee efficient thread parallelism.  

_**Highlighted implementation choices**_:
- To let threads lock graph partitions, `std::atomic_flag` structures were employed instead of `std::mutex` because, when a certain partition gets locked and “processed” by a thread, the same partition doesn’t need to be processed by any other thread, hence the `std::atomic_flag::test_and_set` method was used for this purpose. The same could be done by `std::mutex::try_lock` method that, however, according to the C++ reference documentation (https://en.cppreference.com/w/cpp/thread/mutex/try_lock) is allowed to fail spuriously: _\<\<This function is allowed to fail spuriously and return false even if the mutex is not currently locked by any other thread.\>\>_

### 1.2.2 Nodes generation
At the beginning of nodes generation, the graph `x`-`y` coordinate space gets evenly divided in a certain number of partitions such that it becomes a 2D square grid where each grid cell is a graph partition having certain `x`-`y` bounds. The number of created partitions is proportional to `num_nodes` and gets computed by the `optimal_graph_axes_partit_size` function in `utilities.cpp/.h` files.

Then, all threads run concurrently to lock graph partitions and, when one gets locked, the corresponding thread generates inside it a certain number of nodes having `x` and `y` coordinates within the partition boundaries. The number of nodes inside each partition gets calculated such that, at the end, the overall number of nodes will be equal to the `num_nodes` specified by the user.

_**Highlighted implementation choices**_: 
- `optimal_graph_axes_partit_size` function computes the optimal `graph_axes_partit` (i.e., number of graph partitions to be created along `x` and `y` coordinates) in order to have the least overall number of links in the graph to be generated. This is just an implementation choice made in order to obtain a graph where it is “challenging” to find a path between two nodes and to reduce the time needed for the graph generation, but any choice of `graph_axes_partit` strictly greater than zero is totally fine.
- Excluding cases in which `num_threads` is greater than `num_nodes` (which is very unusual), the number of graph partitions is sufficiently greater than `num_threads` so that faster threads can lock multiple partitions and slower threads don’t become the bottleneck, but it is also not too big in order to avoid excessively increasing the overhead due to threads parallelism.


### 1.2.3 Links generation

Each thread waits on a barrier for all the others to finish the nodes generation and, when all threads are done, they go through all the graph partitions a second time in order to generate the links of the nodes belonging to each partition.

Links get created between nodes both belonging to the same graph partition and belonging to different but adjacent partitions. In order to decide the number of links to be generated and their weight, the following arbitrary choices have been made:
- The number of links between nodes belonging to the same graph partition is random and proportional to the `log2` of the number of nodes inside that partition.
- The number of links going from the nodes of a partition _A_ to the nodes of another partition _B_ adjacent to _A_ is deterministic and proportional to the `sqrt` of the number of nodes inside partition _B_.
- The weight of each link is equal to the Euclidean distance on the `x`-`y` plane between the two nodes that the links connect, multiplied by a random `float` factor between 1 and 2.

_**Highlighted implementation choices**_:
- It has been chosen to generate at least one incoming link and one outgoing link for every node, so that the resulting graph is strongly connected. Therefore, the proposed graph generation algorithm creates graphs for which there always exists a path between every `start` and `stop` node couple.


### 1.2.4 How the graph gets stored in long-term memory

When all threads are done with the links generation as well (which is guaranteed by a barrier), the graph data structure can be serialized (i.e., stored) into the file specified by the user through the terminal command window. The graph serialization gets performed in binary format through the `write` system call, so that the graph structure occupies less bytes in the disk.

The main steps of the graph storing algorithm are the following:
1. One thread stores at the beginning of the file the information regarding nodes and links, which are:
* 4 bytes `unsigned int` for the number of generated nodes.
* 4 bytes `unsigned int` for the number of generated links.
* 4 bytes `unsigned int` for the number of graph partitions.
* For each graph partition, 4 bytes `unsigned int` for the number of links that start from that partition.
2. Threads concurrently go through every graph partition and, for each of them, store in the file the information regarding each node of that partition, which are:
* 4 bytes `unsigned int` for the node `id`.
* 4 bytes `int` for the node `x` coordinate.
* 4 bytes `int` for the node `y` coordinate.
3. Same as step 2., but now threads store in the file the information regarding the links of each node of graph partitions, which are:
* 4 bytes `unsigned int` for the `id` of the node from which the link starts.
* 4 bytes `unsigned int` for the `id` of the node in which the link ends.
* 4 bytes `unsigned int` for the link’s weight.


_**Highlighted implementation choices**_:
- In order to preserve the correctness of the content that gets written into the file, the three steps mentioned above are separated by thread barriers.
- `lseek` and `fcntl` system calls have been employed to, respectively, locate the exact file region where to write and to ensure that, when a thread is writing on a certain file region, no other thread is allowed to write on the same region.
- Each thread, after writing something in the file, checks the return value of the `write` system call in order to guarantee that the number of written bytes is actually correct.




## 1.3 Graph loading
Once the algorithm for generating the graph and storing it in a file has been completed, it was necessary to implement a second algorithm that, given the name of a file storing a graph in the long-term memory, is able to rebuild it from the information stored in the file and to load it in the memory of the running program, so that A* could be executed on it.
As for the graph generation, considering the potentially huge size of the graph, it has been decided to implement the graph loading algorithm in a parallel way, so that it could be executed both in single-thread and multi-thread.
Through the user interface provided by `menu.cpp/.h` files, the user can choose to run the program in order to load a graph from a file by specifying the number of threads that should concurrently read the graph’s nodes and links from the files. During this process, the user gets also asked about which A* algorithm versions should be run on the graph after it gets successfully read (more details on this later).


### 1.3.1 Details about the graph loading procedure
The function `graph_read_from_file` called by the main thread initializes all the data structures and synchronization primitives needed for the graph loading and then, for each thread that has to run concurrently, an instance of the function `nodes_links_read_from_file` gets launched in order to read the graph in parallel. Both above mentioned functions are in `graph_gen_store_load.cpp/.h`.

For efficient threads synchronization, the same primitives already discussed in [Section 1.2.1](#121-details-about-the-graph-generation-procedure) have been employed.

The deserialization of the graph information contained in the file is performed by following the same protocol used for the serialization of graph in file, whose details are described in [Section 1.2.4](#124-how-the-graph-gets-stored-in-long-term-memory). In particular:
1. The main thread reads the information regarding the number of nodes and links at the beginning of the file.
2. All threads concurrently go through every graph partition and, for each of them, read from the file the information regarding each node of that partition.
3. Same as step 2., but now threads read from the file the information regarding the links of each node of graph partitions.

As for the graph generation, each of the above steps of the graph loading gets performed by:
- locking through the `fcntl` system call the file regions that are currently being read by a thread.
- checking every time the return value of the `read` system call to ensure that the correct number of bytes have been actually read from the file.

Once this procedure has been carried out successfully, the graph will be found in a `std::vector<Node>` that is ready to be used by the A* algorithm.




# 2. A* data structures and algorithms design
Considering the state-of-the-art approaches for A* carried out in the literature of path search algorithms, it has been decided to implement 3 different versions of the A* algorithm, of which one is single-thread and the other two allow multi-thread execution:
- `Sequential A*` that is the classical A* algorithm running on a single thread.
- `Centralized A*` that runs on multiple threads but all the data structures are shared between them and protected by appropriate synchronization primitives.
- `Decentralized A*` that is a highly optimized parallel version where each thread has its own data structures, so that locks and resources shared among all threads are reduced as much as possible.

Details about each one of the above mentioned versions can be seen in the `a_star.cpp/.h` files and are presented in following sections.



## 2.1 Sequential A*
The Sequential A*, as the name suggests, runs on a single thread in a sequential way, so that graph nodes are explored one at a time until the best path (i.e., the path with the least cost) between `start` and `stop` nodes is found.

### 2.1.1 Sequential A* data structures
The following is a list of the main data structures exploited by the Sequential A* in order to carry out a correct algorithm execution:
- `graph`: a `std::vector<Node>` instance that represents the graph on which we want to run the A*.
- `start` and `stop`: nodes of `graph` between which we want to find the best path.
- `best_path`: object where the best path information will be stored, i.e., the sequence of nodes that makes up the path, how many these nodes are and the overall cost of the path. It is an instance of the `graph_path_t` data structure, which is defined as follows:
```c++
// set of nodes in a graph's path (the order in which the nodes must be traversed is given by the key of the hash map)
typedef std::unordered_map<unsigned int, Node> path_umap;

/******************** Path (i.e. sequence of nodes) of the graph ********************/
typedef struct graph_path_s {

    std::unique_ptr<path_umap> path_ptr;    // path itself
    unsigned int path_num_nodes;            // number of nodes contained in the path
    std::atomic<int> path_cost;             // path's overall cost

} graph_path_t;
```

- `open`: a `std::priority_queue` instance that stores the nodes that still need to be visited, together with their `f_cost` and `h_cost`. Each `open` list entry has the following structure:
```c++
// tuple that, given a node 'N', stores f(N), h(N) and N itself
typedef std::tuple<double, double, Node> list_elem;
```

- `closed`: a `std::unordered_map` instance used to keep track of nodes that have been already visited. Its key-value pairs are made out of, respectively, the `id` of the nodes and their corresponding `list_elem` structure, which is shown above.
- `from`: a `std::unordered_map` instance having as key-value pairs a `Node` (called _child_ node) and its parent (i.e., the node from which we arrived at the child node), respectively. It is used to keep track of the current best path from `start` to `stop`. It will be read at the end to reconstruct the whole path.
- `cost`: a `std::unordered_map` instance having as key-value pairs a `Node` and an `unsigned int` representing the current best cost to arrive at it, respectively.


_**Highlighted implementation choices**_:
- For the `open` structure it has been chosen to use a `std::priority_queue` because it allows fast insertion and fast removal of the lower cost node (both with _O(1)_ complexity), which is what is needed. As a drawback, `std::priority_queue` doesn't allow random access (i.e., doesn’t allow editing nodes already inserted in it), so we cannot update the cost of a node when we find a lower cost path to arrive at it. However, a simple and effective workaround of this is to just duplicate the node in the `open` list with different costs and, thanks to the `std::priority_queue` properties mentioned above, it is guaranteed that the node copy having the lower cost will be always picked up first, and all other duplicates of the same node will be discarded when it's their turn to be removed from the `open` list (we keep track of the nodes already expanded in the `closed` list).
- The heuristic used to compute the `h_cost` of each node is the Euclidean norm because, besides its simplicity, it is an admissible heuristic (i.e., it never overestimates the distance to the `stop` node) and a consistent heuristic (i.e., satisfies the triangle inequality). An admissible and consistent heuristic guarantees that, when a node gets expanded for the first time, it will be with the lowest possible cost. This statement also implies that, when the Sequential A* reaches for the first time the `stop` node, then the algorithm can terminate because it is for sure the path with the lowest cost. However, the above discussion stands in general only for the Sequential A* and _not_ for the Centralized and Decentralized A* (more details on this later).
- For the `closed`, `from` and `cost` structures we chose the `std::unordered_map` data type because we wanted constant computational complexity to perform random access on them.


### 2.1.2 Details about the Sequential A* implementation
The sequential implementation of A* is straight forward: starting from the user specified `start` node, a single thread progressively expands all the lowest cost neighbors of the already visited nodes, until the `stop` node is eventually found.

The main algorithm steps are described below:
1. Extract from the top of the `open` list the node that is currently reachable with the lowest cost.
2. Check if the extracted node is already in the `closed` list: if yes then discard it (we already visited that node with a lower cost), otherwise expand it and continue.
3. Check if the expanded node is `stop`: if yes then the algorithm terminates and the `rebuild_path_single_thread` function gets called in order to rebuild the path going backward from `stop` to `start`, otherwise continue.
4. For each neighbor of the node just expanded, check if that neighbor is worth expanding too: if yes then add it to the `open` list, otherwise discard it.
5. Loop through the steps 1-4 until the `stop` node is found or until there are no more nodes in the `open` list to be expanded. The latter circumstance would mean that there exists no path that connects `start` to `stop`.










### Centralized A*

Centralized A* is a simple attempt to parallelize the sequential algorithm.
It uses the same data structures of the sequential algorithm (plus a vector `end` described below) and it shares them between multiple threads.
The code run by each thread is pretty similar to the sequential case but with 2 main differences.
First, since data structures can now be accessed concurrently by multiple threads, they need to be protected. For this reason, we have to add 2 locks:
- m0 for the open set, closed set and the end vector
- m1 for all the others

We tried to limit the number of locks as much as possible to reduce the associated overhead while still trying to have some kind of parallelism (?)

the assumption of admissible + consistent h_cost is not anymore valid (also for Decentralized A*)





Second, unlike the sequential algorithm, in this case finding a path from start to end does not guarantee it is the shortest. For this reason, we have to continue to open new nodes until either we don't have any nodes left in the open list or the cost of the opened nodes exceeds the cost of the current best path.
Those conditions must be true for all the threads at the same time. For this reason, there is the need of an algorithm for threads to collectively decide when the computation is finished and the best path has been found.
In the case of the centralized A*, the termination algorithm is very simple. It uses a shared vector of bool `end` of N elements where N is the number of threads. When all conditions are met, a thread sets `end[thread_id]` to true and to false otherwise. When all elements in `end` are true, the algorithm terminates and all threads exit.



## 2.3 Decentralized A*

The Decentralized A* is a parallel implementation of the algorithm that aims at being more efficient and less time-consuming than the Centralized A* version by removing the overhead due to the locks and to the usage of common data structures shared among all threads.

To achieve this goal, each thread has its own `open` and `closed` lists and it runs the A* algorithm locally using those structures in such a way that the overall knowledge of the graph will be distributed (i.e., each thread will have information regarding only a sub-part of the explored graph) and _not_ centralized as it happens for the Sequential and Centralized A* versions.

In particular, the proposed algorithm implements the Hash Distributed A* (HDA*), a decentralized version of A* that evenly splits between threads the algorithm workload by relying on ad-hoc hash functions.

**NOTE**: from now on we will refer to a thread as the _owner_ of a node if the chosen hash function applied to that node returns the `id` of that thread. This means that the hashed thread is responsible for (eventually) expanding that node.

_**Highlighted implementation choices**_:
- The option of assigning the ownership of a node to a thread by using a random generator of nodes `id` was discarded in the first place because, in order to properly carry out a decentralized execution of the A*, it is needed that each node that we hash always returns the `id` of the same node in a deterministic way (i.e., each node must be owned by only one thread).


### 2.3.1 Decentralized A* hashing methods
Looking at the literature, it has been decided to implement 3 different hashing methods to be used by the HDA* where each one focuses on improving a specific feature of the hashing task, so that they could be compared with each other.

The 3 hashing methods are described below:
1. Multiplicative hash (_MHDA*_): given a `Node`, the `id` of its owner gets computed by:
```c++
(int) (num_threads * (k*A - (int) (k*A)))
```
where:
- `A` can be any positive number. For this implementation we chose the golden ratio $\frac{1 + \sqrt{5}}{2}$ since it has been shown in the literature that it works quite well for the MHDA*.
- `k` is a value obtained by hashing the `x` and `y` coordinates of the given `Node`. The workload distribution effectiveness of the MHDA* strongly relies on the design of a good hash function that computes `k` from `x` and `y`, which is not a trivial task.

2. Zobrist hash (_ZHDA*_): given a `Node`, the `id` of its owner gets computed by:
$$R[x’] \oplus R[y’]$$
where:

_**Considerations about the proposed hashing methods**_:
1. 




### 2.3.2 Decentralized A* data structures




The algorithm implementation is composed by the following steps, which are being executed in parallel by each thread:
1. Fill up the random-bit-strings table R (spiegare brevemente perché serve)
2. Check if the current thread has no useful task to be accomplished other than sleeping on a condition variable (to save CPU computation)
3. Check if there is any message to be read by the current thread, which would eventually contain the info about a new node to be potentially expanded
4. Expand a node in the open list (if is there any that is actually worth expanding)


### 2.3.4 Decentralized A* path rebuild
rebuild_path_single_thread
rebuilds the path from "start" to "stop" node in case a single thread has all the information needed to do it (not possible in the Decentralized A*).

for rebuilding the path we had two options:
1. send a circular message (too expensive)
2. (the method we actually used)



ansynch communication among threads to avoid wasting time


### 2.3.4 Decentralized A* termination condition
As for the Centralized A*, the assumption of admissible + consistent h_cost is not anymore valid.








# 3. Experimental performance evaluation

after every section (graph gen, read, A* etc) it gets printed the time elapsed in order to evaluate performance

(evaluation of memory usage?) write also the command used (on Ubuntu):
watch -n 0.5 free -m


centr A* exhibits slower runtimes than seq A* because the concurrent access to the open list is the bottleneck, which makes the scalability of the centr A* limited. If expanding a node would have been a lot more expensive than what is now, centr A* would have probably be faster than seq A* because the access to the open list would have not been anymore the bottleneck.










# TODO delete

Each `Node` is characterized by an unique ID and two coordinates that define the node position.
The links are represented by listing, for every node, all the nodes it is linked to. This is done with an unordered map `neighbor` with all the linked nodes as a key and the cost of the link as value.
The node in `neighbor` is represented as an unsigned integer and represents the position of the corresponding node in the graph vector.


Since the graph can have up to millions of nodes and therefore the graph size can become quite big, we decide to implement the graph generation (and loading) in a multithreaded way. 

All the algorithms have at least 3 inputs that are the same:
- The `graph` object that represent the graph in which we want to find the best path
- The `start` and `end` nodes between which we want to find the path in the graph
- The `best_path` object where the solution will be stored.

The `best_path` object contains:
- an unordered map that has nodes as keys and values and it stores the path in order from start to end
- the total cost of the found path
- the number of nodes contained in the path

Since the open list needs fast insertion and fast removal of the lower cost node, we chose the priority queue because the nodes are always ordered by g (and f in case g is the same). Because the std implementation of priority queues doesn't allow the edit of nodes already inserted, nodes inside the queue can be duplicated (but with different costs). The node with lower cost is always picked up first and any other duplicate will be discarded when it's their turn to be removed from the open list.


h_cost is computed with the Euclidean norm, so that the heuristic is admissible (...) and consistent (...) and so we are sure that in sequential case the first solution is the optimal one (anything else???)


- `cost`: is an unordered map having as key a node and as value an integer representing the current best cost to arrive at the specified node. We chose to use an unordered map as the algorithm only requirement is a data structure that is easy to loop through.


The sequential algorithm code is pretty simple. As a first step, we add the start node to the open list and then we do in while loop (until the open set is not empty) the following steps:
- We get the top node from the open list and check if we already visited it and therefore needs to be discarded
- If the node is the last of the path, we reconstruct the path and the algorithm ends
- We loop through the neighbors and, if they are not present in the cost map or the current cost is less than the map cost, we add it to the cost, from and open data structures.

If we exit the while loop because the open list is empty but no solution has been found, it means that the `start` and `end` nodes are not connected.


The work is split evenly between threads thanks to the use of a deterministic hash function. When a thread visits a new node and explores its neighbors, it calculates the hash of each of them and sends them to the respective thread.
Since the hash is deterministic, the same node will always be sent to the same thread.



