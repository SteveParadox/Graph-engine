/**
 * @file graph.h
 * @brief Memory-Driven Graph Intelligence Engine - Core Graph Data Structures
 * 
 * This header defines the core graph data structures using:
 * - Self-referential structs
 * - Pointer-based adjacency lists
 * - Double pointer techniques
 * - Dynamic memory management
 */

#ifndef GRAPH_H
#define GRAPH_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <limits.h>

/* ============================================================================
 * CONSTANTS
 * ============================================================================ */

#define INFINITY_DISTANCE INT_MAX

/* ============================================================================
 * CORE DATA STRUCTURES (Pointer Heavy)
 * ============================================================================ */

/* Forward declaration for self-referential structures */
struct Node;
struct Edge;

/**
 * @struct Edge
 * @brief Represents a weighted edge in the graph
 * 
 * Self-referential structure containing:
 * - weight: Edge weight/cost
 * - destination: Pointer to destination node
 * - next: Pointer to next edge in adjacency list
 */
typedef struct Edge {
    int weight;                 // Edge weight for weighted graphs
    struct Node* destination;   // Pointer to destination node
    struct Edge* next;          // Pointer to next edge (linked list)
} Edge;

/**
 * @struct Node
 * @brief Represents a vertex/node in the graph
 * 
 * Self-referential structure with:
 * - id: Unique node identifier
 * - edges: Pointer to linked list of outgoing edges
 * - next: Pointer to next node in graph's node list
 */
typedef struct Node {
    int id;                     // Unique node identifier
    Edge* edges;                // Pointer to adjacency list (linked list of edges)
    struct Node* next;          // Pointer to next node in graph
} Node;

/**
 * @struct Graph
 * @brief Container for the entire graph structure
 * 
 * Maintains a pointer to the head of the node list.
 * All nodes are stored in a linked list for dynamic growth.
 */
typedef struct Graph {
    Node* head;                 // Pointer to first node in graph
    int nodeCount;              // Number of nodes in graph
    int edgeCount;              // Number of edges in graph
} Graph;

/**
 * @struct VisitResult
 * @brief Stores traversal results
 */
typedef struct VisitResult {
    int* visited;               // Array of visited node IDs
    int* distances;             // Array of distances (for Dijkstra)
    int* predecessors;          // Array of predecessors for path reconstruction
    int count;                  // Number of visited nodes
    int capacity;               // Capacity of arrays
} VisitResult;

/* ============================================================================
 * FUNCTION POINTER TYPE DEFINITIONS (Strategy Pattern)
 * ============================================================================ */

/**
 * @typedef SearchStrategy
 * @brief Function pointer type for graph search algorithms
 * 
 * Allows dynamic selection of search algorithms at runtime.
 * This is the Strategy Pattern implemented in C.
 */
typedef void (*SearchStrategy)(Graph* graph, int startId, VisitResult* result);

/**
 * @typedef TraverseCallback
 * @brief Callback function for node traversal
 */
typedef void (*TraverseCallback)(Node* node, void* userData);

/* ============================================================================
 * GRAPH LIFECYCLE FUNCTIONS
 * ============================================================================ */

/**
 * @brief Creates a new empty graph
 * @return Pointer to newly allocated Graph, or NULL on failure
 */
Graph* createGraph(void);

/**
 * @brief Destroys graph and frees all allocated memory
 * @param graph Double pointer to graph (allows NULLing the pointer)
 * 
 * Uses double pointer to modify the caller's pointer after freeing.
 * This prevents dangling pointer issues.
 */
void destroyGraph(Graph** graph);

/**
 * @brief Clears all nodes and edges from graph
 * @param graph Pointer to graph
 */
void clearGraph(Graph* graph);

/* ============================================================================
 * NODE OPERATIONS (Double Pointer Techniques)
 * ============================================================================ */

/**
 * @brief Creates and adds a new node to the graph
 * @param graph Pointer to graph
 * @param id Unique node identifier
 * @return Pointer to created node, or NULL if node exists or allocation fails
 */
Node* addNode(Graph* graph, int id);

/**
 * @brief Creates node using double pointer technique
 * @param headRef Double pointer to head of node list
 * @param id Node identifier
 * 
 * This function modifies the head pointer itself using double indirection.
 */
void createNode(Node** headRef, int id);

/**
 * @brief Finds a node by ID
 * @param graph Pointer to graph
 * @param id Node identifier to search for
 * @return Pointer to found node, or NULL if not found
 */
Node* findNode(Graph* graph, int id);

/**
 * @brief Removes a node and all its incident edges
 * @param graph Pointer to graph
 * @param id Node identifier to remove
 * @return true if node was found and removed, false otherwise
 */
bool removeNode(Graph* graph, int id);

/**
 * @brief Checks if node exists in graph
 * @param graph Pointer to graph
 * @param id Node identifier
 * @return true if node exists, false otherwise
 */
bool hasNode(Graph* graph, int id);

/* ============================================================================
 * EDGE OPERATIONS (Pointer Linking)
 * ============================================================================ */

/**
 * @brief Adds a weighted edge between two nodes
 * @param graph Pointer to graph
 * @param srcId Source node ID
 * @param destId Destination node ID
 * @param weight Edge weight
 * @return true if edge was added successfully
 * 
 * Creates edge in adjacency list using pointer linking.
 * Edge is inserted at head of adjacency list for O(1) insertion.
 */
bool addEdge(Graph* graph, int srcId, int destId, int weight);

/**
 * @brief Adds undirected edge (creates two directed edges)
 * @param graph Pointer to graph
 * @param node1 First node ID
 * @param node2 Second node ID
 * @param weight Edge weight
 * @return true if edge was added successfully
 */
bool addUndirectedEdge(Graph* graph, int node1, int node2, int weight);

/**
 * @brief Removes an edge from source to destination
 * @param graph Pointer to graph
 * @param srcId Source node ID
 * @param destId Destination node ID
 * @return true if edge was found and removed
 */
bool removeEdge(Graph* graph, int srcId, int destId);

/**
 * @brief Checks if edge exists
 * @param graph Pointer to graph
 * @param srcId Source node ID
 * @param destId Destination node ID
 * @return true if edge exists, false otherwise
 */
bool hasEdge(Graph* graph, int srcId, int destId);

/**
 * @brief Gets edge weight between two nodes
 * @param graph Pointer to graph
 * @param srcId Source node ID
 * @param destId Destination node ID
 * @return Edge weight, or -1 if edge doesn't exist
 */
int getEdgeWeight(Graph* graph, int srcId, int destId);

/* ============================================================================
 * GRAPH UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief Gets the number of nodes in graph
 * @param graph Pointer to graph
 * @return Number of nodes
 */
int getNodeCount(Graph* graph);

/**
 * @brief Gets the number of edges in graph
 * @param graph Pointer to graph
 * @return Number of edges
 */
int getEdgeCount(Graph* graph);

/**
 * @brief Traverses all nodes with callback function
 * @param graph Pointer to graph
 * @param callback Function to call for each node
 * @param userData Optional data to pass to callback
 */
void traverseNodes(Graph* graph, TraverseCallback callback, void* userData);

/**
 * @brief Prints graph structure to stdout
 * @param graph Pointer to graph
 */
void printGraph(Graph* graph);

/**
 * @brief Prints graph in adjacency list format
 * @param graph Pointer to graph
 */
void printAdjacencyList(Graph* graph);

/**
 * @brief Creates a deep copy of the graph
 * @param source Pointer to source graph
 * @return Pointer to new graph copy, or NULL on failure
 */
Graph* cloneGraph(Graph* source);

/* ============================================================================
 * VISIT RESULT FUNCTIONS
 * ============================================================================ */

/**
 * @brief Creates a new VisitResult structure
 * @param capacity Initial capacity
 * @return Pointer to VisitResult
 */
VisitResult* createVisitResult(int capacity);

/**
 * @brief Frees VisitResult memory
 * @param result Double pointer to result
 */
void destroyVisitResult(VisitResult** result);

/**
 * @brief Adds a visited node to result
 * @param result Pointer to result
 * @param nodeId Node ID that was visited
 */
void addVisited(VisitResult* result, int nodeId);

/**
 * @brief Prints traversal result
 * @param result Pointer to result
 */
void printVisitResult(VisitResult* result);

/**
 * @brief Reconstructs path from start to target using predecessors
 * @param result Pointer to result with predecessor info
 * @param startId Starting node ID
 * @param targetId Target node ID
 * @param path Output array for path (must be pre-allocated)
 * @param pathLength Output parameter for path length
 * @return true if path exists
 */
bool reconstructPath(VisitResult* result, int startId, int targetId, 
                     int* path, int* pathLength);

/* ============================================================================
 * MEMORY MANAGEMENT UTILITIES
 * ============================================================================ */

/**
 * @brief Gets total allocated memory for graph
 * @param graph Pointer to graph
 * @return Total bytes allocated
 */
size_t getGraphMemoryUsage(Graph* graph);

/**
 * @brief Validates graph structure integrity
 * @param graph Pointer to graph
 * @return true if graph is valid
 */
bool validateGraph(Graph* graph);

#endif /* GRAPH_H */
