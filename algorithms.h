/**
 * @file algorithms.h
 * @brief Graph Search Algorithms - BFS, DFS, Dijkstra
 * 
 * Implements three fundamental graph algorithms:
 * - Breadth-First Search (BFS) using queue
 * - Depth-First Search (DFS) using recursion
 * - Dijkstra's Shortest Path using min-heap
 */

#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "graph.h"
#include "heap.h"

/* ============================================================================
 * ALGORITHM CONFIGURATION
 * ============================================================================ */

/* INFINITY_DISTANCE is defined in graph.h */
#define MAX_NODES 1000

/* ============================================================================
 * BREADTH-FIRST SEARCH (BFS)
 * ============================================================================ */

/**
 * @brief Performs Breadth-First Search from starting node
 * @param graph Pointer to graph
 * @param startId Starting node ID
 * @param result Output structure for traversal results
 * 
 * Uses queue data structure (implemented with linked list).
 * Time Complexity: O(V + E)
 * Space Complexity: O(V)
 */
void bfs(Graph* graph, int startId, VisitResult* result);

/**
 * @brief BFS that stops when target is found
 * @param graph Pointer to graph
 * @param startId Starting node ID
 * @param targetId Target node to find
 * @param result Output structure
 * @return true if target was found
 */
bool bfsFind(Graph* graph, int startId, int targetId, VisitResult* result);

/* ============================================================================
 * DEPTH-FIRST SEARCH (DFS)
 * ============================================================================ */

/**
 * @brief Performs Depth-First Search from starting node
 * @param graph Pointer to graph
 * @param startId Starting node ID
 * @param result Output structure for traversal results
 * 
 * Uses recursion for traversal.
 * Time Complexity: O(V + E)
 * Space Complexity: O(V) for recursion stack
 */
void dfs(Graph* graph, int startId, VisitResult* result);

/**
 * @brief Iterative DFS using explicit stack
 * @param graph Pointer to graph
 * @param startId Starting node ID
 * @param result Output structure
 */
void dfsIterative(Graph* graph, int startId, VisitResult* result);

/**
 * @brief DFS that stops when target is found
 * @param graph Pointer to graph
 * @param startId Starting node ID
 * @param targetId Target node to find
 * @param result Output structure
 * @return true if target was found
 */
bool dfsFind(Graph* graph, int startId, int targetId, VisitResult* result);

/* ============================================================================
 * DIJKSTRA'S SHORTEST PATH ALGORITHM
 * ============================================================================ */

/**
 * @brief Finds shortest paths from start node to all other nodes
 * @param graph Pointer to graph
 * @param startId Starting node ID
 * @param result Output with distances and predecessors
 * 
 * Uses min-heap priority queue for O((V+E) log V) complexity.
 * Handles positive weights only.
 */
void dijkstra(Graph* graph, int startId, VisitResult* result);

/**
 * @brief Finds shortest path between two specific nodes
 * @param graph Pointer to graph
 * @param startId Starting node ID
 * @param targetId Target node ID
 * @param distance Output for shortest distance
 * @param path Output array for path (pre-allocated)
 * @param pathLength Output for path length
 * @return true if path exists
 */
bool dijkstraShortestPath(Graph* graph, int startId, int targetId,
                          int* distance, int* path, int* pathLength);

/**
 * @brief Dijkstra with early termination when target is reached
 * @param graph Pointer to graph
 * @param startId Starting node ID
 * @param targetId Target node ID
 * @param result Output structure
 * @return true if target is reachable
 */
bool dijkstraToTarget(Graph* graph, int startId, int targetId, VisitResult* result);

/* ============================================================================
 * ALGORITHM UTILITIES
 * ============================================================================ */

/**
 * @brief Runs algorithm using function pointer strategy
 * @param graph Pointer to graph
 * @param startId Starting node ID
 * @param strategy Function pointer to algorithm
 * @param result Output structure
 * 
 * Example usage:
 *   runAlgorithm(graph, 0, bfs, result);
 *   runAlgorithm(graph, 0, dfs, result);
 *   runAlgorithm(graph, 0, (SearchStrategy)dijkstra, result);
 */
void runAlgorithm(Graph* graph, int startId, SearchStrategy strategy, VisitResult* result);

/**
 * @brief Gets algorithm name as string
 * @param strategy Function pointer
 * @return Algorithm name
 */
const char* getAlgorithmName(SearchStrategy strategy);

/**
 * @brief Prints shortest path from result
 * @param result Pointer to result with predecessor info
 * @param startId Starting node
 * @param targetId Target node
 */
void printShortestPath(VisitResult* result, int startId, int targetId);

/* ============================================================================
 * ADVANCED ALGORITHMS (Bonus)
 * ============================================================================ */

/**
 * @brief Checks if graph is connected using BFS
 * @param graph Pointer to graph
 * @return true if all nodes are reachable from node 0
 */
bool isGraphConnected(Graph* graph);

/**
 * @brief Finds connected components using DFS
 * @param graph Pointer to graph
 * @param components Output array of component IDs per node
 * @return Number of connected components
 */
int findConnectedComponents(Graph* graph, int* components);

/**
 * @brief Detects if graph has a cycle
 * @param graph Pointer to graph
 * @return true if cycle exists
 */
bool hasCycle(Graph* graph);

/**
 * @brief Topological sort using DFS (for DAGs)
 * @param graph Pointer to graph
 * @param sorted Output array for sorted node IDs
 * @return Number of nodes in sorted order, or -1 if cycle detected
 */
int topologicalSort(Graph* graph, int* sorted);

#endif /* ALGORITHMS_H */
