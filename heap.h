/**
 * @file heap.h
 * @brief Min-Heap Implementation with Pointers for Dijkstra's Algorithm
 * 
 * A binary min-heap implemented using dynamic arrays and pointers.
 * Used as a priority queue for Dijkstra's shortest path algorithm.
 */

#ifndef HEAP_H
#define HEAP_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <limits.h>

/* ============================================================================
 * HEAP DATA STRUCTURES
 * ============================================================================ */

/**
 * @struct HeapNode
 * @brief Represents a node in the priority queue
 * 
 * Stores:
 * - nodeId: The graph node identifier
 * - distance: Current shortest distance (priority key)
 */
typedef struct HeapNode {
    int nodeId;         // Graph node identifier
    int distance;       // Priority (distance for Dijkstra)
} HeapNode;

/**
 * @struct MinHeap
 * @brief Binary min-heap structure
 * 
 * Implements priority queue with:
 * - Dynamic array of heap nodes
 * - Position tracking for O(1) lookup
 * - Automatic resizing
 */
typedef struct MinHeap {
    HeapNode* nodes;        // Dynamic array of heap nodes
    int* position;          // Array to track node positions in heap
    int size;               // Current number of elements
    int capacity;           // Maximum capacity
    int maxNodeId;          // Maximum node ID for position array sizing
} MinHeap;

/* ============================================================================
 * HEAP LIFECYCLE
 * ============================================================================ */

/**
 * @brief Creates a new min-heap
 * @param capacity Initial capacity
 * @param maxNodeId Maximum expected node ID
 * @return Pointer to new MinHeap, or NULL on failure
 */
MinHeap* createMinHeap(int capacity, int maxNodeId);

/**
 * @brief Destroys heap and frees memory
 * @param heap Double pointer to heap
 */
void destroyMinHeap(MinHeap** heap);

/**
 * @brief Clears all elements from heap
 * @param heap Pointer to heap
 */
void clearMinHeap(MinHeap* heap);

/* ============================================================================
 * HEAP OPERATIONS
 * ============================================================================ */

/**
 * @brief Inserts a node into heap
 * @param heap Pointer to heap
 * @param nodeId Node identifier
 * @param distance Priority distance
 * @return true if insertion succeeded
 */
bool heapInsert(MinHeap* heap, int nodeId, int distance);

/**
 * @brief Extracts minimum element from heap
 * @param heap Pointer to heap
 * @param nodeId Output parameter for node ID
 * @param distance Output parameter for distance
 * @return true if extraction succeeded (heap not empty)
 */
bool heapExtractMin(MinHeap* heap, int* nodeId, int* distance);

/**
 * @brief Gets minimum element without removing
 * @param heap Pointer to heap
 * @param nodeId Output parameter for node ID
 * @param distance Output parameter for distance
 * @return true if heap not empty
 */
bool heapPeek(MinHeap* heap, int* nodeId, int* distance);

/**
 * @brief Decreases key (distance) of a node
 * @param heap Pointer to heap
 * @param nodeId Node to update
 * @param newDistance New distance value
 * 
 * This is crucial for Dijkstra's algorithm when a shorter
 * path to a node is discovered.
 */
void heapDecreaseKey(MinHeap* heap, int nodeId, int newDistance);

/**
 * @brief Checks if node exists in heap
 * @param heap Pointer to heap
 * @param nodeId Node to check
 * @return true if node is in heap
 */
bool heapContains(MinHeap* heap, int nodeId);

/**
 * @brief Gets current distance of node in heap
 * @param heap Pointer to heap
 * @param nodeId Node to query
 * @return Current distance, or INT_MAX if not in heap
 */
int heapGetDistance(MinHeap* heap, int nodeId);

/* ============================================================================
 * HEAP UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief Checks if heap is empty
 * @param heap Pointer to heap
 * @return true if heap is empty
 */
bool isHeapEmpty(MinHeap* heap);

/**
 * @brief Gets number of elements in heap
 * @param heap Pointer to heap
 * @return Current size
 */
int heapSize(MinHeap* heap);

/**
 * @brief Prints heap structure for debugging
 * @param heap Pointer to heap
 */
void printHeap(MinHeap* heap);

/**
 * @brief Validates heap property
 * @param heap Pointer to heap
 * @return true if heap is valid
 */
bool validateHeap(MinHeap* heap);

/* ============================================================================
 * INTERNAL HEAP FUNCTIONS (exposed for testing)
 * ============================================================================ */

/**
 * @brief Maintains heap property by moving element down
 * @param heap Pointer to heap
 * @param idx Index to heapify from
 */
void minHeapify(MinHeap* heap, int idx);

/**
 * @brief Swaps two heap nodes
 * @param heap Pointer to heap
 * @param i First index
 * @param j Second index
 */
void swapHeapNodes(MinHeap* heap, int i, int j);

/**
 * @brief Gets parent index
 * @param i Child index
 * @return Parent index
 */
static inline int parent(int i) { return (i - 1) / 2; }

/**
 * @brief Gets left child index
 * @param i Parent index
 * @return Left child index
 */
static inline int leftChild(int i) { return 2 * i + 1; }

/**
 * @brief Gets right child index
 * @param i Parent index
 * @return Right child index
 */
static inline int rightChild(int i) { return 2 * i + 2; }

#endif /* HEAP_H */
