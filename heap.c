/**
 * @file heap.c
 * @brief Min-Heap Implementation for Dijkstra's Algorithm
 * 
 * A binary min-heap priority queue with:
 * - Dynamic array-based storage
 * - Position tracking for O(1) lookups
 * - Decrease-key operation in O(log n)
 * - Automatic resizing
 */

#include "heap.h"

/* ============================================================================
 * HEAP LIFECYCLE
 * ============================================================================ */

/**
 * @brief Creates a new min-heap
 * 
 * Allocates:
 * - Dynamic array for heap nodes
 * - Position array for O(1) node lookup
 */
MinHeap* createMinHeap(int capacity, int maxNodeId) {
    MinHeap* heap = (MinHeap*)malloc(sizeof(MinHeap));
    if (!heap) {
        fprintf(stderr, "Error: Failed to allocate heap\n");
        return NULL;
    }
    
    heap->nodes = (HeapNode*)malloc(capacity * sizeof(HeapNode));
    heap->position = (int*)malloc(maxNodeId * sizeof(int));
    
    if (!heap->nodes || !heap->position) {
        fprintf(stderr, "Error: Failed to allocate heap arrays\n");
        free(heap->nodes);
        free(heap->position);
        free(heap);
        return NULL;
    }
    
    // Initialize position array to -1 (not in heap)
    for (int i = 0; i < maxNodeId; i++) {
        heap->position[i] = -1;
    }
    
    heap->size = 0;
    heap->capacity = capacity;
    heap->maxNodeId = maxNodeId;
    
    return heap;
}

/**
 * @brief Destroys heap and frees all memory
 */
void destroyMinHeap(MinHeap** heap) {
    if (!heap || !*heap) return;
    
    if ((*heap)->nodes) free((*heap)->nodes);
    if ((*heap)->position) free((*heap)->position);
    
    free(*heap);
    *heap = NULL;
}

/**
 * @brief Clears all elements from heap
 */
void clearMinHeap(MinHeap* heap) {
    if (!heap) return;
    
    // Reset position array
    for (int i = 0; i < heap->maxNodeId; i++) {
        heap->position[i] = -1;
    }
    
    heap->size = 0;
}

/* ============================================================================
 * INTERNAL HEAP OPERATIONS
 * ============================================================================ */

/**
 * @brief Swaps two heap nodes and updates position tracking
 * 
 * This is crucial for maintaining the position array which
 * allows O(1) lookup of any node's position in the heap.
 */
void swapHeapNodes(MinHeap* heap, int i, int j) {
    if (!heap || i < 0 || j < 0 || i >= heap->size || j >= heap->size) return;
    
    // Update position tracking
    heap->position[heap->nodes[i].nodeId] = j;
    heap->position[heap->nodes[j].nodeId] = i;
    
    // Swap the nodes
    HeapNode temp = heap->nodes[i];
    heap->nodes[i] = heap->nodes[j];
    heap->nodes[j] = temp;
}

/**
 * @brief Maintains heap property by moving element down
 * 
 * Compares node with its children and swaps with smallest.
 * Recursively continues until heap property is restored.
 */
void minHeapify(MinHeap* heap, int idx) {
    if (!heap || idx < 0 || idx >= heap->size) return;
    
    int smallest = idx;
    int left = leftChild(idx);
    int right = rightChild(idx);
    
    // Find smallest among parent, left child, and right child
    if (left < heap->size && 
        heap->nodes[left].distance < heap->nodes[smallest].distance) {
        smallest = left;
    }
    
    if (right < heap->size && 
        heap->nodes[right].distance < heap->nodes[smallest].distance) {
        smallest = right;
    }
    
    // If smallest is not the parent, swap and continue heapifying
    if (smallest != idx) {
        swapHeapNodes(heap, idx, smallest);
        minHeapify(heap, smallest);
    }
}

/**
 * @brief Resizes heap array when full
 */
static bool resizeHeap(MinHeap* heap) {
    if (!heap) return false;
    
    int newCapacity = heap->capacity * 2;
    HeapNode* newNodes = (HeapNode*)realloc(heap->nodes, newCapacity * sizeof(HeapNode));
    
    if (!newNodes) {
        fprintf(stderr, "Error: Failed to resize heap\n");
        return false;
    }
    
    heap->nodes = newNodes;
    heap->capacity = newCapacity;
    
    return true;
}

/* ============================================================================
 * PUBLIC HEAP OPERATIONS
 * ============================================================================ */

/**
 * @brief Inserts a node into heap
 * 
 * Algorithm:
 * 1. Add node at end of array
 * 2. Bubble up until heap property restored
 */
bool heapInsert(MinHeap* heap, int nodeId, int distance) {
    if (!heap) return false;
    
    // Check if node already in heap
    if (heapContains(heap, nodeId)) {
        // Update distance if new one is smaller
        if (distance < heapGetDistance(heap, nodeId)) {
            heapDecreaseKey(heap, nodeId, distance);
            return true;
        }
        return false;
    }
    
    // Resize if necessary
    if (heap->size >= heap->capacity) {
        if (!resizeHeap(heap)) return false;
    }
    
    // Insert at end
    int i = heap->size;
    heap->nodes[i].nodeId = nodeId;
    heap->nodes[i].distance = distance;
    heap->position[nodeId] = i;
    heap->size++;
    
    // Bubble up to maintain heap property
    while (i > 0 && 
           heap->nodes[parent(i)].distance > heap->nodes[i].distance) {
        swapHeapNodes(heap, i, parent(i));
        i = parent(i);
    }
    
    return true;
}

/**
 * @brief Extracts minimum element from heap
 * 
 * Algorithm:
 * 1. Remove root (minimum)
 * 2. Move last element to root
 * 3. Heapify down to restore property
 */
bool heapExtractMin(MinHeap* heap, int* nodeId, int* distance) {
    if (!heap || isHeapEmpty(heap) || !nodeId || !distance) return false;
    
    // Get root (minimum)
    *nodeId = heap->nodes[0].nodeId;
    *distance = heap->nodes[0].distance;
    
    // Mark as not in heap
    heap->position[*nodeId] = -1;
    
    // Move last element to root
    heap->size--;
    if (heap->size > 0) {
        heap->nodes[0] = heap->nodes[heap->size];
        heap->position[heap->nodes[0].nodeId] = 0;
        
        // Heapify down
        minHeapify(heap, 0);
    }
    
    return true;
}

/**
 * @brief Peeks at minimum without removing
 */
bool heapPeek(MinHeap* heap, int* nodeId, int* distance) {
    if (!heap || isHeapEmpty(heap) || !nodeId || !distance) return false;
    
    *nodeId = heap->nodes[0].nodeId;
    *distance = heap->nodes[0].distance;
    
    return true;
}

/**
 * @brief Decreases key (distance) of a node
 * 
 * This is the KEY OPERATION for Dijkstra's algorithm.
 * When we find a shorter path to a node, we update its distance.
 * 
 * Time Complexity: O(log n)
 */
void heapDecreaseKey(MinHeap* heap, int nodeId, int newDistance) {
    if (!heap) return;
    
    // Get current position
    int i = heap->position[nodeId];
    if (i == -1) return;  // Not in heap
    
    // Only decrease allowed (for min-heap)
    if (newDistance >= heap->nodes[i].distance) return;
    
    // Update distance
    heap->nodes[i].distance = newDistance;
    
    // Bubble up to maintain heap property
    while (i > 0 && 
           heap->nodes[parent(i)].distance > heap->nodes[i].distance) {
        swapHeapNodes(heap, i, parent(i));
        i = parent(i);
    }
}

/**
 * @brief Checks if node is in heap
 */
bool heapContains(MinHeap* heap, int nodeId) {
    if (!heap || nodeId < 0 || nodeId >= heap->maxNodeId) return false;
    return heap->position[nodeId] != -1;
}

/**
 * @brief Gets current distance of node
 */
int heapGetDistance(MinHeap* heap, int nodeId) {
    if (!heapContains(heap, nodeId)) return INT_MAX;
    return heap->nodes[heap->position[nodeId]].distance;
}

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

bool isHeapEmpty(MinHeap* heap) {
    return !heap || heap->size == 0;
}

int heapSize(MinHeap* heap) {
    return heap ? heap->size : 0;
}

/**
 * @brief Prints heap structure for debugging
 */
void printHeap(MinHeap* heap) {
    if (!heap) {
        printf("Heap is NULL\n");
        return;
    }
    
    printf("\n╔══════════════════════════════════════╗\n");
    printf("║       MIN HEAP STRUCTURE             ║\n");
    printf("╠══════════════════════════════════════╣\n");
    printf("║ Size: %d / Capacity: %d\n", heap->size, heap->capacity);
    printf("║─── Heap Array ───\n");
    
    for (int i = 0; i < heap->size; i++) {
        printf("║ [%d] Node %d (dist=%d)\n", 
               i, heap->nodes[i].nodeId, heap->nodes[i].distance);
    }
    
    printf("╚══════════════════════════════════════╝\n");
}

/**
 * @brief Validates heap property
 */
bool validateHeap(MinHeap* heap) {
    if (!heap) return false;
    
    // Check heap property: parent <= children
    for (int i = 0; i < heap->size; i++) {
        int left = leftChild(i);
        int right = rightChild(i);
        
        if (left < heap->size && 
            heap->nodes[i].distance > heap->nodes[left].distance) {
            fprintf(stderr, "Heap validation failed at node %d (left child)\n", i);
            return false;
        }
        
        if (right < heap->size && 
            heap->nodes[i].distance > heap->nodes[right].distance) {
            fprintf(stderr, "Heap validation failed at node %d (right child)\n", i);
            return false;
        }
        
        // Check position consistency
        int nodeId = heap->nodes[i].nodeId;
        if (heap->position[nodeId] != i) {
            fprintf(stderr, "Position array inconsistent for node %d\n", nodeId);
            return false;
        }
    }
    
    return true;
}
