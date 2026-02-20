/**
 * @file graph.c
 * @brief Graph Implementation - Pointer Cuisine at its Finest
 * 
 * This implementation features:
 * - Double pointer techniques
 * - Self-referential struct navigation
 * - Dynamic memory allocation with malloc/free
 * - Linked list operations for adjacency lists
 * - Memory ownership responsibility
 */

#include "graph.h"
#include <string.h>

/* ============================================================================
 * GRAPH LIFECYCLE
 * ============================================================================ */

/**
 * @brief Creates a new empty graph
 * 
 * Allocates memory for the graph structure and initializes
 * all fields to default values.
 */
Graph* createGraph(void) {
    Graph* graph = (Graph*)malloc(sizeof(Graph));
    if (!graph) {
        fprintf(stderr, "Error: Failed to allocate memory for graph\n");
        return NULL;
    }
    
    // Initialize graph fields
    graph->head = NULL;
    graph->nodeCount = 0;
    graph->edgeCount = 0;
    
    return graph;
}

/**
 * @brief Destroys graph and frees all memory
 * 
 * Uses double pointer to:
 * 1. Free all allocated memory
 * 2. Set the caller's pointer to NULL (prevents dangling pointer)
 * 
 * This is proper memory hygiene in C.
 */
void destroyGraph(Graph** graph) {
    if (!graph || !*graph) return;
    
    // Clear all nodes and edges first
    clearGraph(*graph);
    
    // Free the graph structure itself
    free(*graph);
    
    // Set caller's pointer to NULL - this is why we use double pointer
    *graph = NULL;
}

/**
 * @brief Clears all nodes and edges from graph
 * 
 * Traverses the entire graph structure and frees:
 * - All edges in each adjacency list
 * - All nodes
 * 
 * Resets counts but keeps graph structure valid.
 */
void clearGraph(Graph* graph) {
    if (!graph) return;
    
    Node* current = graph->head;
    
    // Traverse all nodes
    while (current) {
        Node* nodeToFree = current;
        current = current->next;
        
        // Free all edges in this node's adjacency list
        Edge* edge = nodeToFree->edges;
        while (edge) {
            Edge* edgeToFree = edge;
            edge = edge->next;
            free(edgeToFree);
        }
        
        // Free the node itself
        free(nodeToFree);
    }
    
    // Reset graph state
    graph->head = NULL;
    graph->nodeCount = 0;
    graph->edgeCount = 0;
}

/* ============================================================================
 * NODE OPERATIONS (Double Pointer Magic)
 * ============================================================================ */

/**
 * @brief Creates node using double pointer technique
 * 
 * This is the POWER MOVE in C pointer programming.
 * 
 * By accepting Node**, we can modify the caller's head pointer
 * directly. This allows us to:
 * - Insert at the beginning without special cases
 * - Modify the actual pointer value in the caller
 * 
 * Example:
 *   Node* head = NULL;
 *   createNode(&head, 5);  // head now points to new node
 */
void createNode(Node** headRef, int id) {
    if (!headRef) return;
    
    // Allocate memory for new node
    Node* newNode = (Node*)malloc(sizeof(Node));
    if (!newNode) {
        fprintf(stderr, "Error: Failed to allocate memory for node\n");
        return;
    }
    
    // Initialize node fields
    newNode->id = id;
    newNode->edges = NULL;  // No edges yet
    
    // Link to existing list - insert at head for O(1)
    newNode->next = *headRef;
    
    // CRITICAL: Modify the caller's pointer
    // This is why we passed Node** (pointer to pointer)
    *headRef = newNode;
}

/**
 * @brief Adds a new node to the graph
 * 
 * Checks for duplicates, then uses createNode to insert.
 */
Node* addNode(Graph* graph, int id) {
    if (!graph) return NULL;
    
    // Check if node already exists
    if (findNode(graph, id)) {
        fprintf(stderr, "Error: Node %d already exists\n", id);
        return NULL;
    }
    
    // Use double pointer technique to create and insert node
    createNode(&graph->head, id);
    graph->nodeCount++;
    
    return graph->head;
}

/**
 * @brief Finds a node by ID using pointer traversal
 * 
 * Classic linked list traversal pattern:
 *   while (ptr && ptr->id != target)
 *       ptr = ptr->next;
 */
Node* findNode(Graph* graph, int id) {
    if (!graph) return NULL;
    
    Node* current = graph->head;
    
    // Traverse until we find the node or reach end
    while (current && current->id != id) {
        current = current->next;
    }
    
    return current;  // Returns NULL if not found
}

/**
 * @brief Removes a node and all its incident edges
 * 
 * This is complex because we need to:
 * 1. Remove the node from the node list
 * 2. Remove all edges pointing to this node (from other nodes)
 * 3. Free all edges in this node's adjacency list
 * 4. Free the node itself
 */
bool removeNode(Graph* graph, int id) {
    if (!graph || !graph->head) return false;
    
    // First, remove all edges pointing to this node from other nodes
    Node* current = graph->head;
    while (current) {
        if (current->id != id) {
            // Remove edge to this node if it exists
            removeEdge(graph, current->id, id);
        }
        current = current->next;
    }
    
    // Now remove the node itself from the list
    Node** currentRef = &graph->head;
    
    while (*currentRef) {
        if ((*currentRef)->id == id) {
            Node* nodeToRemove = *currentRef;
            
            // Free all edges in adjacency list
            Edge* edge = nodeToRemove->edges;
            while (edge) {
                Edge* edgeToFree = edge;
                edge = edge->next;
                free(edgeToFree);
                graph->edgeCount--;
            }
            
            // Unlink from list
            *currentRef = nodeToRemove->next;
            
            // Free the node
            free(nodeToRemove);
            graph->nodeCount--;
            
            return true;
        }
        currentRef = &(*currentRef)->next;
    }
    
    return false;  // Node not found
}

/**
 * @brief Checks if node exists
 */
bool hasNode(Graph* graph, int id) {
    return findNode(graph, id) != NULL;
}

/* ============================================================================
 * EDGE OPERATIONS (Pointer Linking)
 * ============================================================================ */

/**
 * @brief Adds a weighted edge between two nodes
 * 
 * This is where pointer cuisine gets spicy:
 * 
 * 1. Find source and destination nodes (pointer traversal)
 * 2. Allocate new edge
 * 3. Link into adjacency list using pointer manipulation
 * 
 * Edge is inserted at HEAD of adjacency list for O(1) time.
 */
bool addEdge(Graph* graph, int srcId, int destId, int weight) {
    if (!graph) return false;
    
    // Find source node - pointer traversal
    Node* src = graph->head;
    while (src && src->id != srcId) {
        src = src->next;
    }
    
    if (!src) {
        fprintf(stderr, "Error: Source node %d not found\n", srcId);
        return false;
    }
    
    // Find destination node - pointer traversal
    Node* dest = graph->head;
    while (dest && dest->id != destId) {
        dest = dest->next;
    }
    
    if (!dest) {
        fprintf(stderr, "Error: Destination node %d not found\n", destId);
        return false;
    }
    
    // Check if edge already exists
    if (hasEdge(graph, srcId, destId)) {
        fprintf(stderr, "Warning: Edge %d->%d already exists, updating weight\n", srcId, destId);
        // Update weight instead of creating duplicate
        Edge* edge = src->edges;
        while (edge) {
            if (edge->destination->id == destId) {
                edge->weight = weight;
                return true;
            }
            edge = edge->next;
        }
        return false;
    }
    
    // Allocate new edge - dynamic memory
    Edge* newEdge = (Edge*)malloc(sizeof(Edge));
    if (!newEdge) {
        fprintf(stderr, "Error: Failed to allocate memory for edge\n");
        return false;
    }
    
    // Initialize edge
    newEdge->weight = weight;
    newEdge->destination = dest;  // Store pointer to destination node
    
    // Insert at HEAD of adjacency list - O(1) insertion
    // This is pointer linking at its finest
    newEdge->next = src->edges;   // New edge points to current first edge
    src->edges = newEdge;         // Head now points to new edge
    
    graph->edgeCount++;
    return true;
}

/**
 * @brief Adds undirected edge (two directed edges)
 */
bool addUndirectedEdge(Graph* graph, int node1, int node2, int weight) {
    bool e1 = addEdge(graph, node1, node2, weight);
    bool e2 = addEdge(graph, node2, node1, weight);
    return e1 && e2;
}

/**
 * @brief Removes an edge from adjacency list
 * 
 * Uses double pointer technique to remove from linked list
 * without special cases for head removal.
 */
bool removeEdge(Graph* graph, int srcId, int destId) {
    if (!graph) return false;
    
    Node* src = findNode(graph, srcId);
    if (!src) return false;
    
    // Use double pointer for elegant removal
    Edge** currentRef = &src->edges;
    
    while (*currentRef) {
        if ((*currentRef)->destination->id == destId) {
            Edge* edgeToRemove = *currentRef;
            
            // Unlink from list
            *currentRef = edgeToRemove->next;
            
            // Free the edge
            free(edgeToRemove);
            graph->edgeCount--;
            
            return true;
        }
        currentRef = &(*currentRef)->next;
    }
    
    return false;  // Edge not found
}

/**
 * @brief Checks if edge exists
 */
bool hasEdge(Graph* graph, int srcId, int destId) {
    Node* src = findNode(graph, srcId);
    if (!src) return false;
    
    Edge* edge = src->edges;
    while (edge) {
        if (edge->destination->id == destId) {
            return true;
        }
        edge = edge->next;
    }
    
    return false;
}

/**
 * @brief Gets edge weight
 */
int getEdgeWeight(Graph* graph, int srcId, int destId) {
    Node* src = findNode(graph, srcId);
    if (!src) return -1;
    
    Edge* edge = src->edges;
    while (edge) {
        if (edge->destination->id == destId) {
            return edge->weight;
        }
        edge = edge->next;
    }
    
    return -1;  // Edge not found
}

/* ============================================================================
 * GRAPH UTILITY FUNCTIONS
 * ============================================================================ */

int getNodeCount(Graph* graph) {
    return graph ? graph->nodeCount : 0;
}

int getEdgeCount(Graph* graph) {
    return graph ? graph->edgeCount : 0;
}

/**
 * @brief Traverses all nodes with callback
 * 
 * Demonstrates function pointer usage for callbacks.
 */
void traverseNodes(Graph* graph, TraverseCallback callback, void* userData) {
    if (!graph || !callback) return;
    
    Node* current = graph->head;
    while (current) {
        callback(current, userData);
        current = current->next;
    }
}

/**
 * @brief Prints graph structure
 */
void printGraph(Graph* graph) {
    if (!graph) {
        printf("Graph is NULL\n");
        return;
    }
    
    printf("\n╔══════════════════════════════════════╗\n");
    printf("║       GRAPH STRUCTURE                ║\n");
    printf("╠══════════════════════════════════════╣\n");
    printf("║ Nodes: %-3d  Edges: %-3d              ║\n", 
           graph->nodeCount, graph->edgeCount);
    printf("╚══════════════════════════════════════╝\n\n");
    
    printAdjacencyList(graph);
}

/**
 * @brief Prints adjacency list representation
 */
void printAdjacencyList(Graph* graph) {
    if (!graph) return;
    
    printf("Adjacency List:\n");
    printf("───────────────\n");
    
    Node* node = graph->head;
    while (node) {
        printf("Node %d: ", node->id);
        
        Edge* edge = node->edges;
        if (!edge) {
            printf("(no outgoing edges)");
        }
        
        while (edge) {
            printf("→ [%d|%d] ", edge->destination->id, edge->weight);
            edge = edge->next;
        }
        
        printf("\n");
        node = node->next;
    }
    
    printf("\n");
}

/**
 * @brief Creates a deep copy of the graph
 * 
 * This is advanced pointer work:
 * 1. Create new graph
 * 2. Copy all nodes
 * 3. Copy all edges (with proper destination pointers)
 */
Graph* cloneGraph(Graph* source) {
    if (!source) return NULL;
    
    Graph* clone = createGraph();
    if (!clone) return NULL;
    
    // First pass: create all nodes
    Node* srcNode = source->head;
    while (srcNode) {
        addNode(clone, srcNode->id);
        srcNode = srcNode->next;
    }
    
    // Second pass: create all edges
    srcNode = source->head;
    while (srcNode) {
        Edge* srcEdge = srcNode->edges;
        while (srcEdge) {
            addEdge(clone, srcNode->id, srcEdge->destination->id, srcEdge->weight);
            srcEdge = srcEdge->next;
        }
        srcNode = srcNode->next;
    }
    
    return clone;
}

/* ============================================================================
 * VISIT RESULT FUNCTIONS
 * ============================================================================ */

VisitResult* createVisitResult(int capacity) {
    VisitResult* result = (VisitResult*)malloc(sizeof(VisitResult));
    if (!result) return NULL;
    
    result->visited = (int*)calloc(capacity, sizeof(int));
    result->distances = (int*)malloc(capacity * sizeof(int));
    result->predecessors = (int*)malloc(capacity * sizeof(int));
    
    if (!result->visited || !result->distances || !result->predecessors) {
        destroyVisitResult(&result);
        return NULL;
    }
    
    // Initialize arrays
    for (int i = 0; i < capacity; i++) {
        result->distances[i] = INFINITY_DISTANCE;
        result->predecessors[i] = -1;
    }
    
    result->count = 0;
    result->capacity = capacity;
    
    return result;
}

void destroyVisitResult(VisitResult** result) {
    if (!result || !*result) return;
    
    if ((*result)->visited) free((*result)->visited);
    if ((*result)->distances) free((*result)->distances);
    if ((*result)->predecessors) free((*result)->predecessors);
    
    free(*result);
    *result = NULL;
}

void addVisited(VisitResult* result, int nodeId) {
    if (!result || result->count >= result->capacity) return;
    result->visited[result->count++] = nodeId;
}

void printVisitResult(VisitResult* result) {
    if (!result) {
        printf("Result is NULL\n");
        return;
    }
    
    printf("\n╔══════════════════════════════════════╗\n");
    printf("║       TRAVERSAL RESULT               ║\n");
    printf("╠══════════════════════════════════════╣\n");
    printf("║ Visited Nodes (%d): ", result->count);
    
    for (int i = 0; i < result->count; i++) {
        printf("%d ", result->visited[i]);
    }
    printf("\n");
    
    // Print distances if they were computed
    bool hasDistances = false;
    for (int i = 0; i < result->capacity; i++) {
        if (result->distances[i] != INFINITY_DISTANCE) {
            hasDistances = true;
            break;
        }
    }
    
    if (hasDistances) {
        printf("║ Distances:\n");
        for (int i = 0; i < result->capacity; i++) {
            if (result->distances[i] != INFINITY_DISTANCE) {
                printf("║   Node %d: %d\n", i, result->distances[i]);
            }
        }
    }
    
    printf("╚══════════════════════════════════════╝\n");
}

bool reconstructPath(VisitResult* result, int startId, int targetId, 
                     int* path, int* pathLength) {
    if (!result || !path || !pathLength) return false;
    
    if (result->distances[targetId] == INFINITY_DISTANCE) {
        return false;  // Target not reachable
    }
    
    // Build path backwards from target to start
    int current = targetId;
    int idx = 0;
    int tempPath[result->capacity];
    
    while (current != -1 && current != startId) {
        tempPath[idx++] = current;
        current = result->predecessors[current];
    }
    
    if (current == -1) {
        return false;  // No path exists
    }
    
    tempPath[idx++] = startId;
    
    // Reverse to get start -> target
    for (int i = 0; i < idx; i++) {
        path[i] = tempPath[idx - 1 - i];
    }
    
    *pathLength = idx;
    return true;
}

/* ============================================================================
 * MEMORY MANAGEMENT UTILITIES
 * ============================================================================ */

size_t getGraphMemoryUsage(Graph* graph) {
    if (!graph) return 0;
    
    size_t total = sizeof(Graph);
    
    Node* node = graph->head;
    while (node) {
        total += sizeof(Node);
        
        Edge* edge = node->edges;
        while (edge) {
            total += sizeof(Edge);
            edge = edge->next;
        }
        
        node = node->next;
    }
    
    return total;
}

bool validateGraph(Graph* graph) {
    if (!graph) return false;
    
    // Count nodes manually and compare
    int manualCount = 0;
    Node* node = graph->head;
    
    while (node) {
        manualCount++;
        
        // Validate edges
        Edge* edge = node->edges;
        while (edge) {
            if (!edge->destination) {
                fprintf(stderr, "Validation Error: Edge with NULL destination\n");
                return false;
            }
            edge = edge->next;
        }
        
        node = node->next;
    }
    
    if (manualCount != graph->nodeCount) {
        fprintf(stderr, "Validation Error: Node count mismatch\n");
        return false;
    }
    
    return true;
}
