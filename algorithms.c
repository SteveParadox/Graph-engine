/**
 * @file algorithms.c
 * @brief Graph Search Algorithms Implementation
 * 
 * Implements:
 * - BFS: Queue-based level-order traversal
 * - DFS: Recursive and iterative depth-first traversal
 * - Dijkstra: Shortest path with min-heap priority queue
 */

#include "algorithms.h"
#include <string.h>

/* ============================================================================
 * QUEUE STRUCTURE FOR BFS
 * ============================================================================ */

typedef struct QueueNode {
    int data;
    struct QueueNode* next;
} QueueNode;

typedef struct Queue {
    QueueNode* front;
    QueueNode* rear;
    int size;
} Queue;

static Queue* createQueue(void) {
    Queue* q = (Queue*)malloc(sizeof(Queue));
    q->front = q->rear = NULL;
    q->size = 0;
    return q;
}

static void enqueue(Queue* q, int data) {
    QueueNode* newNode = (QueueNode*)malloc(sizeof(QueueNode));
    newNode->data = data;
    newNode->next = NULL;
    
    if (!q->rear) {
        q->front = q->rear = newNode;
    } else {
        q->rear->next = newNode;
        q->rear = newNode;
    }
    q->size++;
}

static int dequeue(Queue* q) {
    if (!q->front) return -1;
    
    QueueNode* temp = q->front;
    int data = temp->data;
    q->front = q->front->next;
    
    if (!q->front) q->rear = NULL;
    
    free(temp);
    q->size--;
    return data;
}

static bool isQueueEmpty(Queue* q) {
    return !q || q->size == 0;
}

static void destroyQueue(Queue* q) {
    if (!q) return;
    while (!isQueueEmpty(q)) dequeue(q);
    free(q);
}

/* ============================================================================
 * STACK STRUCTURE FOR ITERATIVE DFS
 * ============================================================================ */

typedef struct StackNode {
    int data;
    struct StackNode* next;
} StackNode;

typedef struct Stack {
    StackNode* top;
    int size;
} Stack;

static Stack* createStack(void) {
    Stack* s = (Stack*)malloc(sizeof(Stack));
    s->top = NULL;
    s->size = 0;
    return s;
}

static void push(Stack* s, int data) {
    StackNode* newNode = (StackNode*)malloc(sizeof(StackNode));
    newNode->data = data;
    newNode->next = s->top;
    s->top = newNode;
    s->size++;
}

static int pop(Stack* s) {
    if (!s || !s->top) return -1;
    
    StackNode* temp = s->top;
    int data = temp->data;
    s->top = s->top->next;
    free(temp);
    s->size--;
    return data;
}

static bool isStackEmpty(Stack* s) {
    return !s || s->size == 0;
}

static void destroyStack(Stack* s) {
    if (!s) return;
    while (!isStackEmpty(s)) pop(s);
    free(s);
}

/* ============================================================================
 * BREADTH-FIRST SEARCH (BFS)
 * ============================================================================ */

/**
 * @brief BFS using queue - level order traversal
 * 
 * Algorithm:
 * 1. Mark start as visited, enqueue it
 * 2. While queue not empty:
 *    - Dequeue a node
 *    - Process it
 *    - Enqueue all unvisited neighbors
 * 
 * Time: O(V + E)
 * Space: O(V)
 */
void bfs(Graph* graph, int startId, VisitResult* result) {
    if (!graph || !result) return;
    
    Node* startNode = findNode(graph, startId);
    if (!startNode) {
        fprintf(stderr, "Error: Start node %d not found\n", startId);
        return;
    }
    
    // Initialize visited array
    bool* visited = (bool*)calloc(result->capacity, sizeof(bool));
    if (!visited) return;
    
    Queue* queue = createQueue();
    
    // Start BFS
    visited[startId] = true;
    enqueue(queue, startId);
    addVisited(result, startId);
    result->distances[startId] = 0;
    
    while (!isQueueEmpty(queue)) {
        int currentId = dequeue(queue);
        Node* current = findNode(graph, currentId);
        
        // Visit all neighbors
        Edge* edge = current->edges;
        while (edge) {
            int neighborId = edge->destination->id;
            
            if (!visited[neighborId]) {
                visited[neighborId] = true;
                result->distances[neighborId] = result->distances[currentId] + 1;
                result->predecessors[neighborId] = currentId;
                enqueue(queue, neighborId);
                addVisited(result, neighborId);
            }
            
            edge = edge->next;
        }
    }
    
    destroyQueue(queue);
    free(visited);
}

/**
 * @brief BFS that stops when target is found
 */
bool bfsFind(Graph* graph, int startId, int targetId, VisitResult* result) {
    if (!graph || !result) return false;
    
    Node* startNode = findNode(graph, startId);
    Node* targetNode = findNode(graph, targetId);
    
    if (!startNode || !targetNode) return false;
    
    bool* visited = (bool*)calloc(result->capacity, sizeof(bool));
    if (!visited) return false;
    
    Queue* queue = createQueue();
    
    visited[startId] = true;
    enqueue(queue, startId);
    addVisited(result, startId);
    result->distances[startId] = 0;
    
    while (!isQueueEmpty(queue)) {
        int currentId = dequeue(queue);
        
        if (currentId == targetId) {
            destroyQueue(queue);
            free(visited);
            return true;
        }
        
        Node* current = findNode(graph, currentId);
        Edge* edge = current->edges;
        
        while (edge) {
            int neighborId = edge->destination->id;
            
            if (!visited[neighborId]) {
                visited[neighborId] = true;
                result->distances[neighborId] = result->distances[currentId] + 1;
                result->predecessors[neighborId] = currentId;
                enqueue(queue, neighborId);
                addVisited(result, neighborId);
            }
            
            edge = edge->next;
        }
    }
    
    destroyQueue(queue);
    free(visited);
    return false;
}

/* ============================================================================
 * DEPTH-FIRST SEARCH (DFS)
 * ============================================================================ */

/**
 * @brief Recursive DFS helper
 */
static void dfsRecursiveHelper(Graph* graph, Node* node, bool* visited, 
                                VisitResult* result) {
    if (!node || visited[node->id]) return;
    
    // Mark and process current node
    visited[node->id] = true;
    addVisited(result, node->id);
    
    // Recursively visit all neighbors
    Edge* edge = node->edges;
    while (edge) {
        Node* neighbor = edge->destination;
        if (!visited[neighbor->id]) {
            result->predecessors[neighbor->id] = node->id;
            dfsRecursiveHelper(graph, neighbor, visited, result);
        }
        edge = edge->next;
    }
}

/**
 * @brief DFS using recursion
 * 
 * Algorithm:
 * 1. Mark current node as visited
 * 2. Process current node
 * 3. Recursively visit all unvisited neighbors
 * 
 * Time: O(V + E)
 * Space: O(V) for recursion stack
 */
void dfs(Graph* graph, int startId, VisitResult* result) {
    if (!graph || !result) return;
    
    Node* startNode = findNode(graph, startId);
    if (!startNode) {
        fprintf(stderr, "Error: Start node %d not found\n", startId);
        return;
    }
    
    bool* visited = (bool*)calloc(result->capacity, sizeof(bool));
    if (!visited) return;
    
    dfsRecursiveHelper(graph, startNode, visited, result);
    
    free(visited);
}

/**
 * @brief Iterative DFS using explicit stack
 * 
 * Same result as recursive DFS but uses explicit stack.
 * Useful for very deep graphs to avoid stack overflow.
 */
void dfsIterative(Graph* graph, int startId, VisitResult* result) {
    if (!graph || !result) return;
    
    Node* startNode = findNode(graph, startId);
    if (!startNode) return;
    
    bool* visited = (bool*)calloc(result->capacity, sizeof(bool));
    if (!visited) return;
    
    Stack* stack = createStack();
    
    push(stack, startId);
    
    while (!isStackEmpty(stack)) {
        int currentId = pop(stack);
        
        if (visited[currentId]) continue;
        
        visited[currentId] = true;
        addVisited(result, currentId);
        
        Node* current = findNode(graph, currentId);
        
        // Push all neighbors (in reverse order for same result as recursive)
        Edge* edge = current->edges;
        // Count edges first to push in reverse
        int edgeCount = 0;
        Edge* temp = edge;
        while (temp) { edgeCount++; temp = temp->next; }
        
        // Store neighbors in array
        int* neighbors = (int*)malloc(edgeCount * sizeof(int));
        int i = 0;
        temp = edge;
        while (temp) {
            neighbors[i++] = temp->destination->id;
            temp = temp->next;
        }
        
        // Push in reverse order
        for (int j = edgeCount - 1; j >= 0; j--) {
            if (!visited[neighbors[j]]) {
                result->predecessors[neighbors[j]] = currentId;
                push(stack, neighbors[j]);
            }
        }
        
        free(neighbors);
    }
    
    destroyStack(stack);
    free(visited);
}

/**
 * @brief DFS that stops when target is found
 */
bool dfsFind(Graph* graph, int startId, int targetId, VisitResult* result) {
    if (!graph || !result) return false;
    
    Node* startNode = findNode(graph, startId);
    if (!startNode) return false;
    
    bool* visited = (bool*)calloc(result->capacity, sizeof(bool));
    if (!visited) return false;
    
    Stack* stack = createStack();
    push(stack, startId);
    
    while (!isStackEmpty(stack)) {
        int currentId = pop(stack);
        
        if (visited[currentId]) continue;
        
        visited[currentId] = true;
        addVisited(result, currentId);
        
        if (currentId == targetId) {
            destroyStack(stack);
            free(visited);
            return true;
        }
        
        Node* current = findNode(graph, currentId);
        Edge* edge = current->edges;
        
        while (edge) {
            int neighborId = edge->destination->id;
            if (!visited[neighborId]) {
                result->predecessors[neighborId] = currentId;
                push(stack, neighborId);
            }
            edge = edge->next;
        }
    }
    
    destroyStack(stack);
    free(visited);
    return false;
}

/* ============================================================================
 * DIJKSTRA'S SHORTEST PATH ALGORITHM
 * ============================================================================ */

/**
 * @brief Dijkstra's algorithm with min-heap
 * 
 * Algorithm:
 * 1. Initialize distances to infinity, start to 0
 * 2. Insert all nodes into priority queue
 * 3. While queue not empty:
 *    - Extract minimum distance node
 *    - For each neighbor: relax edge
 * 
 * Time: O((V + E) log V) with heap
 * Space: O(V)
 */
void dijkstra(Graph* graph, int startId, VisitResult* result) {
    if (!graph || !result) return;
    
    Node* startNode = findNode(graph, startId);
    if (!startNode) {
        fprintf(stderr, "Error: Start node %d not found\n", startId);
        return;
    }
    
    // Find max node ID for heap sizing
    int maxNodeId = 0;
    Node* node = graph->head;
    while (node) {
        if (node->id > maxNodeId) maxNodeId = node->id;
        node = node->next;
    }
    maxNodeId++;  // For 0-indexed arrays
    
    // Create min-heap priority queue
    MinHeap* heap = createMinHeap(graph->nodeCount + 10, maxNodeId + 10);
    if (!heap) return;
    
    // Initialize distances
    for (int i = 0; i < maxNodeId; i++) {
        result->distances[i] = INFINITY_DISTANCE;
    }
    result->distances[startId] = 0;
    
    // Insert start node
    heapInsert(heap, startId, 0);
    
    // Track visited nodes
    bool* visited = (bool*)calloc(maxNodeId, sizeof(bool));
    if (!visited) {
        destroyMinHeap(&heap);
        return;
    }
    
    // Main Dijkstra loop
    while (!isHeapEmpty(heap)) {
        int currentId, currentDist;
        heapExtractMin(heap, &currentId, &currentDist);
        
        if (visited[currentId]) continue;
        visited[currentId] = true;
        addVisited(result, currentId);
        
        Node* current = findNode(graph, currentId);
        if (!current) continue;
        
        // Relax all edges
        Edge* edge = current->edges;
        while (edge) {
            int neighborId = edge->destination->id;
            int weight = edge->weight;
            
            if (!visited[neighborId]) {
                int newDist = currentDist + weight;
                
                if (newDist < result->distances[neighborId]) {
                    result->distances[neighborId] = newDist;
                    result->predecessors[neighborId] = currentId;
                    
                    if (heapContains(heap, neighborId)) {
                        heapDecreaseKey(heap, neighborId, newDist);
                    } else {
                        heapInsert(heap, neighborId, newDist);
                    }
                }
            }
            
            edge = edge->next;
        }
    }
    
    free(visited);
    destroyMinHeap(&heap);
}

/**
 * @brief Finds shortest path between two nodes
 */
bool dijkstraShortestPath(Graph* graph, int startId, int targetId,
                          int* distance, int* path, int* pathLength) {
    if (!graph || !distance || !path || !pathLength) return false;
    
    // Find max node ID
    int maxNodeId = 0;
    Node* node = graph->head;
    while (node) {
        if (node->id > maxNodeId) maxNodeId = node->id;
        node = node->next;
    }
    maxNodeId++;
    
    VisitResult* result = createVisitResult(maxNodeId);
    if (!result) return false;
    
    dijkstra(graph, startId, result);
    
    *distance = result->distances[targetId];
    
    if (*distance == INFINITY_DISTANCE) {
        destroyVisitResult(&result);
        return false;
    }
    
    bool success = reconstructPath(result, startId, targetId, path, pathLength);
    
    destroyVisitResult(&result);
    return success;
}

/**
 * @brief Dijkstra with early termination
 */
bool dijkstraToTarget(Graph* graph, int startId, int targetId, VisitResult* result) {
    if (!graph || !result) return false;
    
    Node* startNode = findNode(graph, startId);
    Node* targetNode = findNode(graph, targetId);
    
    if (!startNode || !targetNode) return false;
    
    int maxNodeId = 0;
    Node* node = graph->head;
    while (node) {
        if (node->id > maxNodeId) maxNodeId = node->id;
        node = node->next;
    }
    maxNodeId++;
    
    MinHeap* heap = createMinHeap(graph->nodeCount + 10, maxNodeId + 10);
    if (!heap) return false;
    
    for (int i = 0; i < maxNodeId; i++) {
        result->distances[i] = INFINITY_DISTANCE;
    }
    result->distances[startId] = 0;
    
    heapInsert(heap, startId, 0);
    
    bool* visited = (bool*)calloc(maxNodeId, sizeof(bool));
    if (!visited) {
        destroyMinHeap(&heap);
        return false;
    }
    
    while (!isHeapEmpty(heap)) {
        int currentId, currentDist;
        heapExtractMin(heap, &currentId, &currentDist);
        
        if (visited[currentId]) continue;
        visited[currentId] = true;
        addVisited(result, currentId);
        
        // Early termination!
        if (currentId == targetId) {
            free(visited);
            destroyMinHeap(&heap);
            return true;
        }
        
        Node* current = findNode(graph, currentId);
        if (!current) continue;
        
        Edge* edge = current->edges;
        while (edge) {
            int neighborId = edge->destination->id;
            int weight = edge->weight;
            
            if (!visited[neighborId]) {
                int newDist = currentDist + weight;
                
                if (newDist < result->distances[neighborId]) {
                    result->distances[neighborId] = newDist;
                    result->predecessors[neighborId] = currentId;
                    
                    if (heapContains(heap, neighborId)) {
                        heapDecreaseKey(heap, neighborId, newDist);
                    } else {
                        heapInsert(heap, neighborId, newDist);
                    }
                }
            }
            
            edge = edge->next;
        }
    }
    
    free(visited);
    destroyMinHeap(&heap);
    
    return result->distances[targetId] != INFINITY_DISTANCE;
}

/* ============================================================================
 * ALGORITHM UTILITIES
 * ============================================================================ */

/**
 * @brief Runs algorithm using function pointer strategy
 * 
 * This is the Strategy Pattern in C - allows runtime
 * selection of algorithms through function pointers.
 */
void runAlgorithm(Graph* graph, int startId, SearchStrategy strategy, VisitResult* result) {
    if (!graph || !strategy || !result) return;
    
    printf("\nRunning %s...\n", getAlgorithmName(strategy));
    printf("═══════════════════════════════════════\n");
    
    strategy(graph, startId, result);
    
    printf("Completed.\n");
}

const char* getAlgorithmName(SearchStrategy strategy) {
    if (strategy == (SearchStrategy)bfs) return "Breadth-First Search (BFS)";
    if (strategy == (SearchStrategy)dfs) return "Depth-First Search (DFS)";
    if (strategy == (SearchStrategy)dijkstra) return "Dijkstra's Algorithm";
    if (strategy == (SearchStrategy)dfsIterative) return "Iterative DFS";
    return "Unknown Algorithm";
}

void printShortestPath(VisitResult* result, int startId, int targetId) {
    if (!result) return;
    
    if (result->distances[targetId] == INFINITY_DISTANCE) {
        printf("No path exists from %d to %d\n", startId, targetId);
        return;
    }
    
    int path[result->capacity];
    int pathLength;
    
    if (reconstructPath(result, startId, targetId, path, &pathLength)) {
        printf("Shortest path from %d to %d (distance=%d): ", 
               startId, targetId, result->distances[targetId]);
        
        for (int i = 0; i < pathLength; i++) {
            printf("%d", path[i]);
            if (i < pathLength - 1) printf(" → ");
        }
        printf("\n");
    }
}

/* ============================================================================
 * ADVANCED ALGORITHMS
 * ============================================================================ */

bool isGraphConnected(Graph* graph) {
    if (!graph || graph->nodeCount == 0) return true;
    
    int maxNodeId = 0;
    Node* node = graph->head;
    while (node) {
        if (node->id > maxNodeId) maxNodeId = node->id;
        node = node->next;
    }
    maxNodeId++;
    
    VisitResult* result = createVisitResult(maxNodeId);
    if (!result) return false;
    
    // BFS from first node
    int startId = graph->head->id;
    bfs(graph, startId, result);
    
    bool connected = (result->count == graph->nodeCount);
    
    destroyVisitResult(&result);
    return connected;
}

int findConnectedComponents(Graph* graph, int* components) {
    if (!graph || !components) return 0;
    
    int maxNodeId = 0;
    Node* node = graph->head;
    while (node) {
        if (node->id > maxNodeId) maxNodeId = node->id;
        node = node->next;
    }
    maxNodeId++;
    
    // Initialize all nodes to unvisited (-1)
    for (int i = 0; i < maxNodeId; i++) {
        components[i] = -1;
    }
    
    int componentId = 0;
    node = graph->head;
    
    while (node) {
        if (components[node->id] == -1) {
            // New component - run BFS
            VisitResult* result = createVisitResult(maxNodeId);
            if (result) {
                bfs(graph, node->id, result);
                
                // Mark all visited nodes with component ID
                for (int i = 0; i < result->count; i++) {
                    components[result->visited[i]] = componentId;
                }
                
                destroyVisitResult(&result);
                componentId++;
            }
        }
        node = node->next;
    }
    
    return componentId;
}

bool hasCycle(Graph* graph) {
    if (!graph) return false;
    
    int maxNodeId = 0;
    Node* node = graph->head;
    while (node) {
        if (node->id > maxNodeId) maxNodeId = node->id;
        node = node->next;
    }
    maxNodeId++;
    
    // 0 = unvisited, 1 = visiting (in recursion stack), 2 = visited
    int* state = (int*)calloc(maxNodeId, sizeof(int));
    if (!state) return false;
    
    // DFS from each unvisited node
    node = graph->head;
    while (node) {
        if (state[node->id] == 0) {
            Stack* stack = createStack();
            push(stack, node->id);
            
            while (!isStackEmpty(stack)) {
                int currentId = pop(stack);
                
                if (state[currentId] == 1) {
                    // Found back edge - cycle!
                    destroyStack(stack);
                    free(state);
                    return true;
                }
                
                if (state[currentId] == 0) {
                    state[currentId] = 1;  // Mark as visiting
                    push(stack, currentId);  // Push again to detect back edge
                    
                    Node* current = findNode(graph, currentId);
                    Edge* edge = current->edges;
                    
                    while (edge) {
                        int neighborId = edge->destination->id;
                        if (state[neighborId] == 0) {
                            push(stack, neighborId);
                        } else if (state[neighborId] == 1) {
                            // Back edge found!
                            destroyStack(stack);
                            free(state);
                            return true;
                        }
                        edge = edge->next;
                    }
                    
                    state[currentId] = 2;  // Mark as visited
                }
            }
            
            destroyStack(stack);
        }
        node = node->next;
    }
    
    free(state);
    return false;
}

int topologicalSort(Graph* graph, int* sorted) {
    if (!graph || !sorted) return -1;
    
    // Check for cycles first
    if (hasCycle(graph)) {
        fprintf(stderr, "Error: Graph has cycle, topological sort not possible\n");
        return -1;
    }
    
    int maxNodeId = 0;
    Node* node = graph->head;
    while (node) {
        if (node->id > maxNodeId) maxNodeId = node->id;
        node = node->next;
    }
    maxNodeId++;
    
    // Calculate in-degrees
    int* inDegree = (int*)calloc(maxNodeId, sizeof(int));
    if (!inDegree) return -1;
    
    node = graph->head;
    while (node) {
        Edge* edge = node->edges;
        while (edge) {
            inDegree[edge->destination->id]++;
            edge = edge->next;
        }
        node = node->next;
    }
    
    // Queue for nodes with in-degree 0
    Queue* queue = createQueue();
    node = graph->head;
    while (node) {
        if (inDegree[node->id] == 0) {
            enqueue(queue, node->id);
        }
        node = node->next;
    }
    
    int sortedIdx = 0;
    
    while (!isQueueEmpty(queue)) {
        int currentId = dequeue(queue);
        sorted[sortedIdx++] = currentId;
        
        Node* current = findNode(graph, currentId);
        Edge* edge = current->edges;
        
        while (edge) {
            int neighborId = edge->destination->id;
            inDegree[neighborId]--;
            
            if (inDegree[neighborId] == 0) {
                enqueue(queue, neighborId);
            }
            
            edge = edge->next;
        }
    }
    
    destroyQueue(queue);
    free(inDegree);
    
    return sortedIdx;
}
