# 🧠 Memory-Driven Graph Intelligence Engine

> *"The kind of program that makes first-year C students cry softly."*

A sophisticated, pointer-heavy graph data structure implementation in C featuring dynamic memory management, self-referential structures, and multiple search algorithms.

![C](https://img.shields.io/badge/C-99-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Status](https://img.shields.io/badge/Status-Production-brightgreen.svg)

---

## 📋 Table of Contents

- [Features](#features)
- [Architecture](#architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Algorithms](#algorithms)
- [Demo Scenarios](#demo-scenarios)
- [API Reference](#api-reference)
- [Memory Management](#memory-management)

---

## ✨ Features

### Core Capabilities
- ✅ **Dynamic Graph Construction** - Add/remove nodes and edges at runtime
- ✅ **Weighted Edges** - Support for edge weights/costs
- ✅ **Self-Referential Structures** - Pure pointer cuisine with linked lists
- ✅ **Double Pointer Techniques** - Elegant list operations without special cases
- ✅ **Function Pointer Strategy** - Runtime algorithm selection
- ✅ **Memory Tracking** - Monitor allocation and usage

### Implemented Algorithms
| Algorithm | Time Complexity | Space Complexity | Use Case |
|-----------|-----------------|------------------|----------|
| **BFS** | O(V + E) | O(V) | Level-order traversal, shortest path (unweighted) |
| **DFS** | O(V + E) | O(V) | Path finding, topological sort, cycle detection |
| **Dijkstra** | O((V+E) log V) | O(V) | Shortest path (weighted, positive weights) |

### Advanced Features
- 🔥 **Min-Heap Priority Queue** - For efficient Dijkstra implementation
- 🔥 **Graph Cloning** - Deep copy with proper pointer remapping
- 🔥 **Cycle Detection** - Using DFS with state tracking
- 🔥 **Connected Components** - Find disjoint subgraphs
- 🔥 **Topological Sort** - For directed acyclic graphs

---

## 🏗 Architecture

### Data Structure Hierarchy

```
Graph
  └── head → Node → Node → Node → ...
                │       │       │
                ↓       ↓       ↓
              Edge    Edge    Edge
                │       │       │
                ↓       ↓       ↓
              Node    Node    Node
```

### Core Structures

```c
/* Self-referential struct - Edge contains pointer to Node */
typedef struct Edge {
    int weight;
    struct Node* destination;  // Pointer to destination node
    struct Edge* next;          // Next edge in adjacency list
} Edge;

/* Node with self-referential edge list */
typedef struct Node {
    int id;
    Edge* edges;           // Pointer to adjacency list
    struct Node* next;     // Next node in graph
} Node;

/* Graph container */
typedef struct Graph {
    Node* head;        // First node in graph
    int nodeCount;
    int edgeCount;
} Graph;
```

### Function Pointer Strategy Pattern

```c
typedef void (*SearchStrategy)(Graph*, int, VisitResult*);

void runAlgorithm(Graph* g, int start, SearchStrategy strategy) {
    strategy(g, start);
}

// Usage
runAlgorithm(graph, 0, bfs);
runAlgorithm(graph, 0, (SearchStrategy)dijkstra);
```

---

## 🚀 Installation

### Prerequisites
- GCC compiler (or compatible C compiler)
- Make utility
- Unix-like environment (Linux/macOS/WSL)

### Build Instructions

```bash
# Clone or extract the project
cd graph_engine

# Build release version (optimized)
make

# Build debug version (with symbols)
make debug

# Clean build artifacts
make clean

# Run the program
make run
```

### Build Targets

| Target | Description |
|--------|-------------|
| `make` | Build optimized release |
| `make debug` | Build with debug symbols |
| `make clean` | Remove all build files |
| `make run` | Build and run |
| `make valgrind` | Run with memory leak detection |

---

## 💻 Usage

### Interactive CLI

Launch the program and use the menu system:

```bash
./graph_engine
```

### Menu Options

```
┌────────────────────────────────────────┐
│         MAIN MENU                      │
├────────────────────────────────────────┤
│  1. Add Node                           │
│  2. Add Edge                           │
│  3. Remove Node                        │
│  4. Remove Edge                        │
│  5. View Graph                         │
│  6. Run Search Algorithm               │
│  7. Find Shortest Path                 │
│  8. Memory Usage                       │
│  9. Demo Scenarios                     │
│ 10. Clear Graph                        │
│  0. Exit                               │
└────────────────────────────────────────┘
```

### Quick Example

```
> 1  (Add Node)
Enter node ID: 0
✓ Node 0 added successfully

> 1  (Add Node)
Enter node ID: 1
✓ Node 1 added successfully

> 2  (Add Edge)
Enter source node ID: 0
Enter destination node ID: 1
Enter weight: 5
Directed (d) or Undirected (u)? u
✓ Edge 0 → 1 (weight=5) added successfully

> 6  (Run Algorithm)
Select algorithm: 4 (Dijkstra)
Enter start node ID: 0

Running Dijkstra's Algorithm from node 0...
```

---

## 🔬 Algorithms

### Breadth-First Search (BFS)

Explores graph level by level using a queue.

```c
VisitResult* result = createVisitResult(maxNodeId);
bfs(graph, startNode, result);

// Result contains visited order and distances
for (int i = 0; i < result->count; i++) {
    printf("Visited: %d\n", result->visited[i]);
}
```

**Best for:**
- Finding shortest path in unweighted graphs
- Level-order traversal
- Finding connected components

### Depth-First Search (DFS)

Explores as far as possible along each branch before backtracking.

```c
VisitResult* result = createVisitResult(maxNodeId);
dfs(graph, startNode, result);
```

**Best for:**
- Path finding
- Topological sorting
- Cycle detection
- Maze solving

### Dijkstra's Algorithm

Finds shortest paths from a source node to all other nodes.

```c
VisitResult* result = createVisitResult(maxNodeId);
dijkstra(graph, startNode, result);

// Get shortest path to target
printShortestPath(result, startNode, targetNode);
```

**Best for:**
- GPS navigation systems
- Network routing
- Weighted shortest path problems

---

## 🎮 Demo Scenarios

### 1. Network Topology (Router Network)

Simulates a network of routers with latency weights.

```
Router 0 ──5ms── Router 1 ──8ms── Router 3 ──6ms── Router 5
    │                │                │
   10ms            3ms              2ms
    │                │                │
Router 2 ──7ms── Router 4 ──4ms──────┘
```

**Demonstrates:** Finding optimal routing paths using Dijkstra.

### 2. Social Network (Friend Connections)

Models people as nodes with connection strength weights.

**Demonstrates:** Finding strongest connection paths, connected components.

### 3. Maze Solver (Grid Pathfinding)

4×4 grid with walls represented as missing edges.

**Demonstrates:** BFS vs Dijkstra comparison, path reconstruction.

### 4. Algorithm Race

Runs BFS, DFS, and Dijkstra on the same graph with timing.

**Demonstrates:** Performance characteristics of each algorithm.

---

## 📚 API Reference

### Graph Lifecycle

```c
Graph* createGraph(void);                    // Create empty graph
void destroyGraph(Graph** graph);            // Free all memory
void clearGraph(Graph* graph);               // Remove all nodes/edges
```

### Node Operations

```c
Node* addNode(Graph* graph, int id);         // Add node with ID
Node* findNode(Graph* graph, int id);        // Find node by ID
bool removeNode(Graph* graph, int id);       // Remove node and edges
bool hasNode(Graph* graph, int id);          // Check existence
```

### Edge Operations

```c
bool addEdge(Graph* graph, int src, int dest, int weight);
bool addUndirectedEdge(Graph* graph, int n1, int n2, int weight);
bool removeEdge(Graph* graph, int src, int dest);
bool hasEdge(Graph* graph, int src, int dest);
int getEdgeWeight(Graph* graph, int src, int dest);
```

### Algorithms

```c
void bfs(Graph* graph, int startId, VisitResult* result);
void dfs(Graph* graph, int startId, VisitResult* result);
void dfsIterative(Graph* graph, int startId, VisitResult* result);
void dijkstra(Graph* graph, int startId, VisitResult* result);
bool dijkstraShortestPath(Graph* graph, int start, int target,
                          int* distance, int* path, int* pathLen);
```

### Advanced Operations

```c
Graph* cloneGraph(Graph* source);            // Deep copy
bool isGraphConnected(Graph* graph);
int findConnectedComponents(Graph* graph, int* components);
bool hasCycle(Graph* graph);
int topologicalSort(Graph* graph, int* sorted);
```

### Memory Management

```c
size_t getGraphMemoryUsage(Graph* graph);
bool validateGraph(Graph* graph);
```

---

## 🧠 Memory Management

### Double Pointer Technique

The `destroyGraph` function uses a double pointer to prevent dangling pointers:

```c
void destroyGraph(Graph** graph) {
    // ... free all memory ...
    *graph = NULL;  // Sets caller's pointer to NULL
}

// Usage
Graph* g = createGraph();
// ... use graph ...
destroyGraph(&g);  // g is now NULL
```

### Memory Ownership Rules

1. **Graph owns Nodes** - Graph is responsible for freeing all nodes
2. **Node owns Edges** - Each node frees its adjacency list
3. **Edge references Nodes** - Edges don't own destination nodes (weak reference)

### Memory Layout

```
Stack                    Heap
─────                    ────
graph ────────→ [Graph struct]
                     │
                     ↓
                [Node] ──next──→ [Node] ──next──→ NULL
                   │                │
                   ↓                ↓
                [Edge]            [Edge]
                   │                │
                   ↓                ↓
                [Node*]          [Node*]
              (dest ptr)       (dest ptr)
```

### Memory Usage Report

Access via menu option 8:

```
╔════════════════════════════════════════════════╗
║         MEMORY USAGE REPORT                    ║
╠════════════════════════════════════════════════╣
║ Graph Structure:     24 bytes                  ║
║ Per Node:            24 bytes                  ║
║ Per Edge:            24 bytes                  ║
║────────────────────────────────────────────────║
║ Total Nodes:         6                         ║
║ Total Edges:         8                         ║
║────────────────────────────────────────────────║
║ Total Memory Used:   408 bytes (0.40 KB)       ║
╚════════════════════════════════════════════════╝
```

---

## 🎓 Educational Value

### Pointer Concepts Demonstrated

| Concept | Implementation |
|---------|----------------|
| **Self-referential structs** | `struct Node { struct Node* next; }` |
| **Double pointers** | `void createNode(Node** headRef, int id)` |
| **Pointer traversal** | `while (node && node->id != target)` |
| **Pointer linking** | `newEdge->next = src->edges; src->edges = newEdge;` |
| **Function pointers** | `typedef void (*SearchStrategy)(Graph*, int)` |
| **Dynamic allocation** | `malloc(sizeof(Node))` |
| **Memory ownership** | `destroyGraph(&graph)` sets pointer to NULL |

### Data Structures Implemented

- ✅ Linked List (node list, adjacency lists)
- ✅ Queue (for BFS)
- ✅ Stack (for iterative DFS)
- ✅ Binary Min-Heap (for Dijkstra)
- ✅ Priority Queue (heap-based)

---

## 📁 File Structure

```
graph_engine/
├── graph.h          # Core data structures and graph API
├── graph.c          # Graph implementation
├── heap.h           # Min-heap priority queue API
├── heap.c           # Heap implementation
├── algorithms.h     # Search algorithm declarations
├── algorithms.c     # BFS, DFS, Dijkstra implementations
├── main.c           # CLI interface and demo scenarios
├── Makefile         # Build configuration
└── README.md        # This file
```

---

## 🔧 Advanced Usage

### Custom Traversal Callback

```c
void myCallback(Node* node, void* userData) {
    int* count = (int*)userData;
    (*count)++;
    printf("Visited node %d\n", node->id);
}

int count = 0;
traverseNodes(graph, myCallback, &count);
```

### Path Reconstruction

```c
VisitResult* result = createVisitResult(maxId);
dijkstra(graph, start, result);

int path[100];
int pathLength;
if (reconstructPath(result, start, target, path, &pathLength)) {
    printf("Path: ");
    for (int i = 0; i < pathLength; i++) {
        printf("%d ", path[i]);
    }
}
```

---

## 📝 License

MIT License - Feel free to use, modify, and distribute.

---

## 🙏 Acknowledgments

- Inspired by classic graph algorithm textbooks (CLRS, Sedgewick)
- Built for educational purposes in advanced C programming
- Designed to demonstrate proper memory management in C

---

## 💀 Warning

> "One missed `free` and your program becomes a memory leak documentary."

This code is intentionally pointer-heavy for educational purposes. Always:
- Check return values from `malloc`
- Use `destroyGraph(&graph)` when done
- Run with Valgrind to detect leaks: `make valgrind`

---

**Happy Graph Traversing! 🚀**
