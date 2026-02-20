/**
 * @file main.c
 * @brief CLI-Based Network Simulator
 * 
 * A sophisticated command-line interface for the Graph Intelligence Engine
 * featuring:
 * - Interactive menu system
 * - Function pointer strategy selection
 * - Network simulation scenarios
 * - Memory usage tracking
 * - Graph visualization
 */

#include "graph.h"
#include "algorithms.h"
#include <ctype.h>
#include <time.h>
#include <string.h>

/* ============================================================================
 * CONSTANTS AND MACROS
 * ============================================================================ */

#define VERSION "2.0.0"
#define MAX_NODES 1000
#define MAX_INPUT 256

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

// Menu functions
void printBanner(void);
void printMainMenu(void);
void printAlgorithmMenu(void);
void printDemoMenu(void);

// Command handlers
void handleAddNode(Graph* graph);
void handleAddEdge(Graph* graph);
void handleRemoveNode(Graph* graph);
void handleRemoveEdge(Graph* graph);
void handleViewGraph(Graph* graph);
void handleRunAlgorithm(Graph* graph);
void handleFindPath(Graph* graph);
void handleMemoryUsage(Graph* graph);
void handleClearGraph(Graph* graph);
void handleDemo(Graph* graph);

// Demo scenarios
void demoNetworkTopology(Graph* graph);
void demoSocialNetwork(Graph* graph);
void demoMazeSolver(Graph* graph);
void demoShortestPathRace(Graph* graph);

// Utility functions
void clearScreen(void);
void pauseScreen(void);
int getIntInput(const char* prompt);
void getStringInput(const char* prompt, char* buffer, int size);
bool confirmAction(const char* message);

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

int main(int argc, char* argv[]) {
    (void)argc;  /* Unused */
    (void)argv;  /* Unused */
    
    srand((unsigned int)time(NULL));
    
    Graph* graph = createGraph();
    if (!graph) {
        fprintf(stderr, "Fatal Error: Could not create graph\n");
        return 1;
    }
    
    int choice;
    bool running = true;
    
    clearScreen();
    printBanner();
    
    while (running) {
        printMainMenu();
        choice = getIntInput("Enter choice");
        
        switch (choice) {
            case 1:
                handleAddNode(graph);
                break;
            case 2:
                handleAddEdge(graph);
                break;
            case 3:
                handleRemoveNode(graph);
                break;
            case 4:
                handleRemoveEdge(graph);
                break;
            case 5:
                handleViewGraph(graph);
                break;
            case 6:
                handleRunAlgorithm(graph);
                break;
            case 7:
                handleFindPath(graph);
                break;
            case 8:
                handleMemoryUsage(graph);
                break;
            case 9:
                handleDemo(graph);
                break;
            case 10:
                handleClearGraph(graph);
                break;
            case 0:
                if (confirmAction("Are you sure you want to exit?")) {
                    running = false;
                }
                break;
            default:
                printf("Invalid choice. Please try again.\n");
                pauseScreen();
        }
    }
    
    printf("\nCleaning up and exiting...\n");
    destroyGraph(&graph);
    
    printf("Goodbye!\n");
    return 0;
}

/* ============================================================================
 * MENU DISPLAY FUNCTIONS
 * ============================================================================ */

void printBanner(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║                                                                  ║\n");
    printf("║     🧠 MEMORY-DRIVEN GRAPH INTELLIGENCE ENGINE 🧠               ║\n");
    printf("║                                                                  ║\n");
    printf("║          Pointer Cuisine for the Discerning Programmer          ║\n");
    printf("║                      Version %s                                  ║\n", VERSION);
    printf("║                                                                  ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

void printMainMenu(void) {
    printf("\n");
    printf("┌────────────────────────────────────────┐\n");
    printf("│         MAIN MENU                      │\n");
    printf("├────────────────────────────────────────┤\n");
    printf("│  1. Add Node                           │\n");
    printf("│  2. Add Edge                           │\n");
    printf("│  3. Remove Node                        │\n");
    printf("│  4. Remove Edge                        │\n");
    printf("│  5. View Graph                         │\n");
    printf("│  6. Run Search Algorithm               │\n");
    printf("│  7. Find Shortest Path                 │\n");
    printf("│  8. Memory Usage                       │\n");
    printf("│  9. Demo Scenarios                     │\n");
    printf("│ 10. Clear Graph                        │\n");
    printf("│  0. Exit                               │\n");
    printf("└────────────────────────────────────────┘\n");
}

void printAlgorithmMenu(void) {
    printf("\n");
    printf("┌────────────────────────────────────────┐\n");
    printf("│      SELECT ALGORITHM                  │\n");
    printf("├────────────────────────────────────────┤\n");
    printf("│ 1. Breadth-First Search (BFS)          │\n");
    printf("│ 2. Depth-First Search (DFS)            │\n");
    printf("│ 3. Iterative DFS                       │\n");
    printf("│ 4. Dijkstra's Shortest Path            │\n");
    printf("└────────────────────────────────────────┘\n");
}

void printDemoMenu(void) {
    printf("\n");
    printf("┌────────────────────────────────────────┐\n");
    printf("│      DEMO SCENARIOS                    │\n");
    printf("├────────────────────────────────────────┤\n");
    printf("│ 1. Network Topology (Router Network)   │\n");
    printf("│ 2. Social Network (Friend Connections) │\n");
    printf("│ 3. Maze Solver (Grid-based Pathfinding)│\n");
    printf("│ 4. Algorithm Race (BFS vs DFS vs Dijk) │\n");
    printf("└────────────────────────────────────────┘\n");
}

/* ============================================================================
 * COMMAND HANDLERS
 * ============================================================================ */

void handleAddNode(Graph* graph) {
    printf("\n┌── ADD NODE ──┐\n");
    
    int id = getIntInput("Enter node ID");
    
    if (id < 0 || id >= MAX_NODES) {
        printf("Error: Node ID must be between 0 and %d\n", MAX_NODES - 1);
        pauseScreen();
        return;
    }
    
    if (addNode(graph, id)) {
        printf("✓ Node %d added successfully\n", id);
    } else {
        printf("✗ Failed to add node %d (may already exist)\n", id);
    }
    
    pauseScreen();
}

void handleAddEdge(Graph* graph) {
    printf("\n┌── ADD EDGE ──┐\n");
    
    int src = getIntInput("Enter source node ID");
    int dest = getIntInput("Enter destination node ID");
    int weight = getIntInput("Enter weight");
    
    if (weight < 0) {
        printf("Note: Negative weights not supported for Dijkstra's algorithm\n");
    }
    
    char type;
    printf("Directed (d) or Undirected (u)? ");
    scanf(" %c", &type);
    getchar();  // Consume newline
    
    bool success;
    if (tolower(type) == 'u') {
        success = addUndirectedEdge(graph, src, dest, weight);
    } else {
        success = addEdge(graph, src, dest, weight);
    }
    
    if (success) {
        printf("✓ Edge %d → %d (weight=%d) added successfully\n", src, dest, weight);
    } else {
        printf("✗ Failed to add edge (nodes may not exist)\n");
    }
    
    pauseScreen();
}

void handleRemoveNode(Graph* graph) {
    printf("\n┌── REMOVE NODE ──┐\n");
    
    int id = getIntInput("Enter node ID to remove");
    
    if (confirmAction("This will also remove all connected edges. Continue?")) {
        if (removeNode(graph, id)) {
            printf("✓ Node %d removed successfully\n", id);
        } else {
            printf("✗ Node %d not found\n", id);
        }
    }
    
    pauseScreen();
}

void handleRemoveEdge(Graph* graph) {
    printf("\n┌── REMOVE EDGE ──┐\n");
    
    int src = getIntInput("Enter source node ID");
    int dest = getIntInput("Enter destination node ID");
    
    if (removeEdge(graph, src, dest)) {
        printf("✓ Edge %d → %d removed successfully\n", src, dest);
    } else {
        printf("✗ Edge not found\n");
    }
    
    pauseScreen();
}

void handleViewGraph(Graph* graph) {
    printGraph(graph);
    pauseScreen();
}

void handleRunAlgorithm(Graph* graph) {
    if (graph->nodeCount == 0) {
        printf("Graph is empty. Add some nodes first.\n");
        pauseScreen();
        return;
    }
    
    printAlgorithmMenu();
    int algoChoice = getIntInput("Select algorithm");
    
    SearchStrategy strategy = NULL;
    const char* algoName = "";
    
    switch (algoChoice) {
        case 1:
            strategy = (SearchStrategy)bfs;
            algoName = "BFS";
            break;
        case 2:
            strategy = (SearchStrategy)dfs;
            algoName = "DFS";
            break;
        case 3:
            strategy = (SearchStrategy)dfsIterative;
            algoName = "Iterative DFS";
            break;
        case 4:
            strategy = (SearchStrategy)dijkstra;
            algoName = "Dijkstra";
            break;
        default:
            printf("Invalid algorithm choice\n");
            pauseScreen();
            return;
    }
    
    int startId = getIntInput("Enter start node ID");
    
    if (!hasNode(graph, startId)) {
        printf("Error: Node %d not found\n", startId);
        pauseScreen();
        return;
    }
    
    // Find max node ID for result array
    int maxNodeId = 0;
    Node* node = graph->head;
    while (node) {
        if (node->id > maxNodeId) maxNodeId = node->id;
        node = node->next;
    }
    maxNodeId++;
    
    VisitResult* result = createVisitResult(maxNodeId);
    if (!result) {
        printf("Error: Could not allocate result structure\n");
        pauseScreen();
        return;
    }
    
    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║  Running %s from node %d\n", algoName, startId);
    printf("╚════════════════════════════════════════════════╝\n");
    
    clock_t start = clock();
    runAlgorithm(graph, startId, strategy, result);
    clock_t end = clock();
    
    double timeMs = ((double)(end - start)) / CLOCKS_PER_SEC * 1000;
    
    printVisitResult(result);
    printf("\nExecution time: %.3f ms\n", timeMs);
    
    destroyVisitResult(&result);
    pauseScreen();
}

void handleFindPath(Graph* graph) {
    if (graph->nodeCount == 0) {
        printf("Graph is empty. Add some nodes first.\n");
        pauseScreen();
        return;
    }
    
    int startId = getIntInput("Enter start node ID");
    int targetId = getIntInput("Enter target node ID");
    
    if (!hasNode(graph, startId) || !hasNode(graph, targetId)) {
        printf("Error: One or both nodes not found\n");
        pauseScreen();
        return;
    }
    
    // Find max node ID
    int maxNodeId = 0;
    Node* node = graph->head;
    while (node) {
        if (node->id > maxNodeId) maxNodeId = node->id;
        node = node->next;
    }
    maxNodeId++;
    
    VisitResult* result = createVisitResult(maxNodeId);
    if (!result) {
        printf("Error: Could not allocate result structure\n");
        pauseScreen();
        return;
    }
    
    printf("\nFinding shortest path from %d to %d...\n", startId, targetId);
    
    clock_t start = clock();
    bool found = dijkstraToTarget(graph, startId, targetId, result);
    clock_t end = clock();
    
    double timeMs = ((double)(end - start)) / CLOCKS_PER_SEC * 1000;
    
    if (found) {
        printShortestPath(result, startId, targetId);
    } else {
        printf("No path exists from %d to %d\n", startId, targetId);
    }
    
    printf("Execution time: %.3f ms\n", timeMs);
    
    destroyVisitResult(&result);
    pauseScreen();
}

void handleMemoryUsage(Graph* graph) {
    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║         MEMORY USAGE REPORT                    ║\n");
    printf("╠════════════════════════════════════════════════╣\n");
    printf("║ Graph Structure:     %zu bytes\n", sizeof(Graph));
    printf("║ Per Node:            %zu bytes\n", sizeof(Node));
    printf("║ Per Edge:            %zu bytes\n", sizeof(Edge));
    printf("║────────────────────────────────────────────────║\n");
    printf("║ Total Nodes:         %d\n", graph->nodeCount);
    printf("║ Total Edges:         %d\n", graph->edgeCount);
    printf("║────────────────────────────────────────────────║\n");
    printf("║ Total Memory Used:   %zu bytes (%.2f KB)\n", 
           getGraphMemoryUsage(graph),
           getGraphMemoryUsage(graph) / 1024.0);
    printf("╚════════════════════════════════════════════════╝\n");
    
    pauseScreen();
}

void handleClearGraph(Graph* graph) {
    if (graph->nodeCount == 0) {
        printf("Graph is already empty.\n");
        pauseScreen();
        return;
    }
    
    if (confirmAction("Are you sure you want to clear the entire graph?")) {
        clearGraph(graph);
        printf("✓ Graph cleared successfully\n");
    }
    
    pauseScreen();
}

void handleDemo(Graph* graph) {
    printDemoMenu();
    int demoChoice = getIntInput("Select demo");
    
    // Clear existing graph for demo
    if (graph->nodeCount > 0) {
        if (!confirmAction("This will clear the current graph. Continue?")) {
            return;
        }
        clearGraph(graph);
    }
    
    switch (demoChoice) {
        case 1:
            demoNetworkTopology(graph);
            break;
        case 2:
            demoSocialNetwork(graph);
            break;
        case 3:
            demoMazeSolver(graph);
            break;
        case 4:
            demoShortestPathRace(graph);
            break;
        default:
            printf("Invalid demo choice\n");
    }
    
    pauseScreen();
}

/* ============================================================================
 * DEMO SCENARIOS
 * ============================================================================ */

void demoNetworkTopology(Graph* graph) {
    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║     NETWORK TOPOLOGY DEMO                      ║\n");
    printf("║     Simulating a router network                ║\n");
    printf("╚════════════════════════════════════════════════╝\n");
    
    // Create router network
    // Routers 0-5 with various connections
    for (int i = 0; i <= 5; i++) {
        addNode(graph, i);
    }
    
    // Create mesh-like topology with weights (latency in ms)
    addUndirectedEdge(graph, 0, 1, 5);
    addUndirectedEdge(graph, 0, 2, 10);
    addUndirectedEdge(graph, 1, 2, 3);
    addUndirectedEdge(graph, 1, 3, 8);
    addUndirectedEdge(graph, 2, 4, 7);
    addUndirectedEdge(graph, 3, 4, 2);
    addUndirectedEdge(graph, 3, 5, 6);
    addUndirectedEdge(graph, 4, 5, 4);
    
    printGraph(graph);
    
    printf("\nScenario: Find best route from Router 0 to Router 5\n");
    
    int maxNodeId = 6;
    VisitResult* result = createVisitResult(maxNodeId);
    
    dijkstra(graph, 0, result);
    printShortestPath(result, 0, 5);
    
    destroyVisitResult(&result);
    
    printf("\nNetwork Analysis:\n");
    printf("  - Is connected: %s\n", isGraphConnected(graph) ? "Yes" : "No");
    printf("  - Has cycles: %s\n", hasCycle(graph) ? "Yes" : "No");
}

void demoSocialNetwork(Graph* graph) {
    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║     SOCIAL NETWORK DEMO                        ║\n");
    printf("║     Friend connections with influence weights  ║\n");
    printf("╚════════════════════════════════════════════════╝\n");
    
    // Create people nodes
    char* names[] = {"Alice", "Bob", "Carol", "David", "Eve", "Frank", "Grace"};
    for (int i = 0; i < 7; i++) {
        addNode(graph, i);
    }
    
    printf("\nPeople in network:\n");
    for (int i = 0; i < 7; i++) {
        printf("  %d: %s\n", i, names[i]);
    }
    
    // Create friend connections (undirected, weight = connection strength)
    addUndirectedEdge(graph, 0, 1, 8);   // Alice - Bob
    addUndirectedEdge(graph, 0, 2, 5);   // Alice - Carol
    addUndirectedEdge(graph, 1, 3, 7);   // Bob - David
    addUndirectedEdge(graph, 2, 3, 6);   // Carol - David
    addUndirectedEdge(graph, 2, 4, 9);   // Carol - Eve
    addUndirectedEdge(graph, 3, 5, 4);   // David - Frank
    addUndirectedEdge(graph, 4, 6, 7);   // Eve - Grace
    addUndirectedEdge(graph, 5, 6, 6);   // Frank - Grace
    
    printGraph(graph);
    
    printf("\nScenario: Find strongest connection path from Alice to Grace\n");
    printf("(Note: Lower weight = stronger connection)\n\n");
    
    int maxNodeId = 7;
    VisitResult* result = createVisitResult(maxNodeId);
    
    dijkstra(graph, 0, result);
    printShortestPath(result, 0, 6);
    
    destroyVisitResult(&result);
    
    // Find connected components
    int components[7];
    int numComponents = findConnectedComponents(graph, components);
    printf("\nConnected components: %d\n", numComponents);
}

void demoMazeSolver(Graph* graph) {
    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║     MAZE SOLVER DEMO                           ║\n");
    printf("║     4x4 Grid Pathfinding                       ║\n");
    printf("╚════════════════════════════════════════════════╝\n");
    
    // Create 4x4 grid
    // Node IDs: row * 4 + col
    for (int i = 0; i < 16; i++) {
        addNode(graph, i);
    }
    
    // Create grid connections (right and down)
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            int current = row * 4 + col;
            
            // Connect right
            if (col < 3) {
                addUndirectedEdge(graph, current, current + 1, 1);
            }
            
            // Connect down
            if (row < 3) {
                addUndirectedEdge(graph, current, current + 4, 1);
            }
        }
    }
    
    // Add some "walls" by removing edges
    removeEdge(graph, 1, 2);
    removeEdge(graph, 2, 1);
    removeEdge(graph, 5, 6);
    removeEdge(graph, 6, 5);
    removeEdge(graph, 9, 10);
    removeEdge(graph, 10, 9);
    
    printf("\nMaze Layout (S=start, E=end, #=wall):\n");
    printf("  S  #  .  .\n");
    printf("  .  #  .  .\n");
    printf("  .  #  .  .\n");
    printf("  .  .  .  E\n");
    
    printf("\nNode numbering:\n");
    for (int row = 0; row < 4; row++) {
        printf("  ");
        for (int col = 0; col < 4; col++) {
            printf("%2d ", row * 4 + col);
        }
        printf("\n");
    }
    
    int start = 0;   // Top-left
    int end = 15;    // Bottom-right
    
    printf("\nFinding path from %d to %d...\n", start, end);
    
    int maxNodeId = 16;
    VisitResult* result = createVisitResult(maxNodeId);
    
    // Compare BFS and Dijkstra (should be same for unweighted)
    printf("\n--- BFS Traversal ---\n");
    bfs(graph, start, result);
    printVisitResult(result);
    
    // Reset result
    destroyVisitResult(&result);
    result = createVisitResult(maxNodeId);
    
    printf("\n--- Dijkstra Shortest Path ---\n");
    dijkstra(graph, start, result);
    printShortestPath(result, start, end);
    
    destroyVisitResult(&result);
}

void demoShortestPathRace(Graph* graph) {
    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║     ALGORITHM RACE DEMO                        ║\n");
    printf("║     Comparing BFS, DFS, and Dijkstra           ║\n");
    printf("╚════════════════════════════════════════════════╝\n");
    
    // Create a more complex graph
    for (int i = 0; i < 8; i++) {
        addNode(graph, i);
    }
    
    // Create edges with varying weights
    addEdge(graph, 0, 1, 4);
    addEdge(graph, 0, 2, 2);
    addEdge(graph, 1, 3, 5);
    addEdge(graph, 2, 1, 1);
    addEdge(graph, 2, 3, 8);
    addEdge(graph, 2, 4, 10);
    addEdge(graph, 3, 5, 2);
    addEdge(graph, 3, 6, 6);
    addEdge(graph, 4, 3, 3);
    addEdge(graph, 4, 6, 4);
    addEdge(graph, 5, 6, 1);
    addEdge(graph, 5, 7, 5);
    addEdge(graph, 6, 7, 3);
    
    printGraph(graph);
    
    int start = 0;
    int maxNodeId = 8;
    
    printf("\nRunning all algorithms from node %d...\n\n", start);
    
    // BFS
    VisitResult* bfsResult = createVisitResult(maxNodeId);
    clock_t bfsStart = clock();
    bfs(graph, start, bfsResult);
    clock_t bfsEnd = clock();
    double bfsTime = ((double)(bfsEnd - bfsStart)) / CLOCKS_PER_SEC * 1000;
    
    printf("BFS completed in %.3f ms\n", bfsTime);
    printf("  Visited order: ");
    for (int i = 0; i < bfsResult->count; i++) {
        printf("%d ", bfsResult->visited[i]);
    }
    printf("\n");
    
    // DFS
    VisitResult* dfsResult = createVisitResult(maxNodeId);
    clock_t dfsStart = clock();
    dfs(graph, start, dfsResult);
    clock_t dfsEnd = clock();
    double dfsTime = ((double)(dfsEnd - dfsStart)) / CLOCKS_PER_SEC * 1000;
    
    printf("\nDFS completed in %.3f ms\n", dfsTime);
    printf("  Visited order: ");
    for (int i = 0; i < dfsResult->count; i++) {
        printf("%d ", dfsResult->visited[i]);
    }
    printf("\n");
    
    // Dijkstra
    VisitResult* dijkResult = createVisitResult(maxNodeId);
    clock_t dijkStart = clock();
    dijkstra(graph, start, dijkResult);
    clock_t dijkEnd = clock();
    double dijkTime = ((double)(dijkEnd - dijkStart)) / CLOCKS_PER_SEC * 1000;
    
    printf("\nDijkstra completed in %.3f ms\n", dijkTime);
    printf("  Shortest distances from %d:\n", start);
    for (int i = 0; i < maxNodeId; i++) {
        if (dijkResult->distances[i] != INFINITY_DISTANCE) {
            printf("    to %d: %d\n", i, dijkResult->distances[i]);
        }
    }
    
    printf("\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║         PERFORMANCE SUMMARY                    ║\n");
    printf("╠════════════════════════════════════════════════╣\n");
    printf("║ BFS:      %8.3f ms  (Level-order)         ║\n", bfsTime);
    printf("║ DFS:      %8.3f ms  (Depth-first)         ║\n", dfsTime);
    printf("║ Dijkstra: %8.3f ms  (Shortest path)       ║\n", dijkTime);
    printf("╚════════════════════════════════════════════════╝\n");
    
    destroyVisitResult(&bfsResult);
    destroyVisitResult(&dfsResult);
    destroyVisitResult(&dijkResult);
}

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

void clearScreen(void) {
    #ifdef _WIN32
        system("cls");
    #else
        system("clear");
    #endif
}

void pauseScreen(void) {
    printf("\nPress Enter to continue...");
    getchar();
}

int getIntInput(const char* prompt) {
    int value;
    printf("%s: ", prompt);
    
    while (scanf("%d", &value) != 1) {
        printf("Invalid input. Please enter a number: ");
        while (getchar() != '\n');  // Clear input buffer
    }
    getchar();  // Consume newline
    
    return value;
}

void getStringInput(const char* prompt, char* buffer, int size) {
    printf("%s: ", prompt);
    if (fgets(buffer, size, stdin)) {
        // Remove trailing newline
        size_t len = strlen(buffer);
        if (len > 0 && buffer[len - 1] == '\n') {
            buffer[len - 1] = '\0';
        }
    }
}

bool confirmAction(const char* message) {
    char response;
    printf("%s (y/n): ", message);
    scanf(" %c", &response);
    getchar();  // Consume newline
    return tolower(response) == 'y';
}
