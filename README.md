# OPTIMIZED-DRONE-DELIVERY-ROUTING
DESIGN ANALYSIS AND ALGORITHM 

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#define INF INT_MAX
#define MAX_NODES 100

typedef struct {
    int x, y;
} Point;

typedef struct {
    int id;
    int cost;
} Node;

int graph[MAX_NODES][MAX_NODES];
int heuristic[MAX_NODES]; // Estimated cost to destination
int wind_resistance[MAX_NODES];  // Extra cost based on wind conditions
int battery_usage[MAX_NODES];    // Battery constraints
int no_fly_zone[MAX_NODES];      // No-fly penalty

int minDistance(int dist[], int visited[], int n) {
    int min = INF, min_index;
    for (int v = 0; v < n; v++) {
        if (!visited[v] && dist[v] <= min) {
            min = dist[v], min_index = v;
        }
    }
    return min_index;
}

// Dijkstra’s Algorithm for Shortest Path
void dijkstra(int src, int n) {
    int dist[MAX_NODES], visited[MAX_NODES];

    for (int i = 0; i < n; i++) {
        dist[i] = INF;
        visited[i] = 0;
    }
    dist[src] = 0;

    for (int count = 0; count < n - 1; count++) {
        int u = minDistance(dist, visited, n);
        visited[u] = 1;
        
        for (int v = 0; v < n; v++) {
            if (!visited[v] && graph[u][v] && dist[u] != INF && dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
            }
        }
    }

    printf("\nDijkstra's Shortest Path Distances:\n");
    for (int i = 0; i < n; i++) {
        printf("Node %d -> Distance: %d\n", i, dist[i]);
    }
}

// A* Algorithm considering real-time constraints
void aStar(int start, int goal, int n) {
    int dist[MAX_NODES], visited[MAX_NODES];

    for (int i = 0; i < n; i++) {
        dist[i] = INF;
        visited[i] = 0;
    }
    dist[start] = 0;

    for (int count = 0; count < n - 1; count++) {
        int u = minDistance(dist, visited, n);
        visited[u] = 1;

        for (int v = 0; v < n; v++) {
            if (!visited[v] && graph[u][v] && dist[u] != INF) {
                int realTimeCost = wind_resistance[v] + battery_usage[v] + no_fly_zone[v]; // Dynamic factors
                int newCost = dist[u] + graph[u][v] + heuristic[v] + realTimeCost;

                if (newCost < dist[v]) {
                    dist[v] = newCost;
                }
            }
        }
    }

    printf("\nA* Algorithm Path Distances:\n");
    for (int i = 0; i < n; i++) {
        printf("Node %d -> Distance: %d\n", i, dist[i]);
    }
}

// Main function
int main() {
    int n, edges, src, dest;

    printf("Enter number of nodes: ");
    scanf("%d", &n);

    printf("Enter number of edges: ");
    scanf("%d", &edges);

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            graph[i][j] = (i == j) ? 0 : INF;

    printf("Enter edges (src dest weight):\n");
    for (int i = 0; i < edges; i++) {
        int u, v, w;
        scanf("%d %d %d", &u, &v, &w);
        graph[u][v] = w;
        graph[v][u] = w; // Assuming undirected graph
    }

    printf("Enter real-time constraints (node wind_resistance battery_usage no_fly_zone):\n");
    for (int i = 0; i < n; i++) {
        scanf("%d %d %d", &wind_resistance[i], &battery_usage[i], &no_fly_zone[i]);
    }

    printf("Enter source node: ");
    scanf("%d", &src);

    printf("Enter goal node for A* Search: ");
    scanf("%d", &dest);

    // Set heuristic values (Euclidean distance)
    for (int i = 0; i < n; i++) {
        heuristic[i] = abs(i - dest); // Simplified heuristic for demo
    }

    dijkstra(src, n);
    aStar(src, dest, n);

    return 0;
}
