# gograph Graph Library ☊

The `gograph` library supports graph algorithms and data types in Go. Uses data structures from the [`gods` library](https://github.com/ParkerGits/gods).

View the full documentation for `gograph` on [godocs](https://godocs.io/github.com/ParkerGits/gograph).

## Installation

```
go get -u github.com/ParkerGits/gograph@latest
```

## Overview

The `gograph` library two graph types: AdjMat (Adjacency Matrix) and AdjList (Adjacency List). Both graph types come with functional-style iterator methods (e.g., ForEachEdge, FindEdge, and SomeEdge).

Implements a Graph interface and the following supported algorithms:

- Breadth-First Search
- Iterative Depth-First Search
- Recursive Depth-First Search
- Least Edges Path
- Is Bipartite
- Is Strongly Connected
- Is Cyclic
- Find Cycle
- Topological Ordering
- Dijkstra's Shortest Path
- Prim's Minimum Spanning Tree
- Kruskal's Minimum Spanning Tree
- K-Clustering
- Bellman-Ford Shortest Path

## Example Usage

### Topological Ordering Example

```go
func topOrderEx() {
	// Create a new Adjacency List graph
	adjList := gograph.NewAdjList(7)

	// Build the DAG from Figure 3.8 in Kleinberg/Tardos Algorithms
	adjList.AddDirectedEdge(0, 3, 1)
	adjList.AddDirectedEdge(0, 4, 1)
	adjList.AddDirectedEdge(0, 6, 1)
	adjList.AddDirectedEdge(1, 2, 1)
	adjList.AddDirectedEdge(1, 4, 1)
	adjList.AddDirectedEdge(1, 5, 1)
	adjList.AddDirectedEdge(2, 3, 1)
	adjList.AddDirectedEdge(2, 4, 1)
	adjList.AddDirectedEdge(3, 4, 1)
	adjList.AddDirectedEdge(4, 5, 1)
	adjList.AddDirectedEdge(4, 6, 1)
	adjList.AddDirectedEdge(5, 6, 1)

	// Get the graph's topological ordering.
	ordering, cycle := gograph.TopologicalOrdering(adjList)
	// Topological Ordering
	fmt.Println(*ordering) // [0 1 2 3 4 5 6]
	// Graph Cycles (none for DAG)
	fmt.Println(cycle) // nil
}
```

### Minimum Spanning Tree Example

```go
func mstEx() {
	// Create a new Adjacency Matrix graph
	adjMatrix := gograph.NewAdjMat(6)

	// Build the graph from slide 74 of Greedy Powerpoint
	adjMatrix.AddBothEdge(0,1,6)
	adjMatrix.AddBothEdge(0,2,1)
	adjMatrix.AddBothEdge(0,3,5)
	adjMatrix.AddBothEdge(1,2,5)
	adjMatrix.AddBothEdge(1,4,3)
	adjMatrix.AddBothEdge(2,4,6)
	adjMatrix.AddBothEdge(2,5,4)
	adjMatrix.AddBothEdge(2,3,5)
	adjMatrix.AddBothEdge(3,5,2)
	adjMatrix.AddBothEdge(4,5,6)

	// Get the edges and the cost of the Kruskal MST
	kruskalMst, kruskalCost := gograph.KruskalMST(adjMatrix)
	// Get the edges and the cost of the Prim MST
	primMst, primCost := gograph.PrimMST(adjMatrix)

	// Output: Kruskal Edges: [[0 2] [3 5] [1 4] [2 5] [1 2]], Cost: 15
	fmt.Printf("Kruskal Edges: %v, Cost: %d\n", *kruskalMst, kruskalCost)
	// Output: Prim Edges: [[0 2] [2 5] [5 3] [2 1] [1 4]], Cost: 15
	fmt.Printf("Prim Edges: %v, Cost: %d\n", *primMst, primCost)
}
```
