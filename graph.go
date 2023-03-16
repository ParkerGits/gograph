package gograph

import (
	"github.com/ParkerGits/gods"
	"fmt"
	"math"
	"sort"
)

type Graph interface {
	// Calls f for every edge in the graph.
	ForEachEdge(f func(node, neighbor, weight int))
	// Calls f for every edge incident to node in the graph.
	ForEachIncidentEdge(node int, f func(node, neighbor, weight int))
	// Returns true if some edge in the graph satisfies the predicate f. Returns false otherwise.
	SomeEdge(f func(node, neighbor, weight int) bool) bool
	// Returns true if some edge incident to node in the graph satisfies the predicate f. Returns false otherwise.
	SomeIncidentEdge(node int, f func(node, neighbor, weight int) bool) bool
	// Returns true if every edge in the graph satisfies the predicate f. Returns false otherwise.
	EveryEdge(f func(node, neighbor, weight int) bool) bool
	// Returns a slice containing every edge in the graph. Each edge represented by the array [node, neighbor, weight].
	Edges() *[][3]int
	// Returns the first edge in the graph that satisfies the predicate f. Returns nil if no edges satisfy. The returned edge is represented by the array [node, neighbor, weight].
	FindEdge(f func(node, neighbor, weight int) bool) *[3]int
	// Returns the first edge incident to node that satisfies the predicate f. Returns nil if no edges satisfy. The returned edge is represented by the array [node, neighbor, weight].
	FindIncidentEdge(node int, f func(node, neighbor, weight int) bool) *[3]int
	// Returns the number of nodes in the graph.
	Len() int
	// Returns the reverse graph (i.e., a graph such that the direction of each edge is reversed).
	Reverse() Graph
	// Returns a string representation of the graph.
	String() string
	// Returns the weight of the edge between nodes u and v and true if the edge exists. Returns 0 and false if the edge does not exist.
	EdgeWeight(u, v int) (int, bool)
	// Returns true if nodes u and v are within the length of the graph's underlying data structure. Returns false otherwise.
	ValidEdge(u, v int) bool
}

// If g is a DAG, returns (topologicalOrdering, nil)
// otherwise, returns (nil, nodesInCycles)
func TopologicalOrdering(g Graph) (*[]int, *[]int) {
	ordering := make([]int, 0, g.Len())
	// track number of incoming edges from active nodes for each node
	// incomingActiveEdges[u] gives # of edges incoming from active nodes for node u
	incomingActiveEdges := make([]int, g.Len())
	// set of active nodes with no incoming edges
	noIncomingEdges := gods.NewSet[int]();

	// initialize incomingActiveEdges O(m)
	g.ForEachEdge(func(node, neighbor, weight int) {
		incomingActiveEdges[neighbor]++
	})

	// initialize noIncomingEdges, O(n)
	for node, incomingEdges := range incomingActiveEdges {
		if incomingEdges == 0 {
			noIncomingEdges.Add(node)
		}
	}

	// This part visits each directed edge exactly once, O(m)
	// Reason: Each node will be inserted into noIncomingEdges once; for each of these nodes, iterate through neighbors
	// while there are nodes in noIncomingEdges
	for noIncomingEdges.Len() > 0 {
		// select a node u from noIncomingEdges
		noIncomingEdges.ForEach(func(element int) {
			g.ForEachIncidentEdge(element, func(node, neighbor, weight int) {
				// decrement count in incomingActiveEdges for each node v to which u has an outgoing edge 
				incomingActiveEdges[neighbor]--
				// add any new nodes with incomingActiveEdges == 0 to noIncomingEdges
				if incomingActiveEdges[neighbor] == 0 {
					noIncomingEdges.Add(neighbor)
				}
			})
			// delete u from noIncomingEdges
			noIncomingEdges.Remove(element)
			// append u to ordering
			ordering = append(ordering, element)
		})
	}

	// All nodes that still have incoming edges are part of a cycle
	cycle := gods.NewStack[int]()
	for node, incomingEdges := range incomingActiveEdges {
		if incomingEdges > 0 {
			cycle.Push(node)
		}
	}

	// If there are some nodes part of a cycle, not a DAG
	if cycle.Len() > 0 {
		return nil, cycle.Elements()
	}
	// Otherwise, we have a DAG, return the ordering
	return &ordering, nil
}

// Returns the BFS Tree Edge List generated from BFS starting at node 0
func BFS(g Graph) *[][2]int {
	return BFSAt(g, 0)
}

// Returns the BFS Tree Edge List generated from BFS starting at startnode
func BFSAt(g Graph, startNode int) *[][2]int {
	edgeList := BFSReduce(g, startNode, func(node, neighbor, weight int, accumulator [][2]int) [][2]int {
		return append(accumulator, [2]int{node, neighbor})
	}, make([][2]int, 0))
	return &edgeList
}

// Calls f on every edge in the BFS Tree
func BFSDo(g Graph, startNode int, f func(node, neighbor, weight int)) {
	visited := make([]bool, g.Len())
	queue := gods.NewQueue[int]()
	visited[startNode] = true
	queue.Enqueue(startNode)
	for queue.Len() != 0 {
		node := queue.Dequeue()
		g.ForEachIncidentEdge(node, func(node, neighbor, weight int) {
			if !visited[neighbor] {
				queue.Enqueue(neighbor)
				visited[neighbor] = true
				f(node, neighbor, weight)
			}
		})
	}
}

func BFSReduce[T any](g Graph, startNode int, f func(node, neighbor, weight int, accumulator T) T, initialValue T) T {
	retVal := initialValue
	visited := make([]bool, g.Len())
	queue := gods.NewQueue[int]()
	queue.Enqueue(startNode)
	visited[startNode] = true;
	for queue.Len() > 0 {
		node := queue.Dequeue()
		g.ForEachIncidentEdge(node, func(node, neighbor, weight int) {
			if !visited[neighbor] {
				queue.Enqueue(neighbor)
				visited[neighbor] = true
				retVal = f(node, neighbor, weight, retVal)
			}
		})
	}
	return retVal
}

func DFS(g Graph) *[][2]int {
	return DFSAt(g, 0)
}

func DFSAt(g Graph, startNode int) *[][2]int {
	edgeList := DFSReduce(g, startNode, func(node, neighbor, weight int, accumulator [][2]int) [][2]int {
		return append(accumulator, [2]int{node, neighbor})
	}, make([][2]int, 0))
	return &edgeList
}

func DFSDo(g Graph, startNode int, f func(node, neighbor, weight int)) {
	explored := make([]bool, g.Len())
	// parents[u][0] gives parent node of u
	// parents[u][1] gives weight of edge
	parents := make([][2]int, g.Len())
	stack := gods.NewStack[int]()
	stack.Push(startNode)
	for stack.Len() > 0 {
		node := stack.Pop()
		if !explored[node] {
			explored[node] = true;
			if node != startNode {
				// DFS tree edge from parents[node] to node
				f(parents[node][0], node, parents[node][1])
			}
			g.ForEachIncidentEdge(node, func(node, neighbor, weight int) {
				parents[neighbor] = [2]int{node, weight}
				stack.Push(neighbor)
			})
		}
	}
}

func DFSReduce[T any](g Graph, startNode int, f func(node, neighbor, weight int, accumulator T) T, initialValue T) T {
	// parents[u][0] gives parent node of u
	// parents[u][1] gives weight of edge
	retVal := initialValue
	parents := make([][2]int, g.Len())
	explored := make([]bool, g.Len())
	stack := gods.NewStack[int]()
	stack.Push(startNode)
	for stack.Len() > 0 {
		node := stack.Pop()
		if !explored[node] {
			explored[node] = true;
			if node != startNode {
				// edge from parents[node] to node
				retVal = f(parents[node][0], node, parents[node][1], retVal)
			}
			g.ForEachIncidentEdge(node, func(node, neighbor, weight int) {
				parents[neighbor] = [2]int{node, weight}
				stack.Push(neighbor)
			})
		}
	}
	return retVal
}

func DFSRecursive(g Graph) *[][2]int {
	return DFSRecursiveAt(g, 0)
}

func DFSRecursiveAt(g Graph, startNode int) *[][2]int {
	edgeList := DFSRecursiveReduce(g, startNode, func(node, neighbor, weight int, accumulator [][2]int) [][2]int {
		return append(accumulator, [2]int{node, neighbor})
	}, make([][2]int, 0))
	return &edgeList
}

func DFSRecursiveDo(g Graph, startNode int, f func(node, neighbor, weight int)) {
	explored := make([]bool, g.Len())
	dfsRecursiveDoHelper(g, startNode, f, &explored)
}

func dfsRecursiveDoHelper(g Graph, node int, f func (node, neighbor, weight int), explored *[]bool) {
	(*explored)[node] = true;
	g.ForEachIncidentEdge(node, func(node, neighbor, weight int) {
		if !(*explored)[neighbor] {
			f(node, neighbor, weight)
			dfsRecursiveDoHelper(g, neighbor, f, explored)
		}
	})
}

func DFSRecursiveReduce[T any](g Graph, startNode int, f func(node, neighbor, weight int, accumulator T) T, initialValue T) T {
	explored := make([]bool, g.Len())
	retVal := initialValue
	dfsRecursiveReduceHelper(g, startNode, f, &retVal, &explored)
	return retVal
}

func dfsRecursiveReduceHelper[T any](g Graph, node int, f func(node, neighbor, weight int, accumulator T) T, retVal *T, explored *[]bool) {
	(*explored)[node] = true
	g.ForEachIncidentEdge(node, func(node, neighbor, weight int) {
		if !(*explored)[neighbor] {
			*retVal = f(node, neighbor, weight, *retVal)
			dfsRecursiveReduceHelper(g, neighbor, f, retVal, explored)
		}
	})
}

func IsBipartite(g Graph) bool {
	return IsBipartiteAt(g, 0)
}

func IsBipartiteAt(g Graph, startNode int) bool {
	const (
		_ int = iota
		Red
		Blue
	)

	colors := make([]int, g.Len())
	colors[startNode] = Blue
	BFSDo(g, startNode, func(node, neighbor, weight int) {
		if colors[node] == Blue {
			colors[neighbor] = Red;
		} else {
			colors[neighbor] = Blue
		}
	})

	return g.EveryEdge(func(node, neighbor, weight int) bool {
		return colors[node] != colors[neighbor]
	})
}

func componentFromBFSTreeEdges(bfsTreeEdges *[][2]int) *gods.Set[int] {
	component := gods.NewSet[int]()
	for _, edge := range *bfsTreeEdges {
		for _, node := range edge {
			component.Add(node)
		}
	}
	return component;
}

func IsStronglyConnected(g Graph) bool {
	gRev := g.Reverse();
	gRevComponent := componentFromBFSTreeEdges(BFS(g))
	gBFSTreeEdges := BFS(gRev);

	// build the connected component of G
	gComponent := gods.NewSet[int]()
	for _, gEdge := range *gBFSTreeEdges {
		for _, gNode := range gEdge {
			// gNode must be in the connected component of gRev if g is strongly connected
			if !gRevComponent.Contains(gNode) {
				return false
			}
			gComponent.Add(gNode)
		}
	}

	// strongly connected component if connected component of g equals connected component of gRev
	// at this point, connected components are equal if they contain same number of nodes
	return gComponent.Len() == gRevComponent.Len()
}

func UndirectedHasCycle(g Graph) bool {
	return UndirectedHasCycleAt(g, 0)
}

func UndirectedHasCycleAt(g Graph, startNode int) bool {
	// Graph has cycle if DFS ever visits the same node via two different paths
	// i.e., the same node is pushed onto the stack twice
	explored := make([]bool, g.Len())
	parents := make([]int, g.Len())
	stack := gods.NewStack[int]()
	stack.Push(startNode)
	for stack.Len() > 0 {
		node := stack.Pop()
		// stack contains no parent nodes
		// thus, if already explored node, it was placed on stack by another path
		// then, there are two distinct paths to node and thus a cycle
		if explored[node] {
			return true;
		}
		explored[node] = true;
		// push all non-parent nodes to stack
		g.ForEachIncidentEdge(node, func(node, neighbor, weight int) {
			if neighbor != parents[node] {
				parents[neighbor] = node;
				stack.Push(neighbor)
			}
		})
	}
	// No cycles found, return false
	return false
}

// Returns a cycle if the given undirected graph has one, returns nil otherwise.
func UndirectedGetCycle(g Graph) *[]int {
	return UndirectedGetCycleAt(g, 0)
}

func UndirectedGetCycleAt(g Graph, startNode int) *[]int {
	bfsTree := NewAdjList(g.Len())
	// build the BFS tree
	BFSDo(g, startNode, func(node, neighbor, weight int) {
		bfsTree.AddBothEdge(node, neighbor, weight)
	})

	// determine any edge in g that is not in the bfsTree
	edge := g.FindEdge(func(node, neighbor, weight int) bool {
		return !bfsTree.HasEdge(node, neighbor)
	})
	// if such an edge exists, there is a cycle
	if edge != nil {
		// let u and v be the ends of such an edge
		// simple cycle given by the least-edges-path from u to v
		return LeastEdgesPath(g, edge[0], edge[1]).Elements()
	}

	// at this point, graph has no cycles
	return nil
}

// 
func LeastEdgesPath(g Graph, startNode, endNode int) *gods.Stack[int] {
	// BFS, but keep track of parents in the BFS tree
	if !g.ValidEdge(startNode, endNode) {
		panic(fmt.Sprintf("Edge %v-%v out of range", startNode, endNode))
	}
	parents := make([]int, g.Len())
	visited := make([]bool, g.Len())
	queue := gods.NewQueue[int]()
	queue.Enqueue(startNode)
	visited[startNode] = true
	for queue.Len() > 0 {
		node := queue.Dequeue()
		foundEndNode := g.SomeIncidentEdge(node, func(node, neighbor, weight int) bool {
			if !visited[neighbor] {
				parents[neighbor] = node
				queue.Enqueue(neighbor)
				return neighbor == endNode
			}
			return false
		})
		if foundEndNode {
			return pathFromParents(startNode, endNode, parents)
		}
	}
	return nil
}

func pathFromParents(startNode, endNode int, parents []int) *gods.Stack[int] {
	path := gods.NewStack[int]()
	// start from end node
	node := endNode
	for node != startNode {
		// prepend node to path
		path.Push(node)
		// update node to current parent
		node = parents[node]
	}
	// finally prepend startNode
	path.Push(startNode)
	return path
}

// Returns distance of shortest paths from startNode to each node in graph
// distances[node] = costOfShortestPath
func DijkstraShortestPathCosts(g Graph, startNode int) *[]int {
	distances := make([]int, g.Len())
	visited := make([]bool, g.Len())
	heap := gods.NewBinaryHeap[int](g.Len())
	heap.Insert(startNode, 0)
	visited[startNode] = true
	for heap.Len() > 0 {
		closestNode := heap.ExtractMin()
		// store cost of shortest path to node
		distances[closestNode.Value] = closestNode.Key
		g.ForEachIncidentEdge(closestNode.Value, func(node, neighbor, weight int) {
			distNeighbor, isInQueue := heap.KeyOf(neighbor)
			currPathWeight := closestNode.Key + weight
			if isInQueue {
				// if current path to neighbor costs less than previously discovered path
				if distNeighbor > currPathWeight {
					// update previous path cost with current path cost
					heap.ChangeKey(neighbor, currPathWeight)
				}
			} else if !visited[neighbor] {
				// only add each node to the priority queue once
				visited[neighbor] = true
				heap.Insert(neighbor, currPathWeight)
			}
		})
	}
	return &distances
}

// return the path from startNode to endNode for which cost is minimized
func DijkstraShortestPath(g Graph, startNode, endNode int) *[]int {
	if !g.ValidEdge(startNode, endNode) {
		panic(fmt.Sprintf("Edge %v-%v out of range", startNode, endNode))
	}
	// a parent of some node u is the node to which u connects in the shortest path to u
	parents := make([]int, g.Len())
	visited := make([]bool, g.Len())
	heap := gods.NewBinaryHeap[int](g.Len())
	heap.Insert(startNode, 0)
	visited[startNode] = true
	for heap.Len() > 0 {
		closestNode := heap.ExtractMin()
		if closestNode.Value == endNode {
			// endNode reached, return the path from startNode to endNode
			return pathFromParents(startNode, endNode, parents).Elements()
		}
		g.ForEachIncidentEdge(closestNode.Value, func(node, neighbor, weight int) {
			distNeighbor, isInQueue := heap.KeyOf(neighbor)
			currPathWeight := closestNode.Key + weight
			if isInQueue {
				if distNeighbor > currPathWeight {
					heap.ChangeKey(neighbor, currPathWeight)
					// update parent, since now the shortest path goes through closestNode
					parents[neighbor] = closestNode.Value
				}
			} else if !visited[neighbor] {
				visited[neighbor] = true
				parents[neighbor] = closestNode.Value
				heap.Insert(neighbor, currPathWeight)
			}
		})
	}

	// This occurs if there was no path from startNode to endNode
	return nil
}
// returns the edges of the minimum spanning tree and the total cost, using Prim's Algorithm
func PrimMST(g Graph) (*[][2]int, int) {
	return PrimMSTAt(g, 0)
}

func PrimMSTAt(g Graph, startNode int) (*[][2]int, int) {
	mstEdges := gods.NewQueue[[2]int]()
	totalCost := 0
	parents := make([]int, g.Len())
	visited := make([]bool, g.Len())
	heap := gods.NewBinaryHeap[int](g.Len())
	heap.Insert(startNode, 0)
	visited[startNode] = true
	for heap.Len() > 0 {
		cheapest := heap.ExtractMin()
		if cheapest.Value != startNode {
			totalCost += cheapest.Key
			mstEdges.Enqueue([2]int{parents[cheapest.Value], cheapest.Value})
		}
		g.ForEachIncidentEdge(cheapest.Value, func(node, neighbor, weight int) {
			attachmentCost, isInHeap := heap.KeyOf(neighbor)
			if isInHeap {
				if attachmentCost > weight {
					parents[neighbor] = node
					heap.ChangeKey(neighbor, weight)
				}
			} else if !visited[neighbor] {
				visited[neighbor] = true
				parents[neighbor] = node
				heap.Insert(neighbor, weight)
			}
		})
	}
	return mstEdges.Elements(), totalCost
}

// returns the edges of the minimum spanning tree and the total cost, using Kruskal's Algorithm
func KruskalMST(g Graph) (*[][2]int, int) {
	mstEdges := gods.NewQueue[[2]int]()
	totalCost := 0
	edges := *g.Edges()
	uf := gods.NewUnionFind(g.Len())
	sort.SliceStable(edges, func(i, j int) bool {
		return edges[i][2] < edges[j][2]
	})
	for _, edge := range edges {
		node := edge[0]
		neighbor := edge[1]
		if uf.Find(node) != uf.Find(neighbor) {
			totalCost += edge[2]
			mstEdges.Enqueue([2]int{node, neighbor})
			uf.Union(node, neighbor)
		}
	}
	return mstEdges.Elements(), totalCost
}

// returns edges in the k-clustering of maximum possible spacing
func KClustering(g Graph, k int) *[][2]int {
	// Kruskal's gives us MST edges sorted by weight
	sortedEdges, _ := KruskalMST(g)
	// k-clustering is MST edges minus the (k-1) most expensive edges.
	kCluster := (*sortedEdges)[:len(*sortedEdges)-k+1]
	return &kCluster
}

/*
If no negative cycles, returns 
path from startNode to endNode,
cost of the shortest path from every node to endNode,
false.
If graph has negative cycle, returns
nodes leading up to negative cycle,
nil,
true.
 */
func BellmanFordShortestPath(g Graph, startNode, endNode int) (*[]int, *[]int, bool) {
	numNodes := g.Len()
	shortestPathCost := make([]int, numNodes)
	// for constructing the pointer graph
	successor := make([]int, numNodes)

	// set value of shortestPathCost for each node to infinity
	for i := range shortestPathCost {
		shortestPathCost[i] = math.MaxInt
	}
	// cost from endNode to itself is zero
	shortestPathCost[endNode] = 0

	// compute shortestPathCost for each node
	for i := 1; i < numNodes; i++ {
		// From the book:
		// If we ever execute a complete iteration i in which no M[v] value changes,
		// then no M[v] value will ever change again
		// since future iterations will begin with exactly the same set of array entries
		noChange := true
		g.ForEachEdge(func(node, neighbor, weight int) {
			neighborPathCost := shortestPathCost[neighbor]
			if neighborPathCost != math.MaxInt {
				alternativePathCost := weight + neighborPathCost
				if shortestPathCost[node] > alternativePathCost {
					successor[node] = neighbor
					shortestPathCost[node] = alternativePathCost
					noChange = false
				}
			}
		})
		if noChange {
			break;
		}
	}

	// compute path from startNode to endNode using successor graph
	path := gods.NewQueue[int]()
	visited := make([]bool, numNodes)
	for tmp := startNode; tmp != endNode; tmp = successor[tmp] {
		if !visited[tmp] {
			visited[tmp] = true
			path.Enqueue(tmp)
		} else {
			// we have a cycle (and by 6.27, a negative cycle)
			path.Enqueue(tmp)
			return path.Elements(), nil, true
		}
	}
	path.Enqueue(endNode)

	return path.Elements(), &shortestPathCost, false
}