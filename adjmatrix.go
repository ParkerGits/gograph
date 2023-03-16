package gograph

import (
	"bytes"
	"fmt"
	"os"
	"strconv"
	"strings"
	"github.com/ParkerGits/gods"
)

type AdjMatrix struct {
	// for a node u, node v, and cost c
	// edges[u] gives an array of neighbors. v is a neighbor iff c != 0.
	edges [][]int
}

// Constructs a new adjacency matrix of given size. Available nodes are 0 <= n < numNodes. 
func NewAdjMat(numNodes int) *AdjMatrix {
	ret := &AdjMatrix{edges: make([][]int, numNodes)}
	for node := range ret.edges {
		ret.edges[node] = make([]int, numNodes)
	}
	return ret;
}

// Constructs a new undirected graph in adjacency matrix format from given file.
// Expected file format is a list of edges as follows:
// [numNodes]
// [u] [v] [c1]
// [i] [j] [c2]
// ...
func UndirectedAdjMatFromFile(filename string) *AdjMatrix {
	dat, err := os.ReadFile(filename)
	if err != nil {
		panic(err)
	}
	lines := strings.Split(string(dat), "\n")
	numNodes, err := strconv.ParseInt(string(lines[0][0]), 10, 0)
	if err != nil {
		panic(err)
	}
	g := NewAdjMat(int(numNodes))
	for _, line := range(lines[1:]) {
		g.AddBothEdgeFromString(line)
	}
	return g
}

// Constructs a new directed graph in adjacency matrix format from given file.
// Expected file format is a list of edges as follows:
// [numNodes]
// [u] [v] [c1]
// [i] [j] [c2]
// ...
func DirectedAdjMatFromFile(filename string) *AdjMatrix {
	dat, err := os.ReadFile(filename)
	if err != nil {
		panic(err)
	}
	lines := strings.Split(string(dat), "\n")
	numNodes, err := strconv.ParseInt(string(lines[0][0]), 10, 0)
	if err != nil {
		panic(err)
	}
	g := NewAdjMat(int(numNodes))
	for _, line := range(lines[1:]) {
		g.AddDirectedEdgeFromString(line)
	}
	return g
}

// Adds an undirected edge to the graph from string
// Expected string format is as follows:
// "u v c"
// where u, v, and c are integers representing node, neighbor, and cost respectively.
func (g *AdjMatrix) AddBothEdgeFromString(edge string) {
	u, v, c := parseEdgeStr(edge)
	g.AddBothEdge(u, v, c)
}

// Adds a directed edge to the graph from string
// Expected string format is as follows:
// "u v c"
// where u, v, and c are integers representing node, neighbor, and cost respectively.
func (g *AdjMatrix) AddDirectedEdgeFromString(edge string) {
	u, v, c := parseEdgeStr(edge)
	g.AddDirectedEdge(u, v, c)
}

// Adds the undirected edge u, v, c to the graph, where u, v, and c are node, neighbor, and weight respectively.
func (g *AdjMatrix) AddBothEdge(u, v, c int) {
	if !g.ValidEdge(u, v) {
		panic(fmt.Sprintf("Edge %v-%v out of range", u, v))
	}
	g.edges[u][v] = c
	if u != v {
		g.edges[v][u] = c
	}
}

// Adds the directed edge u, v, c to the graph, where u, v, and c are node, neighbor, and weight respectively.
func (g *AdjMatrix) AddDirectedEdge(u, v, c int) {
	if !g.ValidEdge(u, v) {
		panic(fmt.Sprintf("Edge %v-%v out of range", u, v))
	}
	g.edges[u][v] = c
}

// Edge is valid if both endpoints n satisfy 0 <= n < numNodes.
func (g *AdjMatrix) ValidEdge(u, v int) bool {
	return (u >= 0 || u < len(g.edges)) || (v >= 0 || v < len(g.edges))
}

// Returns the graph in string format.
func (g *AdjMatrix) String() string {
	buf := bytes.NewBufferString("")
	for u, neighbors := range g.edges {
		buf.WriteString(fmt.Sprintf("%d: %v\n", u, neighbors))
	}
	return buf.String()
}

// Returns the graph's reverse.
// A graph's reverse has the same nodes, but the direction of each edge is reversed.
func (g *AdjMatrix) Reverse() Graph {
	rev := NewAdjMat(len(g.edges))
	for u, neighbors := range g.edges {
		for v, c := range neighbors {
			// new edge from v to u
			rev.AddDirectedEdge(v, u, c)
		}
	}
	return Graph(rev)
}

// Returns the number of nodes in the graph.
func (g *AdjMatrix) Len() int {
	return len(g.edges)
}

// Iterates through all edges in the graph, calling f for each edge.
func (g *AdjMatrix) ForEachEdge(f func(node,neighbor,weight int)) {
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			if weight != 0 {
				f(node, neighbor, weight)
			}
		}
	}
}

// Iterates through all edges incident to node in the graph, calling f for each edge.
func (g *AdjMatrix) ForEachIncidentEdge(node int, f func(node,neighbor,weight int)) {
		for neighbor, weight := range g.edges[node] {
			if weight != 0 {
				f(node, neighbor, weight)
			}
		}
}

// Returns true if some edge in the graph satisfies the predicate f. Returns false otherwise.
func (g *AdjMatrix) SomeEdge(f func(node,neighbor,weight int) bool) bool {
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			if weight != 0 && f(node, neighbor, weight) {
				return true
			}
		}
	}
	return false
}

// Returns true if some edge incident to node in the graph satisfies the predicate f. Returns false otherwise.
func (g *AdjMatrix) SomeIncidentEdge(node int, f func(node,neighbor,weight int) bool) bool {
	for neighbor, weight := range g.edges[node] {
		if weight != 0 && f(node, neighbor, weight) {
			return true
		}
	}
	return false
}

// Returns true if every edge in the graph satisfies the predicate f. Returns false otherwise.
func (g *AdjMatrix) EveryEdge(f func(node,neighbor,weight int) bool) bool {
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			if weight != 0 && !f(node, neighbor, weight) {
				return false
			}
		}
	}
	return true
}

// Returns the first edge in the graph that satisfies the predicate f. Returns nil if no edges satisfy. The returned edge is represented by the array [node, neighbor, weight].
func (g *AdjMatrix) FindEdge(f func(node,neighbor,weight int) bool) *[3]int {
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			if weight != 0 && f(node, neighbor, weight) {
				return &[3]int{node, neighbor, weight}
			}
		}
	}
	return nil
}

// Returns the first edge incident to node that satisfies the predicate f. Returns nil if no edges satisfy. The returned edge is represented by the array [node, neighbor, weight].
func (g *AdjMatrix) FindIncidentEdge(node int, f func(node, neighbor, weight int) bool) *[3]int {
	for neighbor, weight := range g.edges[node] {
		if weight != 0 && f(node, neighbor, weight) {
			return &[3]int{node, neighbor, weight}
		}
	}
	return nil
}

// Returns the weight of the edge from u to v.
func (g *AdjMatrix) EdgeWeight(u, v int) (int, bool) {
	weight := g.edges[u][v]
	if weight == 0 {
		return 0, false
	}
	return weight, true
}

// Returns a slice containing every edge in the graph. Each edge represented by the array [node, neighbor, weight].
func (g *AdjMatrix) Edges() *[][3]int {
	edges := gods.NewQueue[[3]int]()
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			if weight != 0 {
				edges.Enqueue( [3]int{node, neighbor, weight})
			}
		}
	}
	return edges.Elements()
}