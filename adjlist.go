package gograph

import (
	"bytes"
	"fmt"
	"os"
	"strconv"
	"strings"
	"github.com/ParkerGits/gods"
)

type AdjList struct {
	// for a node u, node v, and cost c
	// edges[u] gives map with v as keys and c as values
	edges []map[int]int
}

// Constructs a new adjacency list of given size. Available nodes are 0 <= n < numNodes. 
func NewAdjList(size int) *AdjList {
	return &AdjList{edges: make([]map[int]int, size)}
}

// Constructs a new undirected graph in adjacency list format from given file.
// Expected file format is a list of edges as follows:
// [numNodes]
// [u] [v] [c1]
// [i] [j] [c2]
// ...
func UndirectedAdjListFromFile(filename string) *AdjList {
	dat, err := os.ReadFile(filename)
	if err != nil {
		panic(err)
	}
	lines := strings.Split(string(dat), "\n")
	numNodes, err := strconv.ParseInt(string(lines[0][0]), 10, 0)
	if err != nil {
		panic(err)
	}
	graph := NewAdjList(int(numNodes))
	for _, line := range(lines[1:]) {
		graph.AddBothEdgeFromString(line)
	}
	return graph
}

// Constructs a new directed graph in adjacency list format from given file.
// Expected file format is a list of edges as follows:
// [numNodes]
// [u] [v] [c1]
// [i] [j] [c2]
// ...
func DirectedAdjListFromFile(filename string) *AdjList {
	dat, err := os.ReadFile(filename)
	if err != nil {
		panic(err)
	}
	lines := strings.Split(string(dat), "\n")
	numNodes, err := strconv.ParseInt(string(lines[0][0]), 10, 0)
	if err != nil {
		panic(err)
	}
	graph := NewAdjList(int(numNodes))
	for _, line := range(lines[1:]) {
		graph.AddDirectedEdgeFromString(line)
	}
	return graph
}

// Adds an undirected edge to the graph from string
// Expected string format is as follows:
// "u v c"
// where u, v, and c are integers representing node, neighbor, and cost respectively.
func (g *AdjList) AddBothEdgeFromString(edge string) {
	u, v, c := parseEdgeStr(edge)
	g.AddBothEdge(u, v, c)
}

// Adds a directed edge to the graph from string
// Expected string format is as follows:
// "u v c"
// where u, v, and c are integers representing node, neighbor, and cost respectively.
func (g *AdjList) AddDirectedEdgeFromString(edge string) {
	u, v, c := parseEdgeStr(edge)
	g.AddDirectedEdge(u, v, c)
}

func parseEdgeStr(edge string) (int, int, int) {
	edgeDatStr := strings.Split(edge, " ")
	if len(edgeDatStr) != 3 {
		panic("invalid edge string (expected 'u v w' where u, v, w are integers)")
	}

	u, err := strconv.ParseInt(edgeDatStr[0], 10, 0)
	if err != nil {
			panic(err)
	}
	v, err := strconv.ParseInt(edgeDatStr[1], 10, 0)
	if err != nil {
			panic(err)
	}
	c, err := strconv.ParseInt(edgeDatStr[2], 10, 0)
	if err != nil {
			panic(err)
	}

	return int(u), int(v), int(c)
}

// Edge is valid if both endpoints n satisfy 0 <= n < numNodes.
func (g *AdjList) ValidEdge(u, v int) bool {
	return (u >= 0 && u < len(g.edges)) && (v >= 0 || v < len(g.edges))
}

// Returns the number of nodes in the graph.
func (g *AdjList) Len() int {
	return len(g.edges)
}

// Adds the undirected edge u, v, c to the graph, where u, v, and c are node, neighbor, and weight respectively.
func (g *AdjList) AddBothEdge(u, v, c int) {
	if !g.ValidEdge(u, v) {
		panic(fmt.Sprintf("Edge %v-%v out of range", u, v))
	}
	if g.edges[u] == nil {
		g.edges[u] = make(map[int]int)
	}
	g.edges[u][v] = c

	if u != v {
		if g.edges[v] == nil {
			g.edges[v] = make(map[int]int)
		}
		g.edges[v][u] = c
	}
}

// Adds the directed edge u, v, c to the graph, where u, v, and c are node, neighbor, and weight respectively.
func (g *AdjList) AddDirectedEdge(u, v, c int) {
	if !g.ValidEdge(u, v) {
		panic(fmt.Sprintf("Edge %v-%v out of range", u, v))
	}
	if g.edges[u] == nil {
		g.edges[u] = make(map[int]int)
	}
	g.edges[u][v] = c
}

// Returns the graph in string format.
func (g *AdjList) String() string {
	buf := bytes.NewBufferString("")
	for u, neighbors := range g.edges {
		buf.WriteString(fmt.Sprintf("%d: %v\n", u, neighbors))
	}
	return buf.String()
}

// Returns the graph's reverse.
// A graph's reverse has the same nodes, but the direction of each edge is reversed.
func (g *AdjList) Reverse() Graph {
	rev := NewAdjList(len(g.edges))
	for u, neighbors := range g.edges {
		for v, c := range neighbors {
			// new edge from v to u
			rev.AddDirectedEdge(v, u, c)
		}
	}
	return Graph(rev)
}

// Iterates through all edges in the graph, calling f for each edge.
func (g *AdjList) ForEachEdge(f func(node,neighbor,weight int)) {
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			f(node, neighbor, weight)
		}
	}
}

// Iterates through all edges incident to node in the graph, calling f for each edge.
func (g *AdjList) ForEachIncidentEdge(node int, f func(node,neighbor,weight int)) {
	for neighbor, weight := range g.edges[node] {
		f(node, neighbor, weight)
	}
}

// Returns true if some edge in the graph satisfies the predicate f. Returns false otherwise.
func (g *AdjList) SomeEdge(f func(node,neighbor,weight int) bool) bool {
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			if f(node, neighbor, weight) {
				return true
			}
		}
	}
	return false
}

// Returns true if some edge incident to node in the graph satisfies the predicate f. Returns false otherwise.
func (g *AdjList) SomeIncidentEdge(node int, f func(node,neighbor,weight int) bool) bool {
	for neighbor, weight := range g.edges[node] {
		if f(node, neighbor, weight) {
			return true
		}
	}
	return false
}

// Returns true if every edge in the graph satisfies the predicate f. Returns false otherwise.
func (g *AdjList) EveryEdge(f func(node,neighbor,weight int) bool) bool {
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			if !f(node, neighbor, weight) {
				return false
			}
		}
	}
	return true
}

// Returns true if the graph has an edge from node to neighbor.
func (g *AdjList) HasEdge(node, neighbor int) bool {
	_, hasEdge := g.edges[node][neighbor]
	return hasEdge
}

// Returns the first edge in the graph that satisfies the predicate f. Returns nil if no edges satisfy. The returned edge is represented by the array [node, neighbor, weight].
func (g *AdjList) FindEdge(f func(node,neighbor,weight int) bool) *[3]int {
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			if f(node, neighbor, weight) {
				return &[3]int{node, neighbor, weight}
			}
		}
	}
	return nil
}

// Returns the first edge incident to node that satisfies the predicate f. Returns nil if no edges satisfy. The returned edge is represented by the array [node, neighbor, weight].
func (g *AdjList) FindIncidentEdge(node int, f func(node, neighbor, weight int) bool) *[3]int {
	for neighbor, weight := range g.edges[node] {
		if f(node, neighbor, weight) {
			return &[3]int{node, neighbor, weight}
		}
	}
	return nil
}

// Returns the weight of the edge from u to v.
func (g *AdjList) EdgeWeight(u, v int) (int, bool) {
	weight, ok := g.edges[u][v]
	return weight, ok
}

// Returns a slice containing every edge in the graph. Each edge represented by the array [node, neighbor, weight].
func (g *AdjList) Edges() *[][3]int {
	edges := gods.NewQueue[[3]int]()
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			edges.Enqueue([3]int{node, neighbor, weight})
		}
	}
	return edges.Elements()
}