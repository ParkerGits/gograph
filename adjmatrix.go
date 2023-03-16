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
	edges [][]int
}

func NewAdjMat(numNodes int) *AdjMatrix {
	ret := &AdjMatrix{edges: make([][]int, numNodes)}
	for node := range ret.edges {
		ret.edges[node] = make([]int, numNodes)
	}
	return ret;
}

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

func (g *AdjMatrix) AddBothEdgeFromString(edge string) {
	u, v, c := parseEdgeStr(edge)
	g.AddBothEdge(u, v, c)
}

func (g *AdjMatrix) AddDirectedEdgeFromString(edge string) {
	u, v, c := parseEdgeStr(edge)
	g.AddDirectedEdge(u, v, c)
}

func (g *AdjMatrix) AddBothEdge(u, v, c int) {
	if !g.ValidEdge(u, v) {
		panic(fmt.Sprintf("Edge %v-%v out of range", u, v))
	}
	g.edges[u][v] = c
	if u != v {
		g.edges[v][u] = c
	}
}

func (g *AdjMatrix) AddDirectedEdge(u, v, c int) {
	if !g.ValidEdge(u, v) {
		panic(fmt.Sprintf("Edge %v-%v out of range", u, v))
	}
	g.edges[u][v] = c
}

func (g *AdjMatrix) ValidEdge(u, v int) bool {
	return (u >= 0 || u < len(g.edges)) || (v >= 0 || v < len(g.edges))
}

func (g *AdjMatrix) String() string {
	buf := bytes.NewBufferString("")
	for u, neighbors := range g.edges {
		buf.WriteString(fmt.Sprintf("%d: %v\n", u, neighbors))
	}
	return buf.String()
}

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

func (g *AdjMatrix) Len() int {
	return len(g.edges)
}

func (g *AdjMatrix) ForEachEdge(f func(node,neighbor,weight int)) {
	for node, neighbors := range g.edges {
		for neighbor, weight := range neighbors {
			if weight != 0 {
				f(node, neighbor, weight)
			}
		}
	}
}

func (g *AdjMatrix) ForEachIncidentEdge(node int, f func(node,neighbor,weight int)) {
		for neighbor, weight := range g.edges[node] {
			if weight != 0 {
				f(node, neighbor, weight)
			}
		}
}

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

func (g *AdjMatrix) SomeIncidentEdge(node int, f func(node,neighbor,weight int) bool) bool {
	for neighbor, weight := range g.edges[node] {
		if weight != 0 && f(node, neighbor, weight) {
			return true
		}
	}
	return false
}

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

func (g *AdjMatrix) FindIncidentEdge(node int, f func(node, neighbor, weight int) bool) *[3]int {
	for neighbor, weight := range g.edges[node] {
		if weight != 0 && f(node, neighbor, weight) {
			return &[3]int{node, neighbor, weight}
		}
	}
	return nil
}

func (g *AdjMatrix) EdgeWeight(u, v int) (int, bool) {
	weight := g.edges[u][v]
	if weight == 0 {
		return 0, false
	}
	return weight, true
}

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