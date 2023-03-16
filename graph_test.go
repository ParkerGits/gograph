package gograph_test

import (
	"github.com/ParkerGits/gods"
	"github.com/ParkerGits/gograph"
	"reflect"
	"sort"
	"testing"
)

func TestTopologicalOrdering(t *testing.T) {
	// Figure 3.7 in book
	expected := []int{0,1,2,3,4,5,6}
	dag1List := gograph.DirectedAdjListFromFile("test/dag1.graph")
	dag1ListOrdering, dag1ListCycles := gograph.TopologicalOrdering(dag1List)
	// First element should be 0 or 1
	if !((*dag1ListOrdering)[0] == 0 || (*dag1ListOrdering)[0] == 1) {
		t.Errorf("Expected ordering to start with 0 or 1. Actual: %v", (*dag1ListOrdering)[0])
	}
	// Last element must be 6
	if (*dag1ListOrdering)[len(*dag1ListOrdering)-1] != 6 {
		t.Errorf("Expected ordering to start with zero or 1. Actual: %v", (*dag1ListOrdering)[len(*dag1ListOrdering) - 1])
	}
	sort.Ints(*dag1ListOrdering)
	// Ordering must contain all nodes
	if !reflect.DeepEqual(*dag1ListOrdering, expected) {
		t.Errorf("Expected ordering to be [0,1,2,3,4,5,6]. Actual: %v", *dag1ListOrdering)
	}
	if dag1ListCycles != nil {
		t.Errorf("Expected no cycles. Actual: %v", *dag1ListCycles)
	}

	dag1Mat := gograph.DirectedAdjMatFromFile("test/dag1.graph")
	dag1MatOrdering, dag1MatCycles := gograph.TopologicalOrdering(dag1Mat)
	// First element should be 0 or 1
	if !((*dag1MatOrdering)[0] == 0 || (*dag1MatOrdering)[0] == 1) {
		t.Errorf("Expected ordering to start with 0 or 1. Actual: %v", (*dag1MatOrdering)[0])
	}
	// Last element must be 6
	if (*dag1MatOrdering)[len(*dag1MatOrdering)-1] != 6 {
		t.Errorf("Expected ordering to start with zero or 1. Actual: %v", (*dag1MatOrdering)[len(*dag1MatOrdering) - 1])
	}
	sort.Ints(*dag1MatOrdering)
	// Ordering must contain all nodes
	if !reflect.DeepEqual(*dag1MatOrdering, expected) {
		t.Errorf("Expected ordering to be [0,1,2,3,4,5,6]. Actual: %v", *dag1MatOrdering)
	}
	if dag1MatCycles != nil {
		t.Errorf("Expected DAG to have no cycles. Actual: %v", *dag1ListCycles)
	}

	// Figure 3.1 in book
	// Tree should have no cycles
	treeList := gograph.DirectedAdjListFromFile("test/tree.graph")
	_, treeCycles := gograph.TopologicalOrdering(treeList) 
	if treeCycles != nil {
		t.Errorf("Expected tree to have no cycles. Actual: %v", *treeCycles)
	}

	// Four-node graph with three-node cycle
	expectedCycle := []int{1,2,3}
	directedCycleList := gograph.DirectedAdjListFromFile("test/directedcycle.graph")
	directedCycleOrdering, directedCycle := gograph.TopologicalOrdering(directedCycleList)
	if directedCycleOrdering != nil {
		t.Errorf("Expected directed cycle to have no ordering. Actual: %v", *directedCycleOrdering)
	}
	if directedCycle == nil {
		t.Errorf("Expected to find directed cycle.")
	} else {
		sort.Ints(*directedCycle)
		if !reflect.DeepEqual(*directedCycle, expectedCycle) {
			t.Errorf("Expected to find cycle 1,2,3. Found: %v", *directedCycle)
		}
	}
}

func TestSearch(t *testing.T) {
	// Connected component in figure 3.2
	expected := [][2]int{{0,1}, {0,2}, {1,3}, {1,4}, {2,6}, {2,7}, {4,5}}

	test1List := gograph.UndirectedAdjListFromFile("test/test1.graph")
	test1ListBFS := gograph.BFS(test1List)
	sort.SliceStable(*test1ListBFS, func(i, j int) bool {
		if (*test1ListBFS)[i][0] == (*test1ListBFS)[j][0] {
			return (*test1ListBFS)[i][1] < (*test1ListBFS)[j][1]
		}
		return (*test1ListBFS)[i][0] < (*test1ListBFS)[j][0]
	})
	if !reflect.DeepEqual(*test1ListBFS, expected) {
		t.Errorf("Expected: %v, Actual: %v", expected, *test1ListBFS)
	}

	test1Mat := gograph.UndirectedAdjMatFromFile("test/test1.graph")
	test1MatBFS := gograph.BFS(test1Mat)
	sort.SliceStable(*test1MatBFS, func(i, j int) bool {
		return (*test1ListBFS)[i][0] < (*test1ListBFS)[j][0]
	})
	if !reflect.DeepEqual(*test1MatBFS, expected) {
		t.Errorf("Expected: %v, Actual: %v", expected, *test1MatBFS)
	}

	// Indeterminate starting edge for DFS
	// Just ensure that every node is reached once
	test1ListDFS := gograph.DFS(test1List)
	nodes := gods.NewSet[int]().From([]int{1, 2, 3, 4, 5, 6, 7})
	for _, edge := range *test1ListDFS {
		nodes.Remove(edge[1])
	}
	if nodes.Len() > 0 {
		t.Errorf("DFS failed to reach all nodes. Nodes remaining: %v", nodes.Elements())
	}

	test1ListDFSRecursive := gograph.DFSRecursive(test1List)
	nodesRecursive := gods.NewSet[int]().From([]int{1, 2, 3, 4, 5, 6, 7})
	for _, edge := range *test1ListDFSRecursive {
		nodesRecursive.Remove(edge[1])
	}
	if nodesRecursive.Len() > 0 {
		t.Errorf("DFS-Recursive failed to reach all nodes. Nodes remaining: %v", nodes.Elements())
	}
}

func TestReduce(t *testing.T) {
	test1List := gograph.UndirectedAdjListFromFile("test/test1.graph")
	// map list can give various results here, unfortunately
	// because map traversal is random (?)
	dfsNeighborSum := gograph.DFSReduce(test1List, 0, func(node, neighbor, weight, accumulator int) int {
		return accumulator + neighbor
	}, 0)

	recursiveNeighborSum := gograph.DFSRecursiveReduce(test1List, 0, func(node, neighbor, weight, accumulator int) int {
		return accumulator + neighbor
	}, 0)

	if dfsNeighborSum != recursiveNeighborSum {
		t.Errorf("Reduce sums are not equal. Iterative: %v, Recursive: %v", dfsNeighborSum, recursiveNeighborSum)
	}

	// This sum should always be the same
	bfsSum := gograph.BFSReduce(test1List, 0, func(node, neighbor, weight, accumulator int) int {
		return accumulator + node
	}, 0)
	if bfsSum != 10 {
		t.Errorf("BFS Reduce Expected: 10, Actual: %v", bfsSum)
	}
}

func TestBipartite(t *testing.T) {
	test2List := gograph.UndirectedAdjListFromFile("test/test2.graph")
	test2Mat := gograph.UndirectedAdjMatFromFile("test/test2.graph")
	if !gograph.IsBipartite(test2List) {
		t.Errorf("test2List IsBipartite returned false, expected true")
	}

	if !gograph.IsBipartite(test2Mat) {
		t.Errorf("test2Mat IsBipartite returned false, expected true")
	}
}

func TestHasCycle(t *testing.T) {
	// Connected component in figure 3.2
	test1List := gograph.UndirectedAdjListFromFile("test/test1.graph")
	if !gograph.UndirectedHasCycle(test1List) {
		t.Errorf("Expected test1 graph to have cycle.")
	}
	test1Mat := gograph.UndirectedAdjMatFromFile("test/test1.graph")
	if !gograph.UndirectedHasCycle(test1Mat) {
		t.Errorf("Expected test1 graph to have cycle.")
	}

	// Tree should have no cycle
	treeList := gograph.UndirectedAdjListFromFile("test/tree.graph")
	if gograph.UndirectedHasCycle(treeList) {
		t.Errorf("Expected tree to have no cycle.")
	}
}

func TestDijkstra(t *testing.T) {
	// Graph in slide 34 of Greedy powerpoint
	test1List := gograph.DirectedAdjListFromFile("test/dijkstra1.graph")
	expectedShortestPathCosts := []int{0, 9, 32, 45, 34, 14, 15, 50}
	test1ListShortestPathCosts := gograph.DijkstraShortestPathCosts(test1List, 0)
	if !reflect.DeepEqual(expectedShortestPathCosts, *test1ListShortestPathCosts) {
		t.Errorf("Expected: %v, Actual: %v", expectedShortestPathCosts, *test1ListShortestPathCosts)
	}
	test1ListShortestPath := gograph.DijkstraShortestPath(test1List, 0, 7)
	expectedShortestPath := []int{0, 5, 2, 4, 7}
	if !reflect.DeepEqual(expectedShortestPath, *test1ListShortestPath) {
		t.Errorf("Expected: %v, Actual: %v", expectedShortestPath, test1ListShortestPath)
	}
}

func TestMST(t *testing.T) {
	// slide 74 of greedy powerpoint
	test1List := gograph.UndirectedAdjListFromFile("test/prim1.graph")

	expectedMSTCost := 15
	expectedPrim := [][2]int{{0,2}, {2,5}, {5,3}, {2,1}, {1,4}}
	prim, primCost := gograph.PrimMST(test1List)
	if !reflect.DeepEqual(*prim, expectedPrim) {
		t.Errorf("Prim Expected: %v, Actual: %v", expectedPrim, *prim)
	}
	if expectedMSTCost != primCost {
		t.Errorf("Prim MST Cost Expected: %v, Actual: %v", expectedMSTCost, primCost)
	}

	expectedKruskal := [][2]int{{0,2}, {3,5}, {1,4}, {2,5}, {1,2}}
	kruskal, kruskalCost := gograph.KruskalMST(test1List)
	if !reflect.DeepEqual(*kruskal, expectedKruskal) {
		t.Errorf("Kruskal Expected: %v, Actual: %v", expectedKruskal, *kruskal)
	}
	if expectedMSTCost != kruskalCost {
		t.Errorf("Kruskal MST Cost Expected: %v, Actual: %v", expectedMSTCost, kruskalCost)
	}	

	
	cluster := gograph.KClustering(test1List, 2)
	// k-cluster is kruskal graph minus most expensive edge
	expectedCluster := expectedKruskal[:len(expectedKruskal)-1]
	if !reflect.DeepEqual(*cluster, expectedCluster) {
		t.Errorf("Expected 2-cluster to be the graph's MST minus the most expensive edge. Expected: %v, Actual: %v", expectedCluster, *cluster)
	}
}

func TestBellmanFord(t *testing.T) {
	// figure 6.23 in the book
	test1List := gograph.DirectedAdjListFromFile("test/bellmanford1.graph")
	test1Path, test1Costs, test1HasNegCycle := gograph.BellmanFordShortestPath(test1List, 0, 5)
	expectedTest1Path := []int{0,1,4,2,5}
	if !reflect.DeepEqual(*test1Path, expectedTest1Path) {
		t.Errorf("Expected shortest path: %v. Path found: %v", expectedTest1Path, *test1Path)
	}
	if test1Costs == nil {
		t.Errorf("Expected to find shortest path costs for each node")
	} else {
		expectedTest1Costs := []int{-6, -2, 3, 0, 0 ,0}
		if !reflect.DeepEqual(*test1Costs, expectedTest1Costs) {
			t.Errorf("Expected shortest path costs: %v, Actual: %v", expectedTest1Costs, *test1Costs)
		}
	}
	if test1HasNegCycle {
		t.Errorf("Expected graph to have no negative cycles.")
	}

	// figure 6.20 in the book, has a negative cycle
	negativeCycleList := gograph.DirectedAdjListFromFile("test/negativecycle.graph")
	negCyclesPath, negCyclesCosts, hasNegCycles := gograph.BellmanFordShortestPath(negativeCycleList, 0, 7)
	expectedNegCyclesPath := []int{0,1,2,3,4,5,2}
	if !reflect.DeepEqual(*negCyclesPath, expectedNegCyclesPath) {
		t.Errorf("Expected to find the negative cycle within following path: %v. Path found: %v", expectedNegCyclesPath, *negCyclesPath)
	}
	if negCyclesCosts != nil {
		t.Errorf("Expected not to find shortest path costs for each node. Found: %v", *negCyclesCosts)
	}
	if !hasNegCycles {
		t.Errorf("Expected graph to have negative cycle.")
	}
}