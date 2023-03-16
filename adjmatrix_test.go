package gograph_test

import (
	"github.com/ParkerGits/gograph"
	"strings"
	"testing"
)

func TestAdjMatrix(t *testing.T) {
	test1Expected := "0: [0 1 1 0 0 0 0 0]\n1: [1 0 1 1 1 0 0 0]\n2: [1 1 0 0 1 0 1 1]\n3: [0 1 0 0 1 0 0 0]\n4: [0 1 1 1 0 1 0 0]\n5: [0 0 0 0 1 0 0 0]\n6: [0 0 1 0 0 0 0 1]\n7: [0 0 1 0 0 0 1 0]\n"
	test1 := gograph.UndirectedAdjMatFromFile("test/test1.graph")
	if val := strings.Compare(test1.String(), test1Expected); val != 0 {
		t.Errorf("test1 graph did not match expected string, comparison = %d", val)
		expectedActualDiff(t,  test1Expected, test1.String())
	}
}
