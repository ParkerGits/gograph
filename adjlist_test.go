package gograph_test

import (
	"strings"
	"testing"
	"github.com/ParkerGits/gograph"
)

func TestFromFile(t *testing.T) {
	test1Expected := "0: map[1:1 2:1]\n1: map[0:1 2:1 3:1 4:1]\n2: map[0:1 1:1 4:1 6:1 7:1]\n3: map[1:1 4:1]\n4: map[1:1 2:1 3:1 5:1]\n5: map[4:1]\n6: map[2:1 7:1]\n7: map[2:1 6:1]\n"
	test1 := gograph.UndirectedAdjListFromFile("test/test1.graph")
	if val := strings.Compare(test1.String(), test1Expected); val != 0 {
		t.Errorf("test1 graph did not match expected string, comparsion = %d", val)
		expectedActualDiff(t,  test1Expected, test1.String())
	}
}


func expectedActualDiff(t *testing.T, expected interface{}, actual interface{}) {
	t.Logf("\n-- Expected --\n%s\n\n-- Actual --\n%s\n", expected, actual)
}
