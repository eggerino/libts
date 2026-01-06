#include <assert.h>

#include "tests.h"
#include "ts.h"

void test_interpolation(void) {
    assert(0 == ts_interpolate_linear1(0, 1, 0));
    assert(1 == ts_interpolate_linear1(0, 1, 1));
    assert(2 == ts_interpolate_linear1(0, 1, 2));
    assert(-1 == ts_interpolate_linear1(0, 1, -1));
    assert(1 == ts_interpolate_linear1(1, 1, -1));
}
