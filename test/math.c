#include "common.h"
#include "ts.h"


void test_vec3_add(void) {
    ts_vec3 a = {1, 2, 3};
    ts_vec3 b = {4, 5, 6};
    ts_vec3 act = {0};
    ts_vec3 exp = {5, 7, 9};

    ts_vec3_add(&a, &b, &act);
    assert_vec3(exp, act);
}

int main(void) {
    test_vec3_add();
    return 0;
}
