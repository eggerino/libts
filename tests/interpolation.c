#include "common.h"
#include "ts.h"

void test_linear3(void) {
    ts_linear3_state s = {0};
    ts_vec3 start = {1, 2, 3};
    ts_vec3 end = {4, 6, 8};
    ts_f64 x = 0.25;
    ts_vec3 act = {0};
    ts_vec3 exp = {1.75, 3, 4.25};

    ts_linear3_init(&start, &end, &s);
    ts_linear3(&s, x, &act);
    assert_vec3(exp, act);
}

void test_linear3_d(void) {
    ts_linear3_state s = {0};
    ts_vec3 start = {1, 2, 3};
    ts_vec3 end = {4, 6, 8};
    ts_vec3 act = {0};
    ts_vec3 exp = {3, 4, 5};

    ts_linear3_init(&start, &end, &s);
    ts_linear3_d(&s, &act);
    assert_vec3(exp, act);
}

void test_linear3_dd(void) {
    ts_vec3 act = {0};
    ts_vec3 exp = {0, 0, 0};

    ts_linear3_dd(&act);
    assert_vec3(exp, act);
}

void test_slerp(void) {
    ts_slerp_state s = {0};
    ts_quat start = {1, 2, 3, 4};
    ts_quat end = {4, 3, 2, 1};
    ts_f64 x = 0.25;
    ts_quat act = {0};
    ts_quat exp = {0.34897316187091065, 0.44231611598353937, 0.53565907009616809, 0.62900202420879681};

    ts_quat_normalize(&start, &start);
    ts_quat_normalize(&end, &end);
    ts_slerp_init(&start, &end, &s);
    ts_slerp(&s, x, &act);

    assert_quat(exp, act);
}

void test_slerp_d(void) {
    ts_slerp_state s = {0};
    ts_quat start = {1, 2, 3, 4};
    ts_quat end = {4, 3, 2, 1};
    ts_f64 x = 0.25;
    ts_quat act = {0};
    ts_quat exp = {0.63955391097354952, 0.27170092176156968, -0.096152067450410161, -0.46400505666239};

    ts_quat_normalize(&start, &start);
    ts_quat_normalize(&end, &end);
    ts_slerp_init(&start, &end, &s);

    ts_slerp_d(&s, x, &act);

    assert_quat(exp, act);
}

void test_slerp_dd() {
    ts_slerp_state s = {0};
    ts_quat start = {1, 2, 3, 4};
    ts_quat end = {4, 3, 2, 1};
    ts_f64 x = 0.25;
    ts_quat act = {0};
    ts_quat exp = {0.34897316187091065, 0.44231611598353937, 0.53565907009616809, 0.62900202420879681};

    ts_quat_normalize(&start, &start);
    ts_quat_normalize(&end, &end);
    ts_slerp_init(&start, &end, &s);
    ts_quat_scale(&exp, -s.omega * s.omega, &exp);    
    
    ts_slerp_dd(&s, x, &act);

    assert_quat(exp, act);
}

// TODO bezier, poly5

int main(void) {
    test_linear3();
    test_linear3_d();
    test_linear3_dd();

    test_slerp();
    test_slerp_d();
    test_slerp_dd();
    return 0;
}
