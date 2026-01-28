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

// TODO bezier

void test_poly5_init_constrained() {
    ts_vec3 start = {1, 2, 3};
    ts_vec3 start_d = {2, 3, 4};
    ts_vec3 start_dd = {3, 4, 5};
    ts_vec3 end = {4, 5, 6};
    ts_vec3 end_d = {5, 6, 7};
    ts_vec3 end_dd = {6, 7, 8};
    ts_poly5_state act = {0};
    ts_poly5_state exp = {{{1, 2, 3}, {2, 3, 4}, {1.5, 2, 2.5}, {-3.5, -14.5, -25.5}, {4.5, 20, 35.5}, {-1.5, -7.5, -13.5}}};

    ts_poly5_init_constrained(&start, &start_d, &start_dd, &end, &end_d, &end_dd, &act);

    int i;
    for (i = 0; i < 6; ++i) {
        assert_vec3(exp.coeffs[i], act.coeffs[i]);
    }
}

void test_poly5(void) {
    ts_poly5_state s = {.coeffs = {{1, 2, 3}, {2, 3, 4}, {3, 4, 5}, {4, 5, 6}, {5, 6, 7}, {6, 7, 8}}};
    ts_f64 x = 0.25;
    ts_vec3 act = {0};
    ts_vec3 exp = {
        .x = 1 + 2.0/4 + 3.0/16 + 4.0/64 + 5.0/256 + 6.0/1024,
        .y = 2 + 3.0/4 + 4.0/16 + 5.0/64 + 6.0/256 + 7.0/1024,
        .z = 3 + 4.0/4 + 5.0/16 + 6.0/64 + 7.0/256 + 8.0/1024,
    };

    ts_poly5(&s, x, &act);

    assert_vec3(exp, act);
}

void test_poly5_d(void) {
    ts_poly5_state s = {.coeffs = {{1, 2, 3}, {2, 3, 4}, {3, 4, 5}, {4, 5, 6}, {5, 6, 7}, {6, 7, 8}}};
    ts_f64 x = 0.25;
    ts_vec3 act = {0};
    ts_vec3 exp = {
        .x = 2.0 + 2*3.0/4 + 3*4.0/16 + 4*5.0/64 + 5*6.0/256,
        .y = 3.0 + 2*4.0/4 + 3*5.0/16 + 4*6.0/64 + 5*7.0/256,
        .z = 4.0 + 2*5.0/4 + 3*6.0/16 + 4*7.0/64 + 5*8.0/256,
    };

    ts_poly5_d(&s, x, &act);

    assert_vec3(exp, act);
}

void test_poly5_dd(void) {
    ts_poly5_state s = {.coeffs = {{1, 2, 3}, {2, 3, 4}, {3, 4, 5}, {4, 5, 6}, {5, 6, 7}, {6, 7, 8}}};
    ts_f64 x = 0.25;
    ts_vec3 act = {0};
    ts_vec3 exp = {
        .x = 2*3.0 + 2*3*4.0/4 + 3*4*5.0/16 + 4*5*6.0/64,
        .y = 2*4.0 + 2*3*5.0/4 + 3*4*6.0/16 + 4*5*7.0/64,
        .z = 2*5.0 + 2*3*6.0/4 + 3*4*7.0/16 + 4*5*8.0/64,
    };

    ts_poly5_dd(&s, x, &act);

    assert_vec3(exp, act);
}

int main(void) {
    test_linear3();
    test_linear3_d();
    test_linear3_dd();

    test_slerp();
    test_slerp_d();
    test_slerp_dd();
    
    test_poly5_init_constrained();
    test_poly5();
    test_poly5_d();
    test_poly5_dd();

    return 0;
}
