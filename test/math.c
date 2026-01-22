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

void test_vec3_sub(void) {
    ts_vec3 a = {1, 2, 3};
    ts_vec3 b = {4, 5, 6};
    ts_vec3 act = {0};
    ts_vec3 exp = {-3, -3, -3};

    ts_vec3_sub(&a, &b, &act);
    assert_vec3(exp, act);
}

void test_vec3_scale(void) {
    ts_vec3 v = {1, 2, 3};
    ts_f64 factor = 2.5;
    ts_vec3 act = {0};
    ts_vec3 exp = {2.5, 5, 7.5};

    ts_vec3_scale(&v, factor, &act);
    assert_vec3(exp, act);
}

void test_vec3_dot(void) {
    ts_vec3 a = {1, 2, 3};
    ts_vec3 b = {4, 5, 6};
    ts_f64 exp = 32.0;

    ts_f64 act = ts_vec3_dot(&a, &b);
    assert_eps(exp, act);
}

void test_vec3_cross(void) {
    ts_vec3 a = {1, 2, 3};
    ts_vec3 b = {4, 5, 6};
    ts_vec3 act = {0};
    ts_vec3 exp = {-3, 6, -3};

    ts_vec3_cross(&a, &b, &act);
    assert_vec3(exp, act);
}

void test_vec3_norm_l2(void) {
    ts_vec3 v = {1, 2, 3};
    ts_f64 exp = ts_sqrt(14);

    ts_f64 act = ts_vec3_norm_l2(&v);
    assert_eps(exp, act);
}

void test_vec3_normalize(void) {
    ts_vec3 v = {1, 2, 3};
    ts_vec3 act = {0};
    ts_vec3 exp = {1.0 / ts_sqrt(14), 2 / ts_sqrt(14), 3 / ts_sqrt(14)};

    ts_vec3_normalize(&v, &act);
    assert_vec3(exp, act);
}

void test_quat_add(void) {
    ts_quat a = {1, 2, 3, 4};
    ts_quat b = {4, 5, 6, 7};
    ts_quat act = {0};
    ts_quat exp = {5, 7, 9, 11};

    ts_quat_add(&a, &b, &act);
    assert_quat(exp, act);
}

void test_quat_sub(void) {
    ts_quat a = {1, 2, 3, 4};
    ts_quat b = {4, 5, 6, 7};
    ts_quat act = {0};
    ts_quat exp = {-3, -3, -3, -3};

    ts_quat_sub(&a, &b, &act);
    assert_quat(exp, act);
}

void test_quat_scale(void) {
    ts_quat q = {1, 2, 3, 4};
    ts_f64 factor = 2.5;
    ts_quat act = {0};
    ts_quat exp = {2.5, 5, 7.5, 10};

    ts_quat_scale(&q, factor, &act);
    assert_quat(exp, act);
}

void test_quat_dot(void) {
    ts_quat a = {1, 2, 3, 4};
    ts_quat b = {4, 5, 6, 7};
    ts_f64 exp = 60;

    ts_f64 act = ts_quat_dot(&a, &b);
    assert_eps(exp, act);
}

void test_quat_mul(void) {
    ts_quat a = {1, 2, 3, 4};
    ts_quat b = {4, 5, 6, 7};
    ts_quat act = {0};
    ts_quat exp = {-52, 10, 24, 20};

    ts_quat_mul(&a, &b, &act);
    assert_quat(exp, act);
}

void test_quat_normalize(void) {
    ts_quat q = {1, 2, 3, 4};
    ts_quat act = {0};
    ts_quat exp = {1.0 / ts_sqrt(30), 2.0 / ts_sqrt(30), 3.0 / ts_sqrt(30), 4.0 / ts_sqrt(30)};

    ts_quat_normalize(&q, &act);
    assert_quat(exp, act);
}

void test_mat3x3_add(void) {
    ts_mat3x3 a = {
        1, 2, 3, 4, 5, 6, 7, 8, 9,
    };
    ts_mat3x3 b = {
        4, 5, 6, 7, 8, 9, 1, 2, 3,
    };
    ts_mat3x3 act = {0};
    ts_mat3x3 exp = {
        5, 7, 9, 11, 13, 15, 8, 10, 12,
    };

    ts_mat3x3_add(&a, &b, &act);
    assert_mat3x3(exp, act);
}

void test_mat3x3_sub(void) {
    ts_mat3x3 a = {
        1, 2, 3, 4, 5, 6, 7, 8, 9,
    };
    ts_mat3x3 b = {
        4, 5, 6, 7, 8, 9, 1, 2, 3,
    };
    ts_mat3x3 act = {0};
    ts_mat3x3 exp = {
        -3, -3, -3, -3, -3, -3, 6, 6, 6,
    };

    ts_mat3x3_sub(&a, &b, &act);
    assert_mat3x3(exp, act);
}

void test_mat3x3_mul(void) {
    ts_mat3x3 a = {
        1, 2, 3, 4, 5, 6, 7, 8, 9,
    };
    ts_mat3x3 b = {
        4, 5, 6, 7, 8, 9, 1, 2, 3,
    };
    ts_mat3x3 act = {0};
    ts_mat3x3 exp = {
        21, 27, 33, 57, 72, 87, 93, 117, 141,
    };

    ts_mat3x3_mul(&a, &b, &act);
    assert_mat3x3(exp, act);
}

void test_mat3x3_vec3_mul(void) {
    ts_mat3x3 a = {
        1, 2, 3, 4, 5, 6, 7, 8, 9,
    };
    ts_vec3 b = {4, 5, 6};
    ts_vec3 act = {0};
    ts_vec3 exp = {32, 77, 122};

    ts_mat3x3_vec3_mul(&a, &b, &act);
    assert_vec3(exp, act);
}

void test_mat3x3_to_quat1(void) {
    ts_mat3x3 m = {
        0.41198225991309989, 0.05872672981356003, 0.90929742922092005,
        -0.6812427255054001, -0.64287283953050012, 0.35017557591120008,
        0.60512725316939997, -0.76371837494208006, -0.22484507008615995,
    };
    ts_quat act = {0};
    ts_quat exp = {-0.36887138766997318, 0.75493381109501456, -0.20614919310918339, 0.50150908323637811};

    ts_mat3x3_to_quat(&m, &act);
    assert_quat(exp, act);
}

void test_mat3x3_to_quat2(void) {
    ts_mat3x3 m = {
        -0.5348952, -0.0762475,  0.8414710,
        -0.8162168,  0.3040046, -0.4912955,
        -0.2183511, -0.9496144, -0.2248451,
    };
    ts_quat act = {0};
    ts_quat exp = {0.36887139593729373, -0.31062246405807137, 0.71828698336410923, -0.50150911764612793 };

    ts_mat3x3_to_quat(&m, &act);
    assert_quat(exp, act);
}

void test_mat3x3_to_quat3(void) {
    ts_mat3x3 m = {
        -0.2248451,  0.3501755,  0.9092974,
        -0.7637183, -0.6428728,  0.0587266,
        0.6051273, -0.6812427,  0.4119822,
    };
    ts_quat act = {0};
    ts_quat exp = {-0.36887137091079952, 0.50150913420128229, -0.20614924620376229, 0.75493380008190059};

    ts_mat3x3_to_quat(&m, &act);
    assert_quat(exp, act);
}

void test_mat3x3_to_quat4(void) {
    ts_mat3x3 m = {
         0.4119822,  0.9001976,  0.1411200,
        -0.4318012,  0.0564971,  0.9001976,
        0.8023829, -0.4318012,  0.4119822,
    };
    ts_quat act = {0};
    ts_quat exp = {-0.68564959653235036, 0.48567038567515614, 0.24110818093505204, 0.48567038567515614};

    ts_mat3x3_to_quat(&m, &act);
    assert_quat(exp, act);
}

void test_quat_to_mat3x3(void) {
    ts_quat q = {-0.3688714, 0.7549338, -0.2061492, 0.5015091};
    ts_mat3x3 act = {0};
    ts_mat3x3 exp = {
        0.41198225991309989, 0.05872672981356003, 0.90929742922092005,
        -0.6812427255054001, -0.64287283953050012, 0.35017557591120008,
        0.60512725316939997, -0.76371837494208006, -0.22484507008615995,
    };

    ts_quat_to_mat3x3(&q, &act);
    assert_mat3x3(exp, act);
}

void test_vec3_to_imag_quat(void) {
    ts_vec3 v = {2, 3, 4};
    ts_quat act = {0};
    ts_quat exp = {0, 2, 3, 4};

    ts_vec3_to_imag_quat(&v, &act);
    assert_quat(exp, act);
}

void test_quat_imag_to_vec3(void) {
    ts_quat q = {1, 2, 3, 4};
    ts_vec3 act = {0};
    ts_vec3 exp = {2, 3, 4};

    ts_quat_imag_to_vec3(&q, &act);
    assert_vec3(exp, act);
}

void test_quat_to_axis_angle_vec3_1(void) {
    ts_quat q = {-1, 0, 0, 0};
    ts_vec3 act = {0};
    ts_vec3 exp = {0, 0, 0};
    
    ts_quat_to_axis_angle_vec3(&q, &act);
    assert_vec3(exp, act);
}

void test_quat_to_axis_angle_vec3_2(void) {
    ts_quat q = {-0.6856496, 0.4856704, 0.2411082, 0.4856704};
    ts_vec3 act = {0};
    ts_vec3 exp = {3.1041679218364981, 1.5410458206465512, 3.1041679218364981};
    
    ts_quat_to_axis_angle_vec3(&q, &act);
    assert_vec3(exp, act);
}

void test_vec3_axis_angle_to_quat1(void) {
    ts_vec3 v = {0, 0, 0};
    ts_quat act = {0};
    ts_quat exp = {1, 0, 0, 0};
    
    ts_vec3_axis_angle_to_quat(&v, &act);
    assert_quat(exp, act);
}

void test_vec3_axis_angle_to_quat2(void) {
    ts_vec3 v = {3.1041679218364981, 1.5410458206465512, 3.1041679218364981};
    ts_quat act = {0};
    ts_quat exp = {-0.68564962070804958, 0.48567038104709903, 0.24110819059094432, 0.48567038104709903};
    
    ts_vec3_axis_angle_to_quat(&v, &act);
    assert_quat(exp, act);
}

int main(void) {
    test_vec3_add();
    test_vec3_sub();
    test_vec3_scale();
    test_vec3_dot();
    test_vec3_cross();
    test_vec3_norm_l2();
    test_vec3_normalize();

    test_quat_add();
    test_quat_sub();
    test_quat_scale();
    test_quat_dot();
    test_quat_mul();
    test_quat_normalize();

    test_mat3x3_add();
    test_mat3x3_sub();
    test_mat3x3_mul();
    test_mat3x3_vec3_mul();

    test_mat3x3_to_quat1();
    test_mat3x3_to_quat2();
    test_mat3x3_to_quat3();
    test_mat3x3_to_quat4();
    test_quat_to_mat3x3();
    test_vec3_to_imag_quat();
    test_quat_imag_to_vec3();
    test_quat_to_axis_angle_vec3_1();
    test_quat_to_axis_angle_vec3_2();
    test_vec3_axis_angle_to_quat1();
    test_vec3_axis_angle_to_quat2();
    return 0;
}
