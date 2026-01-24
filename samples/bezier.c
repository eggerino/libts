#include <stdio.h>
#include "ts.h"

ts_vec3 points[] = {
    {0, 0, 0},
    {1, 1, 1},
    {2, 0, 2},
    {1, 1, 3},
};

int main(void) {
    ts_bezier_state bs = {0};
    ts_bezier_init(points, sizeof(points) / sizeof(*points), &bs);

    printf("t,x,y,z\n");
    ts_f64 x;
    for (x = 0; x <= 1; x += 0.01) {
        ts_vec3 pos;
        ts_bezier(&bs, x, &pos);
        printf("%f,%f,%f,%f\n", x, pos.x, pos.y, pos.z);
    }

    ts_bezier_deinit(&bs);
    return 0;
}