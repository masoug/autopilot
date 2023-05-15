#include "pico/stdlib.h"

#include <cstdio>

#include "utils.hpp"


int main()
{
    stdio_init_all();

    sleep_ms(3000);

    printf("lla2ecef() tests:\n");
    {
        // sf golden gate
        auto ecef = utils::LocalNED::lla2ecef(37.81123473439707, -122.4774845440041, 123.4);
        printf("lla2ecef() results:\n");
        printf("  X: expected=-2705808.11778127 got=%f\n", ecef[0]);
        printf("  y: expected=-4250953.42296182 got=%f\n", ecef[1]);
        printf("  Z: expected=3884092.61681885 got=%f\n", ecef[2]);
        printf("\n\n");

        // sf golden gate 2
        ecef = utils::LocalNED::lla2ecef(37.812214083769526, -122.4776198744307, 123.4);
        printf("lla2ecef() results:\n");
        printf("  X: expected=-2705782.26810597 got=%f\n", ecef[0]);
        printf("  y: expected=-4250890.64692751 got=%f\n", ecef[1]);
        printf("  Z: expected=3884178.17145778 got=%f\n", ecef[2]);
        printf("\n\n");

        // sf golden gate 3
        ecef = utils::LocalNED::lla2ecef(37.81299241805834, -122.47772272625046, 123.4);
        printf("lla2ecef() results:\n");
        printf("  X: expected=-2705761.37459524 got=%f\n", ecef[0]);
        printf("  y: expected=-4250840.97715229 got=%f\n", ecef[1]);
        printf("  Z: expected=3884246.16488105 got=%f\n", ecef[2]);
        printf("\n\n");
    }

    printf("LocalNED tests:\n");
    {
        // sf golden gate
        const utils::LocalNED local_ned(37.81123473439707, -122.4774845440041, 123.4);

        Eigen::Vector3f ned = local_ned.lla2ned(37.81123473439707, -122.4774845440041, 123.4);
        printf("lla2ned() results:\n");
        printf("  N: expected=0 got=%f\n", ned[0]);
        printf("  E: expected=0 got=%f\n", ned[1]);
        printf("  D: expected=0 got=%f\n", ned[2]);
        printf("\n\n");

        ned = local_ned.lla2ned(37.812214083769526, -122.4776198744307, 3.14);
        printf("lla2ned() results:\n");
        printf("  N: expected=108.56521432 got=%f\n", ned[0]);
        printf("  E: expected=-11.9016547 got=%f\n", ned[1]);
        printf("  D: expected=120.61442456 got=%f\n", ned[2]);
        printf("\n\n");

        ned = local_ned.lla2ned(37.81299241805834, -122.47772272625046, 543.21);
        printf("lla2ned() results:\n");
        printf("  N: expected=194.86359999 got=%f\n", ned[0]);
        printf("  E: expected=-20.94852629 got=%f\n", ned[1]);
        printf("  D: expected=-419.17255745 got=%f\n", ned[2]);
        printf("\n\n");
    }

    printf("Tests done.\n");
    sleep_ms(1000);
    return 0;
}
