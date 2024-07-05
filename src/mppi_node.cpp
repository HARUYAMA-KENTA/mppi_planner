#include "mppi_path/mppi.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "mppi_planner");
    MPPIPlanner mppi;
    mppi.run();
    return 0;
}