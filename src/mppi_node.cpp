#include "mppi_path/mppi.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mppi_planner");
    MPPIPlanner mppi;

    ros::Rate loop_rate(10); // 10 Hzのループ

    while (ros::ok())
    {
        mppi.calc_optical_control_input();
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }

    return 0;
}
