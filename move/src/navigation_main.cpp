#include "navigation_0708.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NavigateToTarget");
    Navigation nav;
    nav.Run();
}
