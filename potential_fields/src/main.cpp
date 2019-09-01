#include "Potential_Fields.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "potentialFields");
    PotentialFields potentialFields;
    ros::spin();
    return 0;
}
