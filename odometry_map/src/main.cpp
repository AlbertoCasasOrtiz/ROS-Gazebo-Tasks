#include "OdometryMap.h"

int main(int argc, char **argv) {
    OdometryMap* odometryMap = new OdometryMap(argc, argv);

    delete odometryMap;
    return 0;
}
