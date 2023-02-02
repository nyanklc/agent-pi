#ifndef __MASTER_OBJECT_H
#define __MASTER_OBJECT_H

// idk if units should be meters or centimeters
// we may try and find out which one performs better
// (RPi 4 has hardware floating-point support)
struct Pose {
    double x;
    double y;
    double yaw;
};

// struct that holds position, orientation and velocity data of
// the master.
struct MasterObject {
    MasterObject();

    // TODO: set values

    double width;  // long line of the marker

    Pose pose;
    double vel;  // double or sth different?
};

#endif
