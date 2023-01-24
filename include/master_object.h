#ifndef __MASTER_OBJECT_H
#define __MASTER_OBJECT_H

// struct that holds position, orientation and velocity data of
// the master.
struct MasterObject {
  // TODO:
  // Check if the compiler generates a reasonable copy constructor
  MasterObject(MasterObject &m) = default;
};

#endif
