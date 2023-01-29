#ifndef __GLOBALS_H
#define __GLOBALS_H


#define VIDEO_SOURCE                    1
#define VIEW_MODE                       true

#define SERIAL_ON                       false
#define SERIAL_PORT                     "/dev/ttyUSB0"
#define SERIAL_BAUDRATE                 9600
#define ARDUINO_RESPONSE_ON             false

#define TURN_TOLERANCE                  50
#define REFERENCE_IMG_PATH              "ref_img/sekil_ref.jpg"


#define FOCAL_LENGTH                    491.54655888988026
#define CALIBRATE_MODE                  false
#define CALIBRATE_MODE_TRIAL_COUNT      10
#define QR_LENGTH                       16.5
#define OBJ_LENGTH                      19

#define KNN_ENABLED                     false
#define KNN_K                           2

#define FPS_ON                          true
#define NO_FRAME_FPS_ON                 false

#define APRILTAG_ENABLED                true
#define APRILTAG_THREAD_COUNT           1
#define APRILTAG_FAMILY_BIT_COUNT       2
#define APRILTAG_DEBUG_ON               true
#define APRILTAG_QUAD_DECIMATE          2.0
#define APRILTAG_QUAD_SIGMA             0.0
#define APRILTAG_REFINE_EDGES           1



#endif
