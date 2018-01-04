
#include "Arduino.h"



#ifndef COMM_CONFIG_H
#define COMM_CONFIG_H

#if defined COMM_SERIAL

    #define COMMUNICATOR Serial

#elif defined COMM_WIFI

//#define COMMUNICATION X


#endif


#endif
