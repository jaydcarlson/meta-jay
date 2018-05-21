
// Copyright 2016 Solvz, Inc.
// All rights reserved.

#ifndef LORA_WAN_H
#define LORA_WAN_H



#include <linux/types.h>


#define USER_PACKET_DATA_LENGTH 256

struct lw_user_packet {
    uint32_t device_id;
    int      data_bytes;
    uint8_t  data[USER_PACKET_DATA_LENGTH];
};








#endif

