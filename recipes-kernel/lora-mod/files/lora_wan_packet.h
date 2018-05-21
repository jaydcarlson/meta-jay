
// Copyright 2016 Solvz, Inc.
// All rights reserved.

#ifndef LORA_WAN_PACKET_H
#define LORA_WAN_PACKET_H


#include "lora_raw.h"


struct lwpacket {                           // lwpacket stands for lora_wan_packet
  struct list_head list;
  
  int32_t expiration_jiffies;               // packets past expiration are to be destroyed.
  
  struct lora_raw_full_packet lr_packet;
};


struct lwpacket *lwpacket_create(void);

uint32_t lwpacket_get_device_id(struct lwpacket *p);

int lwpacket_get_data_bytes(struct lwpacket *p);





#endif

