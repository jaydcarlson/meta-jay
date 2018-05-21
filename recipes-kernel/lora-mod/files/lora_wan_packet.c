
// Copyright 2016 Solvz, Inc.
// All rights reserved.

#include "lora_wan_packet.h"

/**
 * @return pointer to newly allocated lwpacket, or NULL on failure
 */
struct lwpacket *lwpacket_create(void)
{
    struct lwpacket *p;
    p = kzalloc(sizeof(struct lwpacket), GFP_ATOMIC);
    if (p != NULL) {
        INIT_LIST_HEAD(&p->list);
    }
    return p;
}

/** read the device id out of a packet
 */
uint32_t lwpacket_get_device_id(struct lwpacket *p)
{
    return 0;   // todo
}


int lwpacket_get_data_bytes(struct lwpacket *p)
{
    return 0;   // todo
}
