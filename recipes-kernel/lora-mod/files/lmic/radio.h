
#ifndef RADIO_H
#define RADIO_H


#include <linux/types.h>

#include "oslmic.h"
#include "lorabase.h"



bool radio_detect (struct hal_radio *r);
void radio_init (struct hal_radio *r);
void radio_start_receiving(struct hal_radio *r, int channel, dr_t data_rate);
void radio_shutdown (struct hal_radio *r);
void radio_irq_handler (struct hal_radio *r);
int  radio_get_num_channels(void);
int  radio_tx(struct hal_radio *r, int channel, dr_t data_rate, s1_t tx_power, void *data, int len);
void radio_set_rx_handler(struct hal_radio *r, 
                          void (*rx_packet_handler)(void *, struct hal_radio_packet *),
                          void *rx_packet_handler_data);

void printk_lora_registers(struct hal_radio *r);    //debug


#endif

