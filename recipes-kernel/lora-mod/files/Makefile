obj-m := lora_raw_driver.o
lora_raw_driver-objs := lora_raw.o lora_timer.o lmic_hal.o deferred.o lmic/radio.o lmic/aes.o
	
obj-m += lora_raw_chardev.o
    
obj-m += lora_wan_driver.o
lora_wan_driver-objs := lora_wan.o lora_wan_packet.o

#set frequency band to US 915 ISM band and select SX1272 radio chip
ccflags-y := -DCFG_us915 -DCFG_sx1272_radio

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
