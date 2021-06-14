obj-n += HALLsensor.o
obj-n += ALSPSsensor.o
obj-n += psensor.o
obj-n += lsensor.o
obj-y += ALSPS_FRGBsensor.o
#obj-y += SARsensor.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_ALGO_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)
