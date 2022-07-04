ifeq (,$(filter edo, $(SOMC_TARGET_PLATFORM)))
ccflags-y += -DCONFIG_ET580_USE_TIMER_SETUP
endif

obj-m := et580-int.o

ifeq (,$(filter edo, $(SOMC_TARGET_PLATFORM)))
et580-int-objs := et580_platform.o
else
et580-int-objs := et580_platform.o navi_input.o
endif
