GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
EXTRA_CFLAGS += -Idrivers/media/video/msm
EXTRA_CFLAGS += -Idrivers/media/video/msm/io
obj-$(CONFIG_MSM_ACTUATOR) += msm_actuator.o
obj-$(CONFIG_IMX091_ACT) += imx091_act.o
obj-$(CONFIG_IMX111_ACT) += imx111_act.o
obj-$(CONFIG_BU64291_ACT) += bu64291_act.o
obj-$(CONFIG_BH6455GUL_ACT) += bh6455gul_act.o
