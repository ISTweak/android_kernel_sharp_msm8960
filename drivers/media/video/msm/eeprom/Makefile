GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
EXTRA_CFLAGS += -Idrivers/media/video/msm/io
obj-$(CONFIG_MSM_EEPROM) += msm_camera_eeprom.o
obj-$(CONFIG_IMX074_EEPROM) += imx074_eeprom.o
obj-$(CONFIG_IMX091_EEPROM) += imx091_eeprom.o
obj-$(CONFIG_IMX111_EEPROM) += imx111_eeprom.o
obj-$(CONFIG_S5K3L1YX_EEPROM) += s5k3l1yx_eeprom.o