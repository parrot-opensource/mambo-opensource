#
# Makefile for the linux kernel.
#

obj-y	:= clock.o timer.o devs.o parrot6.o lcdpanel.o pm.o parrot6i.o board-sysfs.o
#camerasensor.o
obj-y += p6_wdt.o
obj-$(CONFIG_OPROFILE_PARROT) += oprofile.o

obj-$(CONFIG_MACH_PARROT_P6DEV) += parrot6dev.o
obj-$(CONFIG_MACH_PARROT_P6IDEV) += parrot6idev.o
obj-$(CONFIG_MACH_PARROT_SIP6DEV) += parrot6idev.o
obj-$(CONFIG_MACH_PARROT_PARELIA) += parelia.o
obj-$(CONFIG_MACH_PARROT_RNB4) += rnb4.o
obj-$(CONFIG_MACH_PARROT_FC6XXX) += fc6xxx.o
obj-$(CONFIG_MACH_PARROT_MYKONOS) += mykonos.o
obj-$(CONFIG_MACH_PARROT_FC6050) += fc6050.o
obj-$(CONFIG_MACH_PARROT_FC6000P) += fc6000p.o
obj-$(CONFIG_MACH_PARROT_DT1310) += dt1310.o
obj-$(CONFIG_MACH_PARROT_JPSUMO) += jpsumo.o
obj-$(CONFIG_MACH_PARROT_POWERUP) += powerup.o
obj-$(CONFIG_MACH_PARROT_DELOS) += delos.o
