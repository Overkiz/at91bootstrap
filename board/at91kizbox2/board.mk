$(shell $(CC) --target-help > tmp.file)
gcc_cortexa5=$(shell grep cortex-a5 tmp.file)

ifeq (, $(findstring cortex-a5,$(gcc_cortexa5)))
CPPFLAGS += -DCONFIG_AT91KIZBOX2

ASFLAGS += \
	-DCONFIG_AT91KIZBOX2
else
CPPFLAGS += \
	-DCONFIG_AT91KIZBOX2 \
	-mtune=cortex-a5

ASFLAGS += \
	-DCONFIG_AT91KIZBOX2 \
	-mcpu=cortex-a5
endif

$(shell rm tmp.file)
