CPPFLAGS += \
	-DIMG_ADDRESS=$(IMG_ADDRESS)	\
	-DIMG_SIZE=$(IMG_SIZE)		\
	-DJUMP_ADDR=$(JUMP_ADDR)	\
	-DOF_OFFSET=$(OF_OFFSET)	\
	-DOF_ADDRESS=$(OF_ADDRESS)	\
	-DMEM_BANK=$(MEM_BANK)	\
	-DMEM_SIZE=$(MEM_SIZE)	\
	-DIMAGE_NAME="\"$(IMAGE_NAME)\""	\
	-DCMDLINE="\"$(LINUX_KERNEL_ARG_STRING)\""	\
	-DTOP_OF_MEMORY=$(TOP_OF_MEMORY)	\
	-DMACH_TYPE=$(MACH_TYPE)		\

ASFLAGS += -DJUMP_ADDR=$(JUMP_ADDR)		\
	-DTOP_OF_MEMORY=$(TOP_OF_MEMORY)	\
	-DMACH_TYPE=$(MACH_TYPE)		\

ifeq ($(CONFIG_DEBUG),y)
CPPFLAGS += -DCONFIG_DEBUG
endif

ifeq ($(CONFIG_UBI),y)
CPPFLAGS += -DUBI_ADDRESS=$(CONFIG_UBI_ADDRESS)
CPPFLAGS += -DUBI_OFFSET=$(CONFIG_UBI_OFFSET)
CPPFLAGS += -DUBI_SIZE=$(CONFIG_UBI_SIZE)
CPPFLAGS += -DUBI_KERNEL_VOLNAME="\"$(CONFIG_UBI_KERNEL_VOLNAME)\""
CPPFLAGS += -DUBI_KERNEL_SPARE_VOLNAME="\"$(CONFIG_UBI_KERNEL_SPARE_VOLNAME)\""
CPPFLAGS += -DUBI_DTB_VOLNAME="\"$(CONFIG_UBI_DTB_VOLNAME)\""
CPPFLAGS += -DUBI_DTB_SPARE_VOLNAME="\"$(CONFIG_UBI_DTB_SPARE_VOLNAME)\""
endif

ifeq ($(CONFIG_HW_DISPLAY_BANNER),y)
BANNER:="$(CONFIG_HW_BANNER)"
CPPFLAGS += -DBANNER="$(BANNER)"
CPPFLAGS += -DCONFIG_HW_DISPLAY_BANNER
endif

ifeq ($(CONFIG_HW_INIT),y)
CPPFLAGS += -DCONFIG_HW_INIT
endif

ifeq ($(CONFIG_USER_HW_INIT),y)
CPPFLAGS += -DCONFIG_USER_HW_INIT
endif

ifeq ($(CONFIG_OVERRIDE_CMDLINE),y)
CPPFLAGS += -DCONFIG_OVERRIDE_CMDLINE
endif
