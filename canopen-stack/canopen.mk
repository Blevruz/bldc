CANOPENSRC =	canopen-stack/canopen_driver.c \
		canopen-stack/canopen/source/callbacks.c \
		canopen-stack/canopen/source/ \
			canopen-stack/canopen/source/co_core.c \
			canopen-stack/canopen/source/co_csdo.c \
			canopen-stack/canopen/source/co_dict.c \
			canopen-stack/canopen/source/co_emcy.c \
			canopen-stack/canopen/source/co_if.c \
			canopen-stack/canopen/source/co_if_can.c \
			canopen-stack/canopen/source/co_if_nvm.c \
			canopen-stack/canopen/source/co_if_timer.c \
			canopen-stack/canopen/source/co_lss.c \
			canopen-stack/canopen/source/co_nmt.c \
			canopen-stack/canopen/source/co_obj.c \
			canopen-stack/canopen/source/co_para.c \
			canopen-stack/canopen/source/co_pdo.c \
			canopen-stack/canopen/source/co_ssdo.c \
			canopen-stack/canopen/source/co_sync.c \
			canopen-stack/canopen/source/co_tmr.c \
			canopen-stack/canopen/source/co_ver.c \
		canopen-stack/driver/source/ \
			canopen-stack/driver/source/co_can_chos.c \
			canopen-stack/driver/source/co_timer_chos.c \
			canopen-stack/driver/source/co_nvm_chos.c \
			canopen-stack/driver/source/co_timer_swcycle.c

CANOPENINC = canopen-stack \
	     		canopen-stack/canopen \
			canopen-stack/canopen/include \
			canopen-stack/canopen/config \
			canopen-stack/driver/include
