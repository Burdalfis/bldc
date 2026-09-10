CSRC += \
	motor/foc_math.c \
	motor/mc_interface.c \
	motor/mcpwm_foc.c \
	motor/virtual_motor.c

ifeq ($(PROJECT),mini4)
CSRC += motor/mcpwm_stub.c
else
CSRC += motor/mcpwm.c
endif
	
INCDIR += motor

