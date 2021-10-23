TARGETS:= lora-sx1262

#   npl/linux/src/os_eventq.cc
CSRCS  := \
	src/main.c \
	src/radio.c \
	src/sx126x.c \
	src/sx126x-linux.c \
    npl/linux/src/os_callout.c \
    npl/linux/src/os_sem.c \
    npl/linux/src/os_task.c \
    npl/linux/src/os_atomic.c \
    npl/linux/src/os_time.c \
    npl/linux/src/os_mutex.c \

DEPS   := \
	include/radio.h \
	include/sx126x-board.h \
	include/sx126x.h \
	include/sx126x-utilities.h \
    npl/linux/include/nimble/nimble_npl_os.h \
	npl/linux/include/nimble/nimble_npl.h \
    npl/linux/include/nimble/os_types.h \
    npl/linux/include/console/console.h \
    npl/linux/src/wqueue.h \

# TODO: -Werror=all
CCFLAGS:= \
	-g \
	-Wall \
	-Wextra \
	-Wno-unused-parameter \
	-Wno-sign-compare \
	-Wno-old-style-declaration \
	-I include \
	-I npl/linux/include \
	-I npl/linux/include/nimble \
	
LDFLAGS:= \
	-pthread \
	-lrt \
	-lstdc++ \

CC     := gcc
CPP    := gcc

MAINS  := $(addsuffix .o, $(TARGETS) )
OBJ    := \
	$(MAINS) \
	$(CSRCS:.c=.o)

.PHONY: all clean

all: $(TARGETS)

clean:
	rm src/*.o || true
	rm npl/linux/src/*.o || true

$(OBJ): %.o : %.c $(DEPS)
	$(CC) -c -o $@ $< $(CCFLAGS)

# $(OBJ): %.o : %.cc $(DEPS)
# 	$(CPP) -c -o $@ $< $(CCFLAGS)

$(TARGETS): % : $(filter-out $(MAINS), $(OBJ))
	$(CC) -o $@ \
	npl/linux/src/os_eventq.cc \
	$(LIBS) \
	$^ \
	$(CCFLAGS) \
	$(LDFLAGS) \
