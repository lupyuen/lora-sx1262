TARGETS:= lora-sx1262

CSRCS  := \
	src/main.c \
	src/radio.c \
	src/sx126x.c \
	src/sx126x-linux.c \

DEPS   := \
	include/radio.h \
	include/sx126x-board.h \
	include/sx126x.h \
	include/sx126x-utilities.h \

# TODO: -Werror=all
CCFLAGS:= \
	-g \
	-Wall \
	-Wextra \
	-Wno-unused-parameter \
	-Wno-sign-compare \
	-Wno-old-style-declaration \
	-I include \

LDFLAGS:= 

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

$(OBJ): %.o : %.c $(DEPS)
	$(CC) -c -o $@ $< $(CCFLAGS)

# $(OBJ): %.o : %.cpp $(DEPS)
#	$(CPP) -c -o $@ $< $(CCFLAGS)

$(TARGETS): % : $(filter-out $(MAINS), $(OBJ))
	$(CC) -o $@ \
	$(LIBS) \
	$^ \
	$(CCFLAGS) \
	$(LDFLAGS)
