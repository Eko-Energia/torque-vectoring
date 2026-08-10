CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -O2

.PHONY: all test clean

all: torque-vectoring-double

torque-vectoring-double: main.c torque_vectoring.c torque_vectoring.h config.h
	$(CC) $(CFLAGS) main.c torque_vectoring.c -lm -o $@

test-double: test_torque_vectoring.c torque_vectoring.c torque_vectoring.h config.h
	$(CC) $(CFLAGS) test_torque_vectoring.c torque_vectoring.c -lm -o $@

test: test-double
	./test-double

clean:
	rm -f torque-vectoring-double test-double
