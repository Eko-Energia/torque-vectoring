CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -O2
BUILD_DIR := build
SRC_DIR := c_implementation

.PHONY: all test clean

all: $(BUILD_DIR)/torque-vectoring

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/torque-vectoring: $(SRC_DIR)/main.c $(SRC_DIR)/torque_vectoring.c $(SRC_DIR)/torque_vectoring.h $(SRC_DIR)/config.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(SRC_DIR)/main.c $(SRC_DIR)/torque_vectoring.c -o $@

$(BUILD_DIR)/test-torque-vectoring: $(SRC_DIR)/test_torque_vectoring.c $(SRC_DIR)/torque_vectoring.c $(SRC_DIR)/torque_vectoring.h $(SRC_DIR)/config.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(SRC_DIR)/test_torque_vectoring.c $(SRC_DIR)/torque_vectoring.c -o $@

$(BUILD_DIR)/test-e2e-torque-vectoring: $(SRC_DIR)/test_e2e_torque_vectoring.c $(SRC_DIR)/torque_vectoring.c $(SRC_DIR)/torque_vectoring.h $(SRC_DIR)/config.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(SRC_DIR)/test_e2e_torque_vectoring.c $(SRC_DIR)/torque_vectoring.c -o $@

test: $(BUILD_DIR)/test-torque-vectoring $(BUILD_DIR)/test-e2e-torque-vectoring
	./$(BUILD_DIR)/test-torque-vectoring
	./$(BUILD_DIR)/test-e2e-torque-vectoring

clean:
	rm -f $(BUILD_DIR)/torque-vectoring $(BUILD_DIR)/test-torque-vectoring $(BUILD_DIR)/test-e2e-torque-vectoring
