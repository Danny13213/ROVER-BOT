CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic
BUILD_DIR := build
APP := $(BUILD_DIR)/rover_controlA
TEST := $(BUILD_DIR)/test_rover
MODULES := main.c rover_logic.c navigation.c serial.c terminal.c time_util.c

.PHONY: all test clean

all: $(APP)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(APP): $(MODULES) rover.h rover_logic.h navigation.h serial.h terminal.h time_util.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(MODULES) -lm -o $(APP)

$(TEST): test_rover.c rover_logic.c navigation.c rover.h rover_logic.h navigation.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) test_rover.c rover_logic.c navigation.c -lm -o $(TEST)

test: $(TEST)
	./$(TEST)

clean:
	rm -rf $(BUILD_DIR)