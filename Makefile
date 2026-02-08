# SCARA Robot Kinematics & Control - Makefile

CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -I./include -O2
LDFLAGS = -lm -lws2_32

# Directories
SRC_DIR = src
INC_DIR = include
BUILD_DIR = build
TEST_DIR = tests

EXE = .exe

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

# Main executable (exclude main.c for tests)
MAIN_OBJ = $(BUILD_DIR)/main.o
LIB_OBJS = $(filter-out $(MAIN_OBJ),$(OBJS))

# Test files
TEST_SRCS = $(wildcard $(TEST_DIR)/test_*.c)
TEST_BINS = $(patsubst $(TEST_DIR)/%.c,$(BUILD_DIR)\\%$(EXE),$(TEST_SRCS))

# Unity test framework
UNITY_SRC = $(TEST_DIR)/unity/unity.c
UNITY_OBJ = $(BUILD_DIR)/unity.o

# Targets
TARGET = $(BUILD_DIR)/scara_sim

.PHONY: all clean test dirs

all: dirs $(TARGET)

dirs:
	@if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Unity framework
$(UNITY_OBJ): $(UNITY_SRC)
	$(CC) $(CFLAGS) -c -o $@ $<

# Test targets
test: dirs $(UNITY_OBJ) $(LIB_OBJS) $(TEST_BINS)
	@echo Running all tests...
	@for %%t in ($(TEST_BINS)) do call %%t

$(BUILD_DIR)/test_%: $(TEST_DIR)/test_%.c $(UNITY_OBJ) $(LIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Individual test runners
test_math: dirs $(BUILD_DIR)/test_math_utils
	$(BUILD_DIR)/test_math_utils

test_dh: dirs $(BUILD_DIR)/test_dh_transform
	$(BUILD_DIR)/test_dh_transform

test_fk: dirs $(BUILD_DIR)/test_forward_kinematics
	$(BUILD_DIR)/test_forward_kinematics

test_ik: dirs $(BUILD_DIR)/test_inverse_kinematics
	$(BUILD_DIR)/test_inverse_kinematics

test_pid: dirs $(BUILD_DIR)/test_pid_controller
	$(BUILD_DIR)/test_pid_controller

test_traj: dirs $(BUILD_DIR)/test_trajectory
	$(BUILD_DIR)/test_trajectory

test_int: dirs $(BUILD_DIR)/test_integration
	$(BUILD_DIR)/test_integration

clean:
	@if exist $(BUILD_DIR) rmdir /s /q $(BUILD_DIR)

# Debug build
debug: CFLAGS += -g -DDEBUG
debug: all
