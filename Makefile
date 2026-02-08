# SCARA Robot Kinematics & Control - Makefile (Windows-friendly)

CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -I./include -O2
LDFLAGS = -lm -lws2_32

# Directories
SRC_DIR = src
INC_DIR = include
BUILD_DIR = build
TEST_DIR = tests

# Windows executable extension
EXE = .exe

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

# Main executable (exclude main.c for tests)
MAIN_OBJ = $(BUILD_DIR)/main.o
LIB_OBJS = $(filter-out $(MAIN_OBJ),$(OBJS))

# Test files
TEST_SRCS = $(wildcard $(TEST_DIR)/test_*.c)
TEST_BINS = $(patsubst $(TEST_DIR)/%.c,$(BUILD_DIR)/%$(EXE),$(TEST_SRCS))

# Unity test framework
UNITY_SRC = $(TEST_DIR)/unity/unity.c
UNITY_OBJ = $(BUILD_DIR)/unity.o

# Targets
TARGET = $(BUILD_DIR)/scara_sim$(EXE)

.PHONY: all clean test dirs debug \
        test_math test_dh test_fk test_ik test_pid test_traj test_int

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
	@for %%t in ($(subst /,\,$(TEST_BINS))) do call %%t

# Build each test binary as .exe
$(BUILD_DIR)/test_%$(EXE): $(TEST_DIR)/test_%.c $(UNITY_OBJ) $(LIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Individual test runners (now .exe)
test_math: dirs $(BUILD_DIR)/test_math_utils$(EXE)
	@call $(subst /,\,$(BUILD_DIR)/test_math_utils$(EXE))

test_dh: dirs $(BUILD_DIR)/test_dh_transform$(EXE)
	@call $(subst /,\,$(BUILD_DIR)/test_dh_transform$(EXE))

test_fk: dirs $(BUILD_DIR)/test_forward_kinematics$(EXE)
	@call $(subst /,\,$(BUILD_DIR)/test_forward_kinematics$(EXE))

test_ik: dirs $(BUILD_DIR)/test_inverse_kinematics$(EXE)
	@call $(subst /,\,$(BUILD_DIR)/test_inverse_kinematics$(EXE))

test_pid: dirs $(BUILD_DIR)/test_pid_controller$(EXE)
	@call $(subst /,\,$(BUILD_DIR)/test_pid_controller$(EXE))

test_traj: dirs $(BUILD_DIR)/test_trajectory$(EXE)
	@call $(subst /,\,$(BUILD_DIR)/test_trajectory$(EXE))

test_int: dirs $(BUILD_DIR)/test_integration$(EXE)
	@call $(subst /,\,$(BUILD_DIR)/test_integration$(EXE))

clean:
	@if exist $(BUILD_DIR) rmdir /s /q $(BUILD_DIR)

# Debug build
debug: CFLAGS += -O0 -g -DDEBUG
debug: all
