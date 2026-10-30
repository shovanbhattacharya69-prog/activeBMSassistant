CC      ?= gcc
CFLAGS  := -Wall -Wextra -Werror -pedantic -std=c99 -O2

ifeq ($(OS),Windows_NT)
    TARGET      := bms_simulator.exe
    TEST_TARGET := test_runner.exe
    RM          := del /Q /F
else
    TARGET      := bms_simulator
    TEST_TARGET := test_runner
    RM          := rm -f
endif

CORE_SRCS   := bms_core.c
SIM_SRCS    := bms_core.c hal_sensors.c hal_display.c main.c
TEST_SRCS   := bms_core.c test_bms_core.c
HEADERS     := bms_types.h bms_core.h hal_sensors.h hal_display.h

.PHONY: all clean run run-live test

all: $(TARGET) $(TEST_TARGET)

$(TARGET): $(SIM_SRCS) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $(SIM_SRCS)

$(TEST_TARGET): $(TEST_SRCS) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $(TEST_SRCS)

test: $(TEST_TARGET)
	./$(TEST_TARGET)

run: $(TARGET)
	./$(TARGET)

run-live: $(TARGET)
	./$(TARGET) --live

clean:
ifeq ($(OS),Windows_NT)
	-$(RM) *.o $(TARGET) $(TEST_TARGET) 2>NUL || true
else
	$(RM) *.o $(TARGET) $(TEST_TARGET)
endif
