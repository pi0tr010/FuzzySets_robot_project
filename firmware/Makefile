CC ?= cc
CFLAGS ?= -std=c99 -O2 -g -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude

AVR_CC ?= avr-gcc
AVR_SIZE ?= avr-size
AVR_FLAGS = -std=c99 -mmcu=atmega328p -DF_CPU=16000000UL -Os -Wall -Wextra -Iinclude

BUILD := build
SRC := $(wildcard src/*.c)
HDR := $(wildcard include/*.h)
SKETCH := $(BUILD)/arduino/robot

.PHONY: all sim avr-check arduino clean

all: sim

sim: $(BUILD)/robot_sim

$(BUILD)/robot_sim: $(SRC) hal/sim/hal_sim.c $(HDR) | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -D_POSIX_C_SOURCE=200809L -o $@ $(SRC) hal/sim/hal_sim.c -lm

# Compiles robot code (without HAL) for the Uno to catch AVR problems early
# and shows static RAM use (data + bss of 2048 bytes).
avr-check: $(SRC) $(HDR) | $(BUILD)
	@mkdir -p $(BUILD)/avr
	@for f in $(SRC); do \
		echo "avr-gcc $$f"; \
		$(AVR_CC) $(AVR_FLAGS) -c $$f -o $(BUILD)/avr/$$(basename $$f .c).o || exit 1; \
	done
	$(AVR_SIZE) $(BUILD)/avr/*.o

arduino: $(SRC) $(HDR)
	@rm -rf $(SKETCH)
	@mkdir -p $(SKETCH)/src
	cp hal/arduino/robot.ino $(SKETCH)/
	cp $(SRC) $(HDR) hal/arduino/hal_arduino.cpp hal/arduino/hal_arduino_pins.h $(SKETCH)/src/
	@echo "Sketch ready: $(SKETCH)/robot.ino"

$(BUILD):
	@mkdir -p $@

clean:
	rm -rf $(BUILD)
