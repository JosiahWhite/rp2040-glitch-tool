# Copyright (c) 2022 Nicolai Electronics
# SPDX-License-Identifier: MIT

INSTALL_PREFIX := $PWD
BUILD_DIR := build
GENERATED_DIR := generated

.PHONY: all build debug install clean install_rules format

all: build
	@echo "All tasks completed"

build:
	mkdir -p $(BUILD_DIR)
	mkdir -p $(GENERATED_DIR)
	cd $(BUILD_DIR); cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DCMAKE_BUILD_TYPE=Release ..
	$(MAKE) -C $(BUILD_DIR) --no-print-directory all

debug:
	mkdir -p $(BUILD_DIR)
	mkdir -p $(GENERATED_DIR)
	cd $(BUILD_DIR); cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DCMAKE_BUILD_TYPE=Debug ..
	$(MAKE) -C $(BUILD_DIR) --no-print-directory all

install: build
	picotool load $(BUILD_DIR)/rp2040_glitch.bin
	picotool reboot

clean:
	rm -rf $(BUILD_DIR)
	rm -rf $(GENERATED_DIR)

install_rules:
	install -m 0644 tools/70-pico.rules /etc/udev/rules.d/70-pico.rules
	rm -f /etc/udev/rules.d/99-pico.rules
	udevadm control --reload-rules
	@echo "Reconnect the Pico to apply permissions and /dev/rp2040-glitch-SERIAL."

format:
	find . -iname '*.h' -o -iname '*.c' -o -iname '*.cpp' | grep -v '$(BUILD_DIR)' | xargs clang-format -i
