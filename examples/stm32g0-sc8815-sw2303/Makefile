SHELL := /bin/bash

CARGO ?= cargo
PROBE_RS ?= probe-rs

ROOT_DIR := $(abspath $(CURDIR)/../..)
TARGET_TRIPLE := thumbv6m-none-eabi
CHIP := STM32G031G8
BINARY := sc8815_sw2303_fb_example
ELF := target/$(TARGET_TRIPLE)/release/$(BINARY)
SELECT_PROBE_SCRIPT := $(ROOT_DIR)/scripts/select-probe.sh

.PHONY: help select-probe build run attach reset reset-attach require-probe

help:
	@echo 'stm32g0-sc8815-sw2303 example targets:'
	@echo '  help           - Show this message'
	@echo '  select-probe   - Interactively choose a debug probe (prints export command)'
	@echo '  build          - cargo build --release --target $(TARGET_TRIPLE)'
	@echo '  run            - Build + probe-rs run --release'
	@echo '  attach         - Build + probe-rs attach'
	@echo '  reset          - probe-rs reset'
	@echo '  reset-attach   - Build + probe-rs reset + attach (software reset)'
	@echo
	@echo 'Before run/attach/reset targets, set PROBE_ID:'
	@echo '  eval "$$(make -s select-probe)"'

select-probe:
	@$(SELECT_PROBE_SCRIPT)

require-probe:
	@if [ -z "$(PROBE_ID)" ]; then \
		echo 'PROBE_ID is not set. Run: eval "$$(make -s select-probe)"' >&2; \
		exit 1; \
	fi

build:
	$(CARGO) build --release --target $(TARGET_TRIPLE)

run: require-probe build
	$(PROBE_RS) run --probe $(PROBE_ID) --chip $(CHIP) $(ELF)

attach: require-probe build
	$(PROBE_RS) attach --probe $(PROBE_ID) --chip $(CHIP) $(ELF)

reset: require-probe
	$(PROBE_RS) reset --probe $(PROBE_ID) --chip $(CHIP)

reset-attach: require-probe build
	$(PROBE_RS) reset --probe $(PROBE_ID) --chip $(CHIP)
	$(PROBE_RS) attach --probe $(PROBE_ID) --chip $(CHIP) $(ELF)
