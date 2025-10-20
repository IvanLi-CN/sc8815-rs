## Default goal
.DEFAULT_GOAL := help

SHELL := /bin/bash

ROOT_DIR := $(abspath $(CURDIR))
EX_DIR := examples
SELECT_PROBE_SCRIPT := $(ROOT_DIR)/scripts/select-probe.sh

EXAMPLES := stm32g0-charging stm32g0-otg stm32g0-sc8815-sw2303 stm32g4

.PHONY: help select-probe

help:
	@echo 'Usage:'
	@echo '  1. eval "$$(make -s select-probe)"'
	@echo '  2. make <target>'
	@echo
	@echo 'Targets by example (release build + probe-rs tooling):'
	@for ex in $(EXAMPLES); do \
		echo "  $$ex: help run build attach reset reset-attach select-probe"; \
	done
	@echo
	@echo 'Shortcuts:'
	@echo '  run-<example>            -> make -C examples/<example> run'
	@echo '  build-<example>          -> make -C examples/<example> build'
	@echo '  attach-<example>         -> make -C examples/<example> attach'
	@echo '  reset-<example>          -> make -C examples/<example> reset'
	@echo '  reset-attach-<example>   -> make -C examples/<example> reset-attach'
	@echo '  help-<example>           -> make -C examples/<example> help'
	@echo
	@echo 'Examples:'
	@echo '  make run-stm32g0-otg'
	@echo '  make -C examples/stm32g0-otg help'
	@echo '  eval "$$(make -s select-probe)"'

select-probe:
	@$(SELECT_PROBE_SCRIPT)

define MAKE_DELEGATES
.PHONY: run-$(1) build-$(1) attach-$(1) reset-$(1) reset-attach-$(1) help-$(1)

run-$(1):
	$(MAKE) -C $(EX_DIR)/$(1) run

build-$(1):
	$(MAKE) -C $(EX_DIR)/$(1) build

attach-$(1):
	$(MAKE) -C $(EX_DIR)/$(1) attach

reset-$(1):
	$(MAKE) -C $(EX_DIR)/$(1) reset

reset-attach-$(1):
	$(MAKE) -C $(EX_DIR)/$(1) reset-attach

help-$(1):
	$(MAKE) -C $(EX_DIR)/$(1) help

endef

$(foreach ex,$(EXAMPLES),$(eval $(call MAKE_DELEGATES,$(ex))))
