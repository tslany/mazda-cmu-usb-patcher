SHELL := /bin/sh
VERSION ?= 1.0

.PHONY: all
all:
	sh -n installer/install.sh
	python3 tools/build_usb.py '$(VERSION)'
