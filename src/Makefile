SHELL := /bin/sh

M3TOOLCHAIN ?= $(CURDIR)/../m3-toolchain

TRIPLE := arm-cortexa9_neon-linux-gnueabi
CXX := $(M3TOOLCHAIN)/bin/$(TRIPLE)-g++
SYSROOT := $(M3TOOLCHAIN)/$(TRIPLE)/sysroot

PACKAGE_PAYLOAD_DIR := ../packages/projection-guidance/root/data_persist/mazda-cmu-usb-patcher

LIB_AA := $(PACKAGE_PAYLOAD_DIR)/libpatch-blmjciaapa.so
LIB_AA_SERVICE := $(PACKAGE_PAYLOAD_DIR)/libpatch-aap_service.so
LIB_CARPLAY := $(PACKAGE_PAYLOAD_DIR)/libpatch-blmjcicarplay.so
LIB_NAVI := $(PACKAGE_PAYLOAD_DIR)/libpatch-svcjcinavi.so
LIBRARIES := $(LIB_AA) $(LIB_AA_SERVICE) $(LIB_CARPLAY) $(LIB_NAVI)

GUIDANCE_CLIENT_SOURCE := projection/common/guidance/client.cpp
GUIDANCE_BROKER_SOURCES := \
	projection/common/guidance/broker.cpp \
	projection/common/guidance/hud_sink.cpp

AA_SOURCES := $(shell find projection/blmjciaapa -type f -name '*.cpp' | sort) \
	$(GUIDANCE_CLIENT_SOURCE) $(GUIDANCE_BROKER_SOURCES)
AA_SERVICE_SOURCES := $(shell find projection/aap_service -type f -name '*.cpp' | sort)
CARPLAY_SOURCES := $(shell find projection/blmjcicarplay -type f -name '*.cpp' | sort) \
	$(GUIDANCE_CLIENT_SOURCE)
NAVI_SOURCES := $(shell find projection/svcjcinavi -type f -name '*.cpp' | sort)
PROJECTION_INPUTS := $(shell find projection -type f | sort)

CXXFLAGS := -std=c++11 --sysroot=$(SYSROOT) \
	-march=armv7-a -mtune=cortex-a9 -mfpu=neon \
	-fPIC -fvisibility=hidden -D_GLIBCXX_USE_CXX11_ABI=0 \
	-Iprojection \
	-O3 -DNDEBUG -ffunction-sections -fdata-sections -flto \
	-Wall -Wextra -Werror
LDFLAGS := --sysroot=$(SYSROOT) -shared -Wl,--no-undefined \
	-O3 -flto -Wl,--gc-sections -Wl,-s -lpthread -ldl

.PHONY: all libraries

all: libraries

libraries: $(LIBRARIES)

$(LIB_AA): $(AA_SOURCES) $(PROJECTION_INPUTS) Makefile
	@mkdir -p "$(@D)"
	$(CXX) $(CXXFLAGS) -DMAZDA_CMU_GUIDANCE_LOCAL_BROKER=1 \
		$(AA_SOURCES) $(LDFLAGS) \
		-Wl,-soname,$(@F) -o "$@"
	chmod 0644 "$@"

$(LIB_AA_SERVICE): $(AA_SERVICE_SOURCES) $(PROJECTION_INPUTS) Makefile
	@mkdir -p "$(@D)"
	$(CXX) $(CXXFLAGS) $(AA_SERVICE_SOURCES) $(LDFLAGS) \
		-Wl,-soname,$(@F) -o "$@"
	chmod 0644 "$@"

$(LIB_CARPLAY): $(CARPLAY_SOURCES) $(PROJECTION_INPUTS) Makefile
	@mkdir -p "$(@D)"
	$(CXX) $(CXXFLAGS) $(CARPLAY_SOURCES) $(LDFLAGS) \
		-Wl,-soname,$(@F) -o "$@"
	chmod 0644 "$@"

$(LIB_NAVI): $(NAVI_SOURCES) $(PROJECTION_INPUTS) Makefile
	@mkdir -p "$(@D)"
	$(CXX) $(CXXFLAGS) $(NAVI_SOURCES) $(LDFLAGS) \
		-Wl,-soname,$(@F) -o "$@"
	chmod 0644 "$@"
