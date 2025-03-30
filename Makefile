# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2024 RoboMaster.                                        #
#  Illini RoboMaster @ University of Illinois at Urbana-Champaign.       #
#                                                                        #
#  This program is free software: you can redistribute it and/or modify  #
#  it under the terms of the GNU General Public License as published by  #
#  the Free Software Foundation, either version 3 of the License, or     #
#  (at your option) any later version.                                   #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#  GNU General Public License for more details.                          #
#                                                                        #
#  You should have received a copy of the GNU General Public License     #
#  along with this program. If not, see <http://www.gnu.org/licenses/>.  #
#                                                                        #
# ---------------------------------------------------------------------- #
SRC_DIR := arduino
LIB_DIR := $(SRC_DIR)/libraries
BUILD_DIR := build/$(SRC_DIR)
CONF := $(SRC_DIR)/arduino-cli.yaml


BOARD_DEFAULT := OpenRB-150
PORT_DEFAULT := __dummy_port__

FQBN_OpenRB-150 := OpenRB-150:samd:OpenRB-150
PORT_OpenRB-150 := /dev/serial/by-id/usb-ROBOTIS_OpenRB-150_*
EXTN_OpenRB-150 := bin

FQBN_Micro := arduino:avr:micro
PORT_Micro := /dev/serial/by-id/usb-Arduino_LLC_Arduino_Micro-*
EXTN_Micro := hex

FQBN_Nano := arduino:avr:nano
PORT_Nano := /dev/serial/by-id/usb-1a86_USB_Serial-*
EXTN_Nano := hex

FQBN_Mega := arduino:avr:mega
PORT_Mega := /dev/serial/by-id/usb-Arduino_LLC_Arduino_Mega-*
EXTN_Mega := hex

SOURCES := $(shell find $(SRC_DIR)/* -maxdepth 0 \
		   		-type d -not -wholename $(LIB_DIR))
BRDINFO := BOARDINFO
FQBN =
PORT =
EXTN =

CONFIGURE := configure-setup configure-clean configure-resetup configure-update configure--h

ACC = arduino-cli --verbose --config-file $(CONF) \
	   compile --fqbn $(FQBN) --build-path $(BUILD_DIR)
AUP = arduino-cli --verbose --config-file $(CONF) \
	   upload --fqbn $(FQBN) --port $(PORT) --input-file

# foo/examples/bar/* -> foo/bar/*
define STRIP_EXAMPLE
$(shell sed 's%\(^\|\s\)\([^/]\+\)/examples%\1\2%g' <<< "$(1)")
endef
# foo/bar/* -> foo/examples/bar/*
define STRAP_EXAMPLE
$(shell sed 's%\(^\|\s\)\([^/]\+\)%\1\2/examples%g' <<< "$(1)")
endef

define GET_BOARD
$(shell cat $(1)/$(BRDINFO) 2> /dev/null || echo $(BOARD_DEFAULT))
endef

define SET_PORT
$(eval PORT = $(shell
	ls $(PORT_$(call GET_BOARD,$(SRC_DIR)/$(dir $(1)))) >& /dev/null && \
	udevadm info -q property $(PORT_$(call GET_BOARD,$(SRC_DIR)/$(dir $(1)))) | \
		awk -F= '$$1=="DEVNAME"{ print $$2; exit; }' || \
	echo $(PORT_DEFAULT)
	)
)
endef

define SET_EXTN
$(eval EXTN = $(EXTN_$(call GET_BOARD, $(SRC_DIR)/$(dir $(1)))))
endef

define SET_FQBN
$(eval FQBN = $(FQBN_$(call GET_BOARD,$(SRC_DIR)/$(dir $(1)))))
endef

example_sketches = $(call STRIP_EXAMPLE,$(subst $(LIB_DIR)/,, \
				$(shell find $(LIB_DIR) -wholename "*/examples/*.ino" | xargs dirname)))

sketches = $(foreach src,$(SOURCES), \
		   $(shell find $(src)/* -maxdepth 0 -type d | cut -d/ -f2-))

all: $(sketches)

all-example: $(example_sketches:%=example-%)

$(sketches):
	$(call SET_FQBN,$(@))
	$(ACC)/$(@) $(SRC_DIR)/$(@)

$(sketches:%=upload-%):
	$(call SET_FQBN,$(@:upload-%=%))
	$(call SET_PORT,$(@:upload-%=%))
	$(call SET_EXTN,$(@:upload-%=%))
	$(AUP) $(BUILD_DIR)/$(@:upload-%=%)/$(notdir $(@)).ino.$(EXTN)

$(sketches:%=compile-upload-%):
	$(MAKE) $(@:compile-upload-%=%)
	$(MAKE) $(@:compile-%=%)

$(example_sketches:%=example-%):
	$(call SET_FQBN,$(LIB_DIR)/$(dir $(1:example-%=%))/$(BRDINFO))
	$(ACC)/$(call STRAP_EXAMPLE,$(@:example-%=%)) \
		$(LIB_DIR)/$(call STRAP_EXAMPLE,$(@:example-%=%))

$(example_sketches:%=example-upload-%):
	$(call SET_FQBN,$(LIB_DIR)/$(dir $(@:example-upload-%=%))/$(BRDINFO))
	$(call SET_PORT,$(LIB_DIR)/$(dir $(@:example-upload-%=%))/$(BRDINFO))
	$(call SET_EXTN,$(LIB_DIR)/$(dir $(@:example-upload-%=%))/$(BRDINFO))
	$(AUP) $(BUILD_DIR)/$(call STRAP_EXAMPLE,$(@:example-upload-%=%))/$(notdir $(@)).ino.$(EXTN)

$(example_sketches:%=example-compile-upload-%):
	$(MAKE) example-$(@:example-compile-upload-%=%)
	$(MAKE) example-$(@:example-compile-%=%)

.PHONY: clean configure $(ARDUINOCONF)
clean:
	rm -rf $(BUILD_DIR)/*

configure: configure-setup

$(CONFIGURE):
	@./arduino_configuration.sh $(@:configure-%=%)
