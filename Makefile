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
ARDUINO_DIR=arduino
BUILD_DIR = build/$(ARDUINO_DIR)
SRC_DIR = $(ARDUINO_DIR)/tele_arm
LIB_DIR = $(ARDUINO_DIR)/libraries
BOARD = OpenRB-150:samd:OpenRB-150
CONF = $(ARDUINO_DIR)/arduino-cli.yaml
ACC := arduino-cli --verbose --config-file $(CONF) compile --fqbn $(BOARD) --build-path $(BUILD_DIR)
AUP := arduino-cli --verbose --config-file $(CONF) upload --fqbn $(BOARD) --input-file

paths = $(wildcard $(SRC_DIR)/*)
sketches = $(notdir $(paths:%/=%))

# foo/examples/bar/* -> foo/bar/*
func_strip_example = $(shell sed 's%\(^\|\s\)\([^/]\+\)/examples%\1\2%g' <<< "$(1)")
# foo/bar/* -> foo/examples/bar/*
func_strap_example = $(shell sed 's%\(^\|\s\)\([^/]\+\)%\1\2/examples%g' <<< "$(1)")
example_paths = $(shell find $(LIB_DIR) -wholename "*/examples/*.ino" | xargs dirname)
examples = $(call func_strip_example,$(example_paths:$(LIB_DIR)/%=%))

configure = configure-setup configure-clean configure-resetup configure-update configure--h

all: $(sketches)

$(sketches):
	mkdir -p $(BUILD_DIR)
	$(ACC)/$(@) $(SRC_DIR)/$(@)

$(sketches:%=upload-%):
	$(AUP) $(BUILD_DIR)/$(@:upload-%=%)/$(@:upload-%=%).ino.bin

$(sketches:%=compile-upload-%):
	$(MAKE) $(@:compile-upload-%=%)
	$(MAKE) $(@:compile-%=%)

example-all: $(examples:%=example-%)

$(examples:%=example-%):
	$(ACC)/$(call func_strap_example,$(@:example-%=%)) $(LIB_DIR)/$(call func_strap_example,$(@:example-%=%))

$(examples:%=example-upload-%):
	$(AUP) $(BUILD_DIR)/$(call func_strap_example,$(@:example-upload-%=%))/$(notdir $(@)).ino.bin

$(examples:%=example-compile-upload-%):
	$(MAKE) example-$(@:example-compile-upload-%=%)
	$(MAKE) example-$(@:example-compile-%=%)

.PHONY: clean configure $(arduinoconf)
clean:
	rm -rf $(BUILD_DIR)/*

configure: configure-setup

$(configure):
	@./arduino_configuration.sh $(@:configure-%=%)
