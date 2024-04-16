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
BUILD_DIR = build/arduino
SRC_DIR = arduino/tele_arm/
BOARD = OpenRB-150:samd:OpenRB-150
CONF = arduino/arduino-cli.yaml
ACC := arduino-cli --config-file $(CONF) compile --fqbn $(BOARD) --build-path $(BUILD_DIR)
AUP := arduino-cli --config-file $(CONF) upload --fqbn $(BOARD) --input-file

paths = $(wildcard arduino/tele_arm/*)
sketches = $(notdir $(paths:%/=%))

configure = configure-setup configure-clean configure-resetup configure-update configure--h

all: $(sketches)

$(sketches):
	mkdir -p $(BUILD_DIR)
	$(ACC) $(SRC_DIR)/$@

upload-$(sketches):
	$(AUP) $(BUILD_DIR)/$(@:upload-%=%).ino.bin

compile-upload-$(sketches):
	$(MAKE) $(@:compile-upload-%=%)
	$(MAKE) $(@:compile-%=%)

.PHONY: clean configure $(arduinoconf)
clean:
	rm -rf $(BUILD_DIR)/*

configure: configure-setup

$(configure):
	@./arduino_configuration.sh $(@:configure-%=%)
