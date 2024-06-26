# irm_tele_arm
Embedded system development @ Illini RoboMaster
## Setup (GNU/Linux with systemd)
Have the following on your system:
- `python3` (Tested on 3.10+)
- `udevadm`
- `systemd` [Recommended]
- `tmux` [Recommended]
- `make`
- `arduino-cli`


Get under your cloned directory.
1. Make sure you have correct serial port access permissions (by group membership or otherwise). For Debian-based systems, this would be the `dialout` group.
- Check your current memberships.
    ```sh
    groups $USER
    ```
    - Debian/Ubuntu
    ```sh
    sudo usermod -aG dialout $USER
    ```
    - Arch
    ```sh
    sudo usermod -aG uucp $USER
    ```
2. Modify the `*_ID_SERIAL(_SHORT)?` variable(s) in `config.py` based on your the serial id(s) of your device(s). To find the id of a device on `/path/to/serial`, run
    ```sh
    udevadm info -q property '/path/to/serial' | awk -F= '/^ID_SERIAL_SHORT/ { print $2 }'
    ```
3. Create a python venv (in this case named `venv`) and install the requirements.
   ```sh
   python3 -m venv venv
   pip3 install -r requirements.txt
   ```
4. [Recommended] If you want `minipc.py` to be able to run in the background for easier debugging/testing, see `minipc-tmux-daemon-helper.sh`. Run it without arguments for help and details. If you don't want to read run
    ```sh
    ./minipc-tmux-daemon-helper.sh -i install
    ```
    for an interactive installation.

5. To set up Arduino compilation/upload via the Makefile, run `make configure#{,-{,re}setup}`.

## Usage
### Python
For options, run
```sh
python3 python/main.py -h
```
Assuming you have set up the tmux daemon in the previous section (read the help first!),
```sh
# Start the service.
# You can omit `.service'.
# You may want to alias `systemd --user' to something shorter.
systemd --user start minipctd.service
```
After starting the service,
```sh
# Connect to the tmux section.
# You definitely want to alias this to something shorter.
tmux -L mtd attach -t mtd
```
More keybinds are detailed in `minipc-tmux-daemon-helper.sh`. `Alt+f d` to detach, `Alt+f z` to kill.
```sh
systemctl --user {restart,stop} minipctd.service
```
do what you expect, and
```sh
systemctl --user reload minipctd.service
```
sends Ctrl+c (`C-c`) to the tmux session, which the job control handles by restarting `main.py` (after a one second delay). This means in the tmux session you can restart `main.py` manually with `C-c`. The one second delay is to wait for possible user input, in which case the user is dropped into a control menu.
### Arduino
Make sure `set-up-arduino.sh` has been run (either manually or with `make configure`) before continuing.
```sh
# Arduino configurtion options
# See `make configure--h' or `./set-up-arduino.sh -h'.
make configure#{,-{setup,clean,resetup,update,-h}}
# Compile every sketch under Arduino/tele_arm/
make
# Compile `arm_only'.
# In general: `make <sketch name>'.
make arm_only
# Upload `arm_only' without recompiling.
# In general: `make upload-<sketch name>'.
make upload-arm_only
# Compile and upload `arm_only'.
# In general: `make compile-upload-<sketch name>'.
make compile-upload-arm_only
```

## Reminders
1. Init
- If you do not use systemd, you may have to find a replacement for `udevadm` and modify `Communication/communicator.py` accordingly.
- If you do not use systemd, the service is trivial (starts, stops and sends `C-c` to a tmux session). Rewriting should be simple. Starting the tmux session manually is also an option. The service file can be found inside the `install` function of `minipc-tmux-daemon-helper.sh`.
2. Examples for the `Dynamixel2Arduino` library are at `arduino/libraries/Dynamixel2Arduino/UNZIP_ME_FOR_EXAMPLES.zip`. How to retrieve them is currently unkown and requires further study.
