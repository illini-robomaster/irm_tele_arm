#!/usr/bin/env python3
#
# Host the UART communicator. See UARTCommunicator.
#
import os
import sys
import crc
import time
import struct
import serial
import logging
import functools
import threading
from copy import deepcopy
from enum import Enum
from typing import Optional, Union
from subprocess import run, PIPE

# Unit testing
if __name__ == '__main__':
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import config

from communication.tests import *

from thirdparty import dynamixel_sdk
from thirdparty.dynamixel.driver import DynamixelDriver

logger = logging.getLogger(__name__)

# STM32 to Jetson packet size
STJ_MAX_PACKET_SIZE = 33
STJ_MIN_PACKET_SIZE = 10

class MiniPCCommunicationError(Exception):
    pass


class StateDict:

    def __init__(self, **kwargs):
        self.dict = dict(kwargs)
        self.lock = threading.Lock()

    def __getitem__(self, key):
        with self.lock:
            return self.dict[key]

    def __delitem__(self, key):
        with self.lock:
            del self.dict[key]

    def keys(self):
        with self.lock:
            return self.dict.keys()

    def values(self):
        with self.lock:
            return self.dict.values()

    def items(self):
        with self.lock:
            return self.dict.items

    def copy(self):
        """Returns a regular dict"""
        with self.lock:
            return self.dict.copy()

    def deepcopy(self):
        """Returns a regular dict"""
        with self.lock:
            return deepcopy(self.dict)

    def setdefault(self, key, value):
        with self.lock:
            self.dict[key] = value

    def update(self, other):
        with self.lock:
            self.dict |= other

    # Update only the keys already existing
    def specific_update(self, other):
        with self.lock:
            # [s]elf [k]eys, [o]ther [k]eys
            sk_set = set(self.dict.keys())
            ok_set = set(other.keys())
            extra_keys = ok_set - (sk_set & ok_set)
            self.dict |= other
            for k in extra_keys:
                del self.dict[k]


class Communicator:

    def __init__(self):
        pass

    def start_listening(self) -> None:
        pass

    def _listen(self, hz=200):
        pass

    def is_valid(self) -> bool:
        pass

    def is_vacuum(self) -> bool:
        pass

    def is_alive(self) -> Optional[bool]:
        """True or raise MiniPCCommunicationError"""
        pass

    def get_port(self) -> Optional['path']:
        pass

    def create_and_send_packet(self, cmd_id, data) -> None:
        pass

    def create_packet(self, cmd_id, data) -> Union[bytes, dict]:
        pass

    def send_packet(self, packet) -> None:
        pass

    def read_out(self) -> dict:
        pass


class UARTCommunicator(Communicator):
    """USB-TTL-UART communicator for Jetson-STM32 communication."""

    def __init__(
            self,
            config,
            crc_standard=crc.Crc8.MAXIM_DOW,
            endianness='little',
            warn=True,
            serial_dev_path=None,  # None -> guess port, False -> port=None
            serial_dev=None,       # None -> ^^^^^^^^^^, False -> ^^^^^^^^^
            allow_portless=True,
            in_use=None,
            buffer_size=STJ_MAX_PACKET_SIZE * 100):
        """Initialize the UART communicator.

        Args:
            config (python object): shared config
            crc_standard (crc.Crc8): CRC standard
            endianness (str): endianness of the packet. Either 'big' or 'little'
            buffer_size (int): size of the circular buffer
        """
        self.config = config
        self.crc_standard = crc_standard
        self.endianness = endianness

        self.crc_calculator = crc.Calculator(self.crc_standard, optimized=True)

        self.warn = warn

        if in_use is None:
            if self.warn:
                logger.warning('Did not receive a list of ports in use, assuming none.')
            in_use = []

        if serial_dev_path is None:
            if serial_dev is None:
                self.use_uart_device(self.guess_uart_device(in_use), in_use)
            elif not serial_dev:
                self.use_uart_device(None, in_use)
            else:
                self.use_uart_device(serial_dev, in_use)
        elif not serial_dev_path:
            self.use_uart_device_path(None, in_use)
        else:
            self.use_uart_device_path(serial_dev_path, in_use)

        if not allow_portless and not self.is_vacuum():
            raise serial.serialutil.SerialException

        self.circular_buffer = []
        self.buffer_size = buffer_size

        self.stm32_state = StateDict(**{
            'my_color': 'red' if self.config.DEFAULT_ENEMY_TEAM == 'blue' else 'blue',
            'enemy_color': self.config.DEFAULT_ENEMY_TEAM.lower(),
            'rel_yaw': 0,
            'rel_pitch': 0,
            'debug_int': 0,
            'mode': 'ST',
            'vx': 0,
            'vy': 0,
            'vw': 0,
            'floats': {
                'float0': 0.0,
                'float1': 0.0,
                'float2': 0.0,
                'float3': 0.0,
                'float4': 0.0,
                'float5': 0.0,
            }
        })

        self.parsed_packet_cnt = 0
        self.seq_num = 0

    def start_listening(self) -> None:
        """Start a thread to listen to the serial port."""
        self.listen_thread = threading.Thread(target=self._listen)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def _listen(self, hz=200) -> None:
        """
        Listen to the serial port.

        This function updates circular_buffer and stm32_state.dict.

        TODO: test this function on real jetson / STM32!

        Args:
            interval (float): interval between two reads
        """
        while True:
            self.try_read_one()
            self.packet_search()
            time.sleep(1/hz)

    def is_valid(self) -> bool:
        """Return if communicator is valid."""
        return self.serial_port is not None

    def is_vacuum(self) -> bool:
        return not (self.is_valid() and bool(self.serial_port.port))

    def is_alive(self) -> Optional[bool]:
        port = self.get_port()
        try:
            self.serial_port.inWaiting()
        except Exception as e:
            raise MiniPCCommunicationError from e
        return True

    def get_port(self) -> Optional['path']:
        if not self.is_vacuum():
            return self.serial_port.port

    def try_read_one(self) -> bool:
        """Try to copy from serial port to a circular buffer.

        Returns:
            bool: True if there are data waiting in the serial port
        """
        try:
            self.is_alive()
        except Exception:
            return False
        # Read from serial port, if any packet is waiting
        if self.serial_port is not None:
            if self.serial_port.inWaiting() > 0:
                # read the bytes and convert from binary array to ASCII
                byte_array = self.serial_port.read(self.serial_port.inWaiting())

                for c in byte_array:
                    if len(self.circular_buffer) >= self.buffer_size:
                        # pop first element
                        self.circular_buffer = self.circular_buffer[1:]
                    self.circular_buffer.append(c)
                return True
            else:
                return False

    def create_and_send_packet(self, cmd_id, data) -> None:
        """Process a batch of numbers into a CRC-checked packet and send it out.

        Args:
            cmd_id (int): see config.py
            data (dict): all key and values are defined in the docs/comm_protocol.md
            Here's a list of cmd_id's and their data for quick reference
            For a more detailed description, see docs/comm_protocols.md
            cmd_id == GIMBAL_CMD_ID:
            data = {'rel_yaw': float, 'rel_pitch': float, 'mode': 'ST' or 'MY',
                    'debug_int': uint8_t}

            cmd_id == COLOR_CMD_ID:
              data = {'my_color': 'red' or 'blue', 'enemy_color': same as my_color}

            cmd_id == CHASSIS_CMD_ID:
              data = {'vx': float, 'vy': float, 'vw': float}
        """
        packet = self.create_packet(cmd_id, data)
        self.send_packet(packet)

    def send_packet(self, packet) -> None:
        """Send a packet out."""
        if self.serial_port is not None:
            self.serial_port.write(packet)

    def use_uart_device_path(self, dev_path, in_use) -> None:
        if dev_path in in_use:
            logger.warning('{dev_path} already in use: is this really expected?')
        dev = UARTCommunicator.try_uart_device(dev_path, in_use)
        if dev is None:
            if self.warn:
                logger.warning("NO SERIAL DEVICE FOUND! I/O TO VACUUM!")
        else:
            in_use += [dev.port]
        logger.debug(f'I ({self.__class__=}) am using {dev}.')
        self.serial_port = dev

    def use_uart_device(self, dev, in_use) -> None:
        try:
            if dev.port in in_use:
                logger.warning('{dev.port} already in use: is this really expected?')
            else:
                in_use += [dev.port]
        except AttributeError:  # dev is None
            logger.warning("NO SERIAL DEVICE FOUND! I/O TO VACUUM!")
        finally:
            logger.debug(f'I ({self.__class__=}) am using {dev}.')
            self.serial_port = dev

    @staticmethod
    def list_uart_device_paths() -> [Optional['path']]:
        """Guess the UART device paths and return them.

        Note: this function is for UNIX-like systems only!

        OSX prefix: "tty.usbmodem"
        Jetson / Linux prefix: "ttyUSB", "ttyACM"
        Linux: look under "/dev/serial/by-id" for "usb-STMicroelectronics_STM32_STLink_"

        Returns:
            [Maybe dev_paths] : a list of possible device paths
        """
        # list of possible prefixes
        UART_PREFIX_LIST = ('usb-STMicroelectronics_STM32_STLink_',)
        dev_basename = '/dev/serial/by-id'
        try:
            path_list = os.listdir(dev_basename)
        except FileNotFoundError:  # Nothing connected
            path_list = []

        dev_paths = [os.path.join(dev_basename, path)
                     for path in path_list
                     if path.startswith(UART_PREFIX_LIST)]
        return dev_paths or [None]

    # path -> [path] -> Maybe serial.Serial
    @staticmethod
    def try_uart_device(dev_path, in_use) -> Optional[serial.Serial]:
        if dev_path in in_use:
            logger.error(f'Path {dev_path} already in use, returning None.')
            return None
        # Fails with serial.serialutil.SerialException
        serial_port = serial.Serial(
            port=dev_path,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        if serial_port.port is not None:
            logger.debug(f'Successfully opened serial on path: {dev_path}')
            return serial_port
        else:
            logger.debug(f'Failed to open serial on path: {dev_path}, '
                         'returning None object instead.')
            return None

    # [path] -> Maybe serial.Serial
    @staticmethod
    def guess_uart_device(in_use) -> Optional[serial.Serial]:
        """Guess the UART device path and open it.

        Note: this function is for UNIX-like systems only!

        OSX prefix: "tty.usbmodem"
        Jetson / Linux prefix: "ttyUSB", "ttyACM"

        Returns:
            serial.Serial: the serial port object
        """
        logger.info('I will now try to guess a uart device.')
        # list of possible prefixes
        dev_paths = UARTCommunicator.list_uart_device_paths()

        serial_port = None  # ret val
        for dev_path in dev_paths:
            logger.info(f'Guessed {dev_path}.')
            if dev_path in in_use:
                logger.info(f'Giving up as it is already in use.')
                continue
            if dev_path is not None:
                try:
                    serial_port = UARTCommunicator.try_uart_device(dev_path, in_use)
                    if serial_port is not None:
                        return serial_port
                except serial.serialutil.SerialException:
                    print('Could not open serial port, skipping...')
        return serial_port

    def packet_search(self) -> bool:
        """Parse internal circular buffer.

        Returns: True if a valid packet is found
        """
        start_idx = 0
        packet_found = False
        while start_idx <= len(self.circular_buffer) - STJ_MAX_PACKET_SIZE:
            header_letters = (
                self.circular_buffer[start_idx], self.circular_buffer[start_idx + 1])
            if header_letters == (self.config.PACK_START[0], self.config.PACK_START[1]):
                # Try to parse
                possible_packet = self.circular_buffer[start_idx:start_idx + STJ_MAX_PACKET_SIZE]
                ret_dict = self.try_parse_one(possible_packet)
                if ret_dict is not None:
                    # Successfully parsed one
                    self.parsed_packet_cnt += 1
                    self.update_current_state(ret_dict)
                    # Remove parsed bytes from the circular buffer
                    self.circular_buffer = self.circular_buffer[start_idx + (
                        self.config.CMD_TO_LEN[ret_dict['cmd_id']] + self.config.HT_LEN):]
                    packet_found = True
                else:
                    self.circular_buffer = self.circular_buffer[start_idx + STJ_MIN_PACKET_SIZE:]
                start_idx = 0
            else:
                start_idx += 1
        return packet_found

    def update_current_state(self, ret_dict) -> None:
        """
        Update stm32 state dict.

        Helper function.
        """
        # Dont do self.stm32_state.dict = ret_dict['data'] because different
        # threads may need different information from the stm32
        self.stm32_state.specific_update(ret_dict['data'])

    def try_parse_one(self, possible_packet) -> Optional[dict]:
        """
        Parse a possible packet.

        For details on the struct of the packet, refer to docs/comm_protocol.md

        Args:
            possible_packet (list): a list of bytes

        Returns:
            dict: a dictionary of parsed data; None if parsing failed
        """
        assert len(possible_packet) >= STJ_MIN_PACKET_SIZE
        assert possible_packet[0] == self.config.PACK_START[0]
        assert possible_packet[1] == self.config.PACK_START[1]

        # Check CMD ID valid
        cmd_id = int(possible_packet[self.config.CMD_ID_OFFSET])
        try:
            packet_len = self.config.CMD_TO_LEN[cmd_id] + self.config.HT_LEN
        except Exception:
            print("Incorrect CMD_ID " + str(cmd_id))
            return None

        # Check packet end
        if possible_packet[packet_len -
                           2] != self.config.PACK_END[0] or possible_packet[packet_len -
                                                                         1] != self.config.PACK_END[1]:
            return None

        # Compute checksum
        crc_checksum = self.crc_calculator.checksum(bytes(possible_packet[:packet_len - 3]))
        if crc_checksum != possible_packet[packet_len - 3]:
            print("Packet received but crc checksum is wrong")
            return None

        # Parse data into a dictionary
        data = self.parse_data(possible_packet, cmd_id)

        return {
            'cmd_id': cmd_id,
            'data': data
        }

    def parse_data(self, possible_packet, cmd_id) -> Optional[dict]:
        """
        Parse the data section of a possible packet.

        Helper function for details on the struct of the packet, refer to docs/comm_protocol.md

        Args:
            data (map): keys are value types are defined in docs/comm_protocol
        Returns:
            dict: a dictionary of parsed data; None if parsing failed
        """
        data = None
        # Parse Gimbal data, CMD_ID = 0x00
        if cmd_id == self.config.GIMBAL_CMD_ID:
            # "<f" means little endian float
            rel_yaw = struct.unpack('<f', bytes(
                possible_packet[self.config.DATA_OFFSET + 0: self.config.DATA_OFFSET + 4]))[0]
            rel_pitch = struct.unpack('<f', bytes(
                possible_packet[self.config.DATA_OFFSET + 4: self.config.DATA_OFFSET + 8]))[0]
            mode_int = int(possible_packet[self.config.DATA_OFFSET + 8])
            mode = self.config.GIMBAL_MODE[mode_int]
            debug_int = int(possible_packet[self.config.DATA_OFFSET + 9])
            data = {
                'rel_yaw': rel_yaw,
                'rel_pitch': rel_pitch,
                'mode': mode,
                'debug_int': debug_int}
        # Parse Chassis data, CMD_ID = 0x02
        elif cmd_id == self.config.CHASSIS_CMD_ID:
            vx = struct.unpack('<f', bytes(
                possible_packet[self.config.DATA_OFFSET + 0: self.config.DATA_OFFSET + 4]))[0]
            vy = struct.unpack('<f', bytes(
                possible_packet[self.config.DATA_OFFSET + 4: self.config.DATA_OFFSET + 8]))[0]
            vw = struct.unpack('<f', bytes(
                possible_packet[self.config.DATA_OFFSET + 8: self.config.DATA_OFFSET + 12]))[0]
            data = {'vx': vx, 'vy': vy, 'vw': vw}
        # Parse color data, CMD_ID = 0x01
        elif cmd_id == self.config.COLOR_CMD_ID:
            # 0 for RED; 1 for BLUE
            my_color_int = int(possible_packet[self.config.DATA_OFFSET])
            if my_color_int == 0:
                my_color = 'red'
                enemy_color = 'blue'
            else:
                my_color = 'blue'
                enemy_color = 'red'
            data = {'my_color': my_color, 'enemy_color': enemy_color}
        # Parse Selfcheck data, CMD_ID = 0x03
        if cmd_id == self.config.SELFCHECK_CMD_ID:
            mode_int = int(possible_packet[self.config.DATA_OFFSET + 0])
            mode = self.config.SELFCHECK_MODE[mode_int]
            debug_int = int(possible_packet[self.config.DATA_OFFSET + 1])
            data = {
                'mode': mode,
                'debug_int': debug_int}
        # Parse Arm data, CMD_ID = 0x04
        if cmd_id == self.config.ARM_CMD_ID:
            # "<f" means little endian float
            floats = {
                'float0': 0.0,
                'float1': 0.0,
                'float2': 0.0,
                'float3': 0.0,
                'float4': 0.0,
                'float5': 0.0,
            }
            for i, k in enumerate(floats.keys()):
                floats[k] = struct.unpack('<f', bytes(
                    possible_packet[self.config.DATA_OFFSET + 4 * i:
                                    self.config.DATA_OFFSET + 4 * (i + 1)]))[0]
            data = {
                'floats': floats,
            }
        return data

    def create_packet(self, cmd_id, data) -> bytes:
        """
        Create CRC-checked packet from user input.

        Args:
            cmd_id (int): see config.py
            data (dict): all key and values are defined in the docs/comm_protocol.md
        Returns:
            bytes: the packet to be sent to serial

        For more details, see doc/comms_protocol.md, but here is a summary of the packet format:
        Little endian

        HEADER    (2 bytes chars)
        SEQNUM    (2 bytes uint16; wrap around)
        DATA_LEN  (2 bytes uint16)
        CMD_ID    (1 byte  uint8)
        DATA      (see docs/comms_protocol.md for details)
        CRC8      (1 byte  uint8; CRC checksum MAXIM_DOW of contents BEFORE CRC)
                  (i.e., CRC does not include itself and PACK_END!)
        PACK_END  (2 bytes chars)
        """
        # Prepare header
        packet = self.config.PACK_START
        assert isinstance(self.seq_num, int) and self.seq_num >= 0
        if self.seq_num >= 2 ** 16:
            self.seq_num = self.seq_num % (2 ** 16)
        packet += (self.seq_num & 0xFFFF).to_bytes(2, self.endianness)
        packet += self.config.CMD_TO_LEN[cmd_id].to_bytes(1, self.endianness)
        packet += cmd_id.to_bytes(1, self.endianness)

        # Prepare data
        packet += self.create_packet_data(cmd_id, data)

        # Compute CRC
        crc8_checksum = self.crc_calculator.checksum(packet)
        assert crc8_checksum >= 0 and crc8_checksum < 256

        packet += crc8_checksum.to_bytes(1, self.endianness)

        # ENDING
        packet += self.config.PACK_END

        self.seq_num += 1
        return packet

    def create_packet_data(self, cmd_id, data) -> bytes:
        """
        Create the data section for a packet.

        Helper function.
        Args:
            cmd_id (int): see config.py
            data (dict): all key and values are defined in the docs/comm_protocol.md
        Returns:
            bytes: the data section of the packet to be sent to serial
        """
        # empty binary string
        packet = b''
        # Parse Gimbal data, CMD_ID = 0x00
        if cmd_id == self.config.GIMBAL_CMD_ID:
            # "<f" means little endian float
            packet += struct.pack("<f", data['rel_yaw'])
            packet += struct.pack("<f", data['rel_pitch'])
            # 0 for 'ST' 1 for 'MY',
            packet += self.config.GIMBAL_MODE.index(data['mode']).to_bytes(1, self.endianness)
            packet += data['debug_int'].to_bytes(1, self.endianness)
        # Parse Chassis data, CMD_ID = 0x02
        elif cmd_id == self.config.CHASSIS_CMD_ID:
            # "<f" means little endian float
            packet += struct.pack("<f", data['vx'])
            packet += struct.pack("<f", data['vy'])
            packet += struct.pack("<f", data['vw'])
        # Parse color data, CMD_ID = 0x01
        elif cmd_id == self.config.COLOR_CMD_ID:
            # 0 for RED; 1 for BLUE
            if data['my_color'] == 'red':
                my_color_int = 0
            elif data['my_color'] == 'blue':
                my_color_int = 1
            packet += my_color_int.to_bytes(1, self.endianness)
        # Parse Selfcheck data, CMD_ID = 0x03
        if cmd_id == self.config.SELFCHECK_CMD_ID:
            # 0 for 'FLUSH' 1 for 'ECHO' 2 for 'ID',
            packet += self.config.SELFCHECK_MODE.index(data['mode']).to_bytes(1, self.endianness)
            packet += data['debug_int'].to_bytes(1, self.endianness)
        # Parse Arm data, CMD_ID = 0x04
        if cmd_id == self.config.ARM_CMD_ID:
            # "<f" means little endian float
            for v in data['floats'].values():
                packet += struct.pack('<f', v)
        # Data length = Total length - 9
        assert len(packet) == self.config.CMD_TO_LEN[cmd_id]
        return packet

    def get_current_stm32_state(self) -> dict:
        """Read from buffer from STM32 to Jetson and return the current state."""
        return self.stm32_state.deepcopy()

    def read_out(self) -> Optional[dict]:
        if self.is_alive():
            return self.get_current_stm32_state()


class ARMCommunicator(Communicator):

    def __init__(self, config, serial_dev_path, warn=True, in_use=None,):
        self.config = config
        self.serial_dev_path = serial_dev_path
        self.in_use = in_use

        self.warn = warn

        ids = [1, 2, 3, 4, 5, 6]

        if self.serial_dev_path is not None:
            in_use += [self.serial_dev_path]
            self.servo = DynamixelDriver(ids, port=serial_dev_path)
        elif self.warn:
            logger.warning("NO SERIAL DEVICE FOUND! I/O TO VACUUM!")
            self.servo = None
        logger.debug(f'I ({self.__class__=}) am using {self.servo}.')

        self.arm_state = StateDict(**{
            'floats': {
                'float0': 0.0,
                'float1': 0.0,
                'float2': 0.0,
                'float3': 0.0,
                'float4': 0.0,
                'float5': 0.0,
            }
        })

    @staticmethod
    def list_arm_device_paths(id_serial_short) -> Optional['path']:
        """Guess a port for the arm with `udevadm`.

        Note: this function is for UNIX-like systems only!

        Linux: look under "/dev/serial/by-id"

        Returns:
            [Maybe dev_paths] : a list of possible device paths
        """
        # Exclude UARTs
        id_cmd = "udevadm info -q property '%s' | awk -F= '/^ID_SERIAL_SHORT/ { print $2 }'"
        UART_PREFIX_LIST = ('usb-STMicroelectronics_STM32_STLink_',)
        # list of possible prefixes
        dev_basename = '/dev/serial/by-id'
        add_basename = functools.partial(os.path.join, dev_basename)
        try:
            path_list = map(add_basename, os.listdir(dev_basename))
        except FileNotFoundError:
            path_list = []

        dev_paths = [path
                     for path in path_list
                     if not path.startswith(UART_PREFIX_LIST)
                     if run(['/bin/bash', '-c', id_cmd % path],
                            stdout=PIPE).stdout.decode().strip() \
                                == id_serial_short]

        return dev_paths or [None]

    def start_listening(self) -> None:
        """Start a thread to listen to the serial port."""
        self.listen_thread = threading.Thread(target=self._listen)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def _listen(self, hz=200) -> None:
        while True:
            joint_angles = self.servo.get_joints()
            floats = zip(self.arm_state['floats'].keys(), joint_angles)
            self.arm_state.specific_update({'floats': dict(floats)})
            time.sleep(1/hz)

    def is_valid(self) -> bool:
        port = self.get_port()
        return os.path.exists(port)

    def is_vacuum(self) -> bool:
        return not self.is_valid()

    def is_alive(self) -> Optional[bool]:
        port = self.get_port()
        if self.is_valid():
            return True
        else:
            raise MiniPCCommunicationError(f'{port} no longer exists.')

    def get_port(self) -> Optional['path']:
        return self.serial_dev_path

    def create_and_send_packet(self, cmd_id, data) -> None:
        return None

    def create_packet(self, cmd_id, data) -> Union[bytes, dict]:
        return None

    def send_packet(self, packet) -> None:
        return None

    def read_out(self) -> dict:
        if self.is_alive():
            return self.arm_state.deepcopy()


class SPMCommunicator(Communicator):

    def __init__(self):
        pass

    def start_listening(self) -> None:
        pass

    def _listen(self, hz=200):
        pass

    def is_valid(self) -> bool:
        pass

    def is_vacuum(self) -> bool:
        pass

    def is_alive(self) -> bool:
        pass

    def get_port(self) -> Optional['path']:
        pass

    def create_and_send_packet(self, cmd_id, data) -> None:
        pass

    def create_packet(self, cmd_id, data) -> Union[bytes, dict]:
        pass

    def send_packet(self, packet) -> None:
        pass

    def read_out(self) -> dict:
        pass


if __name__ == '__main__':
    print(dir())
    # unit testing
    uart = UARTCommunicator(config)

    class Test(Enum):
        """Class used to choose test for communicator."""
        LATENCY = 1
        PINGPONG = 2
        CRC = 3
        TYPE_A = 4

    # remove first arg if called with python.
    if 'python' in sys.argv[0]:
        sys.argv.pop(0)

    testing = Test.LATENCY
    if len(sys.argv) > 1:
        testing = (Test.LATENCY, Test.PINGPONG,
                   Test.CRC, Test.TYPE_A)[int(sys.argv[1]) - 1]
        print(f'\nUsing test type: {testing}')
    else:
        print(f'\nUsing default test type: {testing}')
    print("Change test type: ./communicator.py {1,2,3,4}")
    print("1: LATENCY, 2: PINGPONG, 3: CRC, 4: TYPE_A\n")

    match testing:
        case Test.LATENCY:
            test_board_latency(uart, logger, listening=False)
        case Test.PINGPONG:
            test_board_pingpong(uart, logger, listening=False)
        case Test.CRC:
            test_board_crc(uart, logger, listening=False)
        case Test.TYPE_A:
            test_board_typea(uart, logger, listening=False)
        case _:
            print("Invalid selection")
