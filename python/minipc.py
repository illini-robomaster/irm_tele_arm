#!/usr/bin/env python3
"""To add a test:
    - Add code into `communication/tests.py'.
    - Include the test in `startup_tests` in `minipc.py'.
    - Add the test into the argparse in `minipc.py` and add an octal
      identification number. Document in the epilog in `minipc.py` and
      `__main__.py` (which extends `minipc' argparse).
To add a new device:
    - Add the DeviceType in `config.py'
    - Add a communicator in `communication/communicator.py' with `Communicator'
      as its parent.
    - Modify 'global' variables in `minipc.py' accordingly.
    - Include it in `serial_devices` in `minipc.py'.
    - Include it in `UnifiedCommunicator.get_communicator`.
To change functionality:
    - Add a runmode in `communication/runmode.py'. Use functionality in
      UnifiedCommunicator.
    - Add the test into the argparse in `__main__.py' and modify
      `communication/menu.py' accordingly. Document in the epilog of
      `__main__.py`.
"""
import time
import serial
import logging
import argparse
import itertools
import threading
from typing import Optional, Union, \
    Sequence, Tuple, List, \
    Set, \
    Mapping, Dict, \
    Callable

import config
from config import DeviceType

from util import colorlog as cl
from util.ansi import *
from util.matching import MatchAll
from util.atomiclist import AtomicList

from communication import tests
from communication import runmode
from communication.menu import get_code, MenuCode
from communication.communicator import Communicator, \
    UARTCommunicator, ARMCommunicator, SPMCommunicator, \
    StateDict, MiniPCCommunicationError

_mt = MatchAll(True)

# Add loggers here
loggers = [logger := logging.getLogger(__name__),
           c_logger := logging.getLogger('communication.communicator'),
           d_logger := logging.getLogger('thirdparty.dynamixel.driver'),]

UART = DeviceType.UART
USB = DeviceType.USB
BRD = DeviceType.BRD
SPM = DeviceType.SPM
ARM = DeviceType.ARM

EMPTY = UARTCommunicator(config, serial_dev_path=False, warn=False)

# This is expected to be thread-safe.
# Muting data in an (Int)Enum is not expected behavior, so:
serial_devices: Dict[DeviceType, Communicator] = {
    UART: EMPTY,
    USB: EMPTY,
    BRD: None,
    SPM: None,
    ARM: None,
}

in_use: List['path'] = AtomicList()
# XXX: Change names or reimplement? Only send_queue actually acts as a queue.
id_queue: List[DeviceType] = AtomicList()
send_queue: List[Tuple[DeviceType, bytes]] = AtomicList()
listen_queue: List[Tuple[DeviceType, Communicator]] = AtomicList()
unified_state = StateDict()

# Returns the argument parser for this script
def minipc_parser():
    """Items in parsed_args:
    parsed_args.verbosity: [DEBUG, INFO, WARNING, ERROR, CRITICAL]
                                                      (default WARNING,
                                                       w/o arg INFO)
    parsed_args.test: 'octal'                         (d. '0o4')
    parsed_args.skip_tests: boolean
    parsed_args.test_only: boolean
    parsed_args.mode: str
    """
    ap = argparse.ArgumentParser(
        description='Control script for minipc',
        prog='minipc.py',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog='TEST values:\n'
        '  test_board_latency:  0o4\n'
        '  test_board_pingpong: 0o2\n'
        '  test_board_crc: ---- 0o1\n'
        'MODE values:\n'
        '  arm_only: ---------- a\n'
        '  spacemouse_only: --- s\n'
        '  both_arm_priority: - ba\n'
        '  both_spm_priority: - bs')

    # TODO: handle numeric verbosity
    ap.add_argument('-v', '--verbosity', '--verbose',
                    action='store',
                    default='WARNING',
                    nargs='?',
                    help='DEBUG, INFO, WARNING, ERROR, or CRITICAL')
    ap.add_argument('-t', '--test', action='store', default='0o4',
                    help='takes an octal value; values below')
    ap.add_argument('-s', '--skip-tests',
                    action='store_true',
                    help='alias for --test 0')
    ap.add_argument('--test-only',
                    action='store',
                    help='exit after completing tests')
    ap.add_argument('-m', '--mode',
                    action='store',
                    help='operating mode; see menu')
    ap.add_argument('--version',
                    action='version',
                    version='%(prog)s 0.1')
    ap.add_argument('--dummy',
                    action='store_true',
                    help=argparse.SUPPRESS)
    return ap

def set_up_loggers(
        loggers: List[logging.Logger], verbosity) -> None:
    """Takes a list of loggers and sets handler and verbosity."""
    ch = logging.StreamHandler()
    ch.setFormatter(cl.ColorFormatter())
    for lgr in loggers:
        lgr.handlers.clear()
        lgr.addHandler(ch)
        lgr.setLevel(verbosity or 'INFO')

#
# XXX: Only _identifier is expected to work with anonymous devices.
# Only use `create_packet` and `push_to_send_queue` outside of the three
# threads (_identifier, _listener, _sender), everything else is for internal
# use only.
#
class UnifiedCommunicator:
    """Contains general communication and identification methods.
    This is expected to be provided to all runmodes.
    """
    def __init__(self,
                 config,
                 logger,
                 in_use,
                 id_queue,
                 send_queue,
                 listen_queue,
                 unified_state,):
        self.config = config
        self.in_use = in_use
        self.logger = logger
        self.id_queue = id_queue
        self.send_queue = send_queue
        self.listen_queue = listen_queue
        self.unified_state = unified_state
        self.serial_devices = serial_devices

        self.UART = self.config.DeviceType.UART
        self.USB = self.config.DeviceType.USB
        self.BRD = self.config.DeviceType.BRD
        self.SPM = self.config.DeviceType.SPM
        self.ARM = self.config.DeviceType.ARM
        self.intenum_to_name = {
            UART: 'UART',
            USB: 'USB',
            BRD: 'BOARD',
            SPM: 'SPACEMOUSE',
            ARM: 'ARM',
        }


    # [i]nt[e]num
    def iename(self, intenum: DeviceType) -> str:
        """Get a string given a DeviceType"""
        return self.intenum_to_name[intenum]


    # [c]lass
    def ctype(self, device: Communicator) -> DeviceType:
        """Get a DeviceType given a Communicator"""
        match device:
            case UARTCommunicator():
                return self.UART
            case ARMCommunicator():
                return self.ARM
            case SPMCommunicator():
                return self.SPM
            case _:
                return None


    # [f]etch [dev]ice
    def fdev(self, dev_type: DeviceType) -> Communicator:
        """Returns a Communicator given a DeviceType"""
        return self.serial_devices[dev_type]


    def is_uart(self, dev_type: DeviceType) -> bool:
        """Check whether a DeviceType is a UART"""
        return dev_type >= self.UART


    def is_usb(self, dev_type: DeviceType) -> bool:
        """Check whether a DeviceType is a USB"""
        return dev_type < self.UART


    def is_arm(self, dev_type: DeviceType) -> bool:
        """Check whether a DeviceType is an ARM"""
        return dev_type == self.ARM


    def is_spm(self, dev_type: DeviceType) -> bool:
        """Check whether a DeviceType is a SPM"""
        return dev_type == self.SPM


    def delist_device(self, device: Communicator) -> None:
        """Remove a Communicator's port from the internal list of used ports"""
        dev_path = device.get_port()
        if dev_path:
            self.logger.debug(f'Freeing {dev_path}.')
            self.in_use.remove(dev_path)


    def get_communicator(
            self, dev_type: DeviceType, *args, **kwargs) -> Communicator:
        """Returns a Communicator given a DeviceType"""
        if self.is_uart(dev_type):
            return UARTCommunicator(in_use=self.in_use, *args, **kwargs)
        elif self.is_arm(dev_type):
            return ARMCommunicator(in_use=self.in_use, *args, **kwargs)
        elif self.is_spm(dev_type):
            return SPMCommunicator(in_use=self.in_use, *args, **kwargs)


    def _create_packet_dev(
            self, device: Communicator, cmd_id: hex, data: dict) -> bytes:
        """Returns a packet(: bytes) given a Communicator"""
        return device.create_packet(cmd_id, data)


    def _send_packet_dev(self, device: Communicator, packet: bytes) -> None:
        """Sends a packet(: bytes) given a Communicator"""
        return device.send_packet(packet)


    def _create_and_send_packet_dev(
            self, device: Communicator, cmd_id: hex, data: dict) -> None:
        """Creates and sends a packet(: bytes) given a Communicator,
        a command id(: hex), and data(: dict)
        """
        return device.create_and_send_packet(cmd_id, data)


    def _read_packet_dev(self, device: Communicator) -> dict:
        """Returns data(: dict) given a Communicator"""
        return device.read_out()


    def _receive_uart_packet_dev(self, device: UARTCommunicator) -> None:
        """Forces a UARTCommunicator to attempt to receive a packet"""
        device.try_read_one()
        device.packet_search()


    def _send_packet(self, dev_type: DeviceType, packet: bytes) -> None:
        """Sends a packet(: bytes) given a DeviceType"""
        device = self.fdev(dev_type)
        return self._send_packet_dev(device, packet)


    def _create_and_send_packet(
            self, dev_type: DeviceType, cmd_id: hex, data: dict) -> None:
        """Creates and sends a packet given a DeviceType,
        a command id(: hex), and data(: dict)
        """
        device = self.fdev(dev_type)
        return self._create_and_send_packet_dev(device, cmd_id, data)


    def _read_packet(self, dev_type: DeviceType) -> dict:
        """Returns data(: dict) given a DeviceType"""
        device = self.fdev(dev_type)
        return self._read_packet_dev(device)


    def create_packet(
            self, dev_type: DeviceType, cmd_id: hex, data: dict) -> bytes:
        """Returns packet(: bytes) given a DeviceType"""
        device = self.fdev(dev_type)
        return self._create_packet_dev(device, cmd_id, data)


    def push_to_send_queue(self, dev_type: DeviceType, packet: bytes) -> None:
        """Queues a packet(: bytes) to be sent given a DeviceType"""
        self.send_queue += [(dev_type, packet)]


    def create_and_push(
            self, dev_type: DeviceType, cmd_id: hex, data: dict) -> None:
        """Queues a packet to be sent given a DeviceType,
        a command id(: hex), and data(: dict)
        """
        self.send_queue += [(dev_type, create_packet(cmd_id, data))]


UC = UnifiedCommunicator(config=config,
                         logger=logger,
                         in_use=in_use,
                         id_queue=id_queue,
                         send_queue=send_queue,
                         listen_queue=listen_queue,
                         unified_state=unified_state)

def _identifier(hz_uart=2, hz_usb=2) -> None:
    """Watches the id queue. If there are devices waiting to be identified,
    attempt to do so. After identification the Communicator is passed to the
    listen queue.
    """
    global UC

    def free_from_id_queue(dev_type: DeviceType) -> None:
        if dev_type in UC.id_queue:
            UC.id_queue.remove(dev_type)

    def push_to_listen_queue(
            dev_type: DeviceType, device: Communicator) -> None:
        UC.listen_queue += [(dev_type, device)]

    def _id_uart(hz):
        """Identify UART devices by polling with an ID request. Response is
        implemented on the UART device.
        """
        serial_experiments = []
        paths, prev_paths = [set()] * 2

        # cmd_id = config.SELFCHECK_CMD_ID
        # data = {'mode': 'ID', 'debug_int': 0}
        cmd_id = config.SELFCHECK_CMD_ID
        data = {'mode': 'ID', 'debug_int': 3}
        packet = UC.create_packet(UART, cmd_id, data)
        while True:
            known_paths = {dev.get_port()
                           for dev in serial_devices.values()
                           if dev is not None
                           if not dev.is_vacuum()}
            if not any(map(UC.is_uart, UC.id_queue)):
                for dev in serial_experiments:
                    UC.delist_device(dev)
                serial_experiments = []
                paths, prev_paths = [set()] * 2
                time.sleep(1 / hz)
                continue
            # Look for devices.
            # `list_uart_device_paths` returns a list or [None]
            paths = set(UARTCommunicator.list_uart_device_paths()) | \
                    set(ARMCommunicator.list_arm_device_paths()) - known_paths
            for pth in paths - prev_paths - {None}:
                try:
                    serial_experiments += [UC.get_communicator(
                        UART, config, serial_dev_path=pth)]
                    logger.info(
                        f'Added {serial_experiments[-1]} '
                        'as identification candidate')
                # Do not skip on error.
                # Prevent `pth' from going into `prev_paths'.
                except Exception:
                    paths -= {pth}
            prev_paths = paths

            # Poll found devices.
            for dev in serial_experiments.copy():
                try:
                    UC._send_packet_dev(dev, packet)
                    UC._receive_uart_packet_dev(dev)
                    received_data = UC._read_packet_dev(dev)
                    dev_type = received_data['debug_int'] - 127
                except Exception:
                    serial_experiments.remove(dev)
                    logger.info(
                            f'Removed {dev} as identification candidate')
                    UC.delist_device(dev)
                else:
                    # Success.
                    if dev_type in UC.id_queue:
                        serial_experiments.remove(dev)
                        logger.info(f'Identified {UC.iename(dev_type)}, '
                                    f'pushing {dev} to listen queue.')
                        push_to_listen_queue(dev_type, dev)
                        logger.info(f'Freeing {UC.iename(dev_type)} '
                                    'from identification queue.')
                        free_from_id_queue(dev_type)
            time.sleep(1 / hz)

    def _id_usb(hz):
        """Identify USB devices by checking their serial ID against known
        serial IDs in `config.py' with `udevadm`.
        """
        arm_cycle = itertools.repeat(ARM)
        spm_cycle = itertools.repeat(SPM)
        serial_experiments = []
        paths, prev_paths = [set()] * 2
        while True:
            known_paths = {dev.get_port()
                           for dev in serial_devices.values()
                           if dev is not None
                           if not dev.is_vacuum()}
            if not any(map(UC.is_usb, UC.id_queue)):
                for dev in serial_experiments:
                    UC.delist_device(dev)
                serial_experiments = []
                paths, prev_paths = [set()] * 2
                time.sleep(1 / hz)
                continue

            # Look for devices.
            # `list_*_device_paths` returns a list or [None]
            arm_paths, spm_paths = [set()] * 2
            # ARM is currently a UART.
            #if ARM in UC.id_queue:
            #    arm_paths = set(
            #            ARMCommunicator.list_arm_device_paths()) - known_paths
            if SPM in UC.id_queue:
                spm_paths = set(
                        SPMCommunicator.list_spm_device_paths(
                            config.SPM_ID_SERIAL)) - known_paths

            paths = set.union(arm_paths, spm_paths)
            for dt, pth in itertools.chain(
                    zip(arm_cycle, arm_paths-prev_paths-{None}),
                    zip(spm_cycle, spm_paths-prev_paths-{None})):
                try:
                    serial_experiments += [UC.get_communicator(
                        dt, config, serial_dev_path=pth)]
                    logger.info(
                        f'Added {serial_experiments[-1]} '
                        'as identification candidate')
                # Do not skip on error.
                # Prevent `pth' from going into `prev_paths'.
                except Exception:
                    paths -= {pth}
            prev_paths = paths

            # Check alive status of found devices
            for dev in serial_experiments.copy():
                try:
                    dev_type = UC.ctype(dev)
                    dev.is_alive()
                except MiniPCCommunicationError:
                    pass
                else:
                    # Success.
                    serial_experiments.remove(dev)
                    logger.info(f'Identified {UC.iename(dev_type)}, '
                                f'pushing {dev} to listen queue.')
                    push_to_listen_queue(dev_type, dev)
                    logger.info(f'Freeing {UC.iename(dev_type)} '
                                'from identification queue.')
                    free_from_id_queue(dev_type)
            time.sleep(1 / hz)

    id_uart_thread = threading.Thread(target=_id_uart,
                                      args=(hz_uart,))
    id_usb_thread = threading.Thread(target=_id_usb,
                                     args=(hz_usb,))
    id_uart_thread.start()
    id_usb_thread.start()

#
# Starts listening for new devices and updates `unified_state'. The
# frequency parameter only controls the rate of scanning `listen_queue' and
# fetching the state from `serial_devices' and not the frequency of the
# Communicator objects (in `serial_devices') themselves.
#
def _listener(hz_pull=4, hz_push=200):
    """Pulls devices from the listen queue and sets them to start listening.
    Reads the state of the Communicator and pushes it to `unified_state`.
    """
    global UC

    def free_from_listen_queue(dev_type: DeviceType) -> None:
        UC.listen_queue.filter_in_place(lambda el: el[0] != dev_type)

    def push_to_id_queue(dev_type: DeviceType) -> None:
        UC.id_queue += [dev_type]

    def assign_device(dev_type: DeviceType, device: Communicator) -> None:
        serial_devices[dev_type] = device

    def deassign_device(dev_type: DeviceType) -> None:
        device = UC.fdev(dev_type)
        assign_device(dev_type, None)
        UC.delist_device(device)

    def _queue_puller(hz):
        """Set devices in queue to listen mode."""
        while True:
            # Listen on and assign new devices
            for dev_type, dev in UC.listen_queue:
                try:
                    dev.start_listening()
                    assign_device(dev_type, dev)
                    logger.info(f'{UC.iename(dev_type)} listening as '
                                f'{dev}.')
                    print(f'=> {UC.iename(dev_type)}: '
                          f'{GREEN if dev else RED}{dev}{RESET}')
                except Exception:
                    logger.error(f'{UC.iename(dev_type)} failed to listen as '
                                 f'{dev}.')
                    logger.info(f'Returning {UC.iename(dev_type)} '
                                'to the identification queue.')
                    push_to_id_queue(dev_type)
                finally:
                    logger.info(f'Freeing {UC.iename(dev_type)} from listen queue.')
                    free_from_listen_queue(dev_type)
            time.sleep(1 / hz)

    def _state_pusher(hz):
        """Read from named Communicators in `serial_devices`."""
        for dev_type, dev in serial_devices.items():
            if dev is None or not dev.is_vacuum:
                unified_state.setdefault(dev_type, dict())
        while True:
            # Get output of known devices.
            for dev_type, dev in serial_devices.items():
                if dev is not None:
                    if not dev.is_vacuum():
                        try:
                            received_data = UC._read_packet(dev_type)
                            unified_state.update({dev_type: received_data})
                        except MiniPCCommunicationError:
                            logger.error('Lost connection to '
                                         f'{UC.iename(dev_type)}, deassigning '
                                         f'{UC.fdev(dev_type)}.')
                            deassign_device(dev_type)
                            logger.info(f'Pushing {UC.iename(dev_type)} '
                                        'to identification queue.')
                            push_to_id_queue(dev_type)
                            print(f'=> {UC.iename(dev_type)}: '
                                  f'{RED}{UC.fdev(dev_type)}{RESET}')
                else:
                    if dev_type not in id_queue and \
                            (dev_type, _mt) not in UC.listen_queue:
                        logger.info(f'Pushing {UC.iename(dev_type)} '
                                    'to identification queue.')
                        push_to_id_queue(dev_type)
            time.sleep(1 / hz)

    queue_puller_thread = threading.Thread(target=_queue_puller,
                                           args=(hz_pull,))
    state_pusher_thread = threading.Thread(target=_state_pusher,
                                           args=(hz_push,))
    queue_puller_thread.start()
    state_pusher_thread.start()


def _sender(hz=200):
    """Sends packets in send queue to their destinations."""
    global UC

    def takeone_from_send_queue() -> bytes:
        return send_queue.take_one()

    def taken_from_send_queue(
            dev_type: DeviceType, n: int = 1) -> List[bytes]:
        return send_queue.take_n(n)

    while True:
        head = takeone_from_send_queue()
        if head is not None:
            dev_type, packet = head
            try:
                UC._send_packet(dev_type, packet)
            except Exception:
                pass
        time.sleep(1 / hz)

#
# Convert octal to binary representation and run tests.
#
def startup_tests(verb: oct = 0o4, hz=4) -> None:
    """Run startup tests from `communication.tests'."""
    global UC
    testable: List[Tuple[Callable, DeviceType]] = [
        (tests.test_board_crc, BRD),       # 0o1
        (tests.test_board_pingpong, BRD),  # 0o2
        (tests.test_board_latency, BRD),   # 0o4
    ]
    # i.e. 0o6 => reversed('110') will run test_board_{latency,pingpong}
    bits = map(int, reversed(bin(verb)[2:]))
    selected_tests = itertools.compress(testable, bits)
    # [d]ev_[t]ype, [dev]ice, [req]uired
    dt_dev_reqset = {(dt, UC.fdev(dt))
                     for _, dt in testable
                     if dt is not None}
    dt_reqlst, dev_reqlst = [*zip(*dt_dev_reqset)]  # Unzip into two lists
    if any(dev is None
           for dev in dev_reqlst):
        print('==> Not all devices required for testing attached, waiting.')
        for dt, dev in dt_dev_reqset:
            print(f'=> {UC.iename(dt)}: {GREEN if dev else RED}{dev}{RESET}')
    # Wait for devices to be ready.
    while any(UC.fdev(dt) is None
              for dt in dt_reqlst):
        time.sleep(1 / hz)
    print('==> All devices attached, continuing.\n')
    try:
        return [action(UC.fdev(dev_type), logger)
                for action, dev_type in selected_tests]
    except Exception as e:
        logger.error('Startup tests failed, exiting...')
        raise RuntimeError from e


def main(args):
    """The main function. Can be exported."""
    global UC
    # Set up logging.
    set_up_loggers(loggers, args.verbosity)
    logger.debug(args)
    logger.debug(loggers)

    # Fork off our communication threads.
    hz_id = hz_wait = hz_pull = 4
    identifier_thread = threading.Thread(target=_identifier,
                                         args=(hz_id, hz_id))
    listener_thread = threading.Thread(target=_listener,
                                       args=(hz_pull,))
    sender_thread = threading.Thread(target=_sender,)

    identifier_thread.daemon = True
    listener_thread.daemon = True
    sender_thread.daemon = True

    identifier_thread.start()
    listener_thread.start()
    sender_thread.start()

    # Run startup tests.
    if not args.skip_tests:
        startup_tests(int(args.test, 8), hz_wait)
    if args.test_only:
        exit(0)

    mode = get_code(args.mode)
    logger.info(f'Runmode {mode}')

    # Program
    match mode:
        case MenuCode.ARMONLY:
            runmode.arm_only(UC)
        case MenuCode.SPMONLY:
            runmode.spm_only(UC)
        case MenuCode.BOTHA:
            runmode.both_arm_priority(UC)
        case MenuCode.BOTHS:
            runmode.both_spm_priority(UC)
        case MenuCode.TESTL:
            startup_tests(0o4)
            input(f'=> ({GREEN}ENTER{RESET} to continue) ')
        case MenuCode.TESTP:
            startup_tests(0o2)
            input(f'=> ({GREEN}ENTER{RESET} to continue) ')
        case MenuCode.TESTC:
            startup_tests(0o1)
            input(f'=> ({GREEN}ENTER{RESET} to continue) ')
        case MenuCode.EXIT130:
            exit(130)
        case _:
            runmode.default(UC)

if __name__ == '__main__':
    #print(dir())
    ap = minipc_parser()
    parsed_args = ap.parse_args()
    try:
        main(parsed_args)
    except KeyboardInterrupt:
        logger.debug('Caught KeyboardInterrupt')
        exit(130)
    finally:
        logger.info('I have died.')
