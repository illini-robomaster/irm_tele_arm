#!/usr/bin/env python3
import os
import sys
from enum import Enum

if __name__ == '__main__':
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from util.ansi import *
from util.matching import MatchIn
from util.timedinput import timedinput, TimedOut


class NothingEnum(Enum):
    """Does not match MenuCode."""

    NOTHING = None


class MenuCode(Enum):
    """Menu codes."""

    # (str, int, message)
    # message can also be matched :)
    USEDEFAULT = ('d', '0', 'Use default')
    ARMONLY = ('a', '1', 'Arm only')
    SPMONLY = ('s', '2', 'Spacemouse only')
    BOTHA = ('ba', '3', 'Both, arm priority')
    BOTHS = ('bs', '4', 'Both, spacemouse priority')

    TESTL = ('tl', '11', 'Run latency test')
    TESTP = ('tp', '12', 'Run pingpong test')
    TESTC = ('tc', '13', 'Run crc test')

    EXIT130 = ('r', '130', 'Exit with code 130')


NOTHING = NothingEnum.NOTHING
USEDEFAULT = MenuCode.USEDEFAULT
ARMONLY = MenuCode.ARMONLY
SPMONLY = MenuCode.SPMONLY
BOTHA = MenuCode.BOTHA
BOTHS = MenuCode.BOTHS
TESTL = MenuCode.TESTL
TESTP = MenuCode.TESTP
TESTC = MenuCode.TESTC
EXIT130 = MenuCode.EXIT130

DEFAULT = ARMONLY


def get_code(
    x, timedout=DEFAULT, default=DEFAULT, unknown=DEFAULT, none=NOTHING
):
    # Fall through
    if isinstance(x, MenuCode):
        return x
    match MatchIn(x):
        case USEDEFAULT.value:
            ret = default
        case ARMONLY.value:
            ret = ARMONLY
        case SPMONLY.value:
            ret = SPMONLY
        case BOTHA.value:
            ret = BOTHA
        case BOTHS.value:
            ret = BOTHS
        case TESTL.value:
            ret = TESTL
        case TESTP.value:
            ret = TESTP
        case TESTC.value:
            ret = TESTC
        case EXIT130.value:
            ret = EXIT130
        case _:
            ret = NOTHING
    if ret is NOTHING:
        match x:
            case None:
                ret = none
            case TimedOut():
                ret = timedout
            case _:
                ret = unknown
    return ret


# Show menu
def menu() -> MenuCode:
    print('==> Menu')
    # Decorate menu entries
    for mc in MenuCode:
        # Newlines for each section
        if mc in (TESTL, EXIT130):
            print()
        # End strings
        if mc == DEFAULT:
            # tail <- default notice
            es = f'{RED}[DEFAULT]{RESET}'
        elif mc == EXIT130:
            # tail <- exit notice
            es = f'{YELLOW}[reload daemon]{RESET}'
        else:
            es = ''
        # Color of str
        if mc == EXIT130:
            col = YELLOW
        else:
            col = GREEN

        s, n, m = mc.value
        print(f'=> {col}{s:<2}{RESET} or {BLUE}{n:<3}{RESET}\t{m}', es)

    print('==> Hint: you use these codes on the previous menu.')
    choice = get_code(input('=> '))

    return choice


def timedmenu() -> MenuCode:
    # Exit on timeout
    print(f'==> Falling to default ({BLUE}{DEFAULT}{RESET}) in one second...')
    response = timedinput(1, f'=> ({GREEN}RETURN{RESET} to enter menu) ')
    code = get_code(response, timedout=DEFAULT, unknown=NOTHING)
    # Or do something if the code is valid
    if code in MenuCode:
        return code
    # Or enter the menu.
    try:
        choice = menu()
        return choice
    except KeyboardInterrupt:
        return EXIT130


if __name__ == '__main__':
    print(dir())
    print(timedmenu())
