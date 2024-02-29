#!/usr/bin/env python3
import os
import sys
from enum import Enum

if __name__ == '__main__':
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from util.ansi import *
from util.timedinput import timedinput, TimedOut

class NothingEnum(Enum):
    """Does not match MenuCode."""
    NOTHING = None

class MenuCode(Enum):
    """Menu codes."""
    ARMONLY = '1'
    SPMONLY = '2'
    BOTHA = '3'
    BOTHS = '4'

    TESTL = '11'
    TESTP = '12'
    TESTC = '13'

    EXIT130 = '130'

NOTHING = NothingEnum.NOTHING

ARMONLY = MenuCode.ARMONLY
SPMONLY = MenuCode.SPMONLY
BOTHA = MenuCode.BOTHA  
BOTHS = MenuCode.BOTHS  

TESTL = MenuCode.TESTL
TESTP = MenuCode.TESTP
TESTC = MenuCode.TESTC
EXIT130 = MenuCode.EXIT130

DEFAULT = ARMONLY

def getcode(x, timedout=DEFAULT, default=DEFAULT, none=NOTHING):
    match x:
        case 'a' | ARMONLY.value:
            ret = ARMONLY
        case 's' | SPMONLY.value:
            ret = SPMONLY
        case 'ba' | BOTHA.value:
            ret = BOTHA
        case 'bs' | BOTHS.value:
            ret = BOTHS
        case 'tl' | TESTL.value:
            ret = TESTL
        case 'tp' | TESTP.value:
            ret = TESTP
        case 'tc' | TESTC.value:
            ret = TESTC
        case 'r' | EXIT130.value:
            ret = EXIT130
        case None:
            ret = none
        case TimedOut():
            ret = timedout
        case _:
            ret = default
    return ret

# Show menu
def menu() -> MenuCode:
    ds = f'{RED}[DEFAULT]{RESET}'
    dr = f'{YELLOW}[reload daemon]{RESET}'
    print('==> Menu')
    print(f'=> {GREEN}a{RESET}  or {BLUE}{ARMONLY.value}{RESET}: Arm only', ds)
    print(f'=> {GREEN}s{RESET}  or {BLUE}{SPMONLY.value}{RESET}: Spacemouse only')
    print(f'=> {GREEN}ba{RESET} or {BLUE}{BOTHA.value}{RESET}: Both, arm priority')
    print(f'=> {GREEN}bs{RESET} or {BLUE}{BOTHS.value}{RESET}: Both, spacemouse priority')
    print()
    print(f'=> {GREEN}tl{RESET} or {BLUE}{TESTL.value}{RESET}: Run latency test')
    print(f'=> {GREEN}tp{RESET} or {BLUE}{TESTP.value}{RESET}: Run pingpong test')
    print(f'=> {GREEN}tc{RESET} or {BLUE}{TESTC.value}{RESET}: Run crc test')
    print()
    print(f'=> {YELLOW}r{RESET} or {BLUE}{EXIT130.value}{RESET}: exit(130)', dr)
    print('==> Hint: you use these codes on the previous menu.')
    choice = getcode(input('=> '))

    return choice

def timedmenu() -> MenuCode:
    # Exit on timeout
    print('==> Falling to default in one second...')
    response = timedinput(1, f'=> ({GREEN}RETURN{RESET} to enter menu) ')
    code = getcode(response, default=NOTHING)
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
