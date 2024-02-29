import argparse
from sys import argv

from minipc import main, minipc_parser

from util.ansi import *

from communication.menu import timedmenu

if __name__ == '__main__':
    #print(dir())
    # Remove first arg if called with python.
    _sl = slice(2, None) if 'python' in argv[0] \
        else slice(1, None)
    """ Items in parsed_args:
    parsed_args.verbosity: {DEBUG, INFO, WARNING, ERROR, CRITICAL}
                                                      (default WARNING,
                                                       w/o arg. INFO)
    parsed_args.test: 'octal'                         (d. '0o4')
    parsed_args.skip_tests: boolean
    parsed_args.test_only: boolean
    parsed_args.mode: str
    parsed_args.display_menu: boolean
    """
    parent_ap = minipc_parser()
    ap = argparse.ArgumentParser(
        parents=[parent_ap],
        add_help=False,
        description='Control script for minipc',
        prog='minipc.py',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog='Tests:\n'
        'test_board_latency:0o4\t'
        'test_board_pingpong:0o2\n'
        'test_board_crc:0o1\n'
        'Modes:\n'
        'arm_only:a\t\tspacemouse_only:s\n'
        'both_arm_priority:ba\tboth_spm_priority:bs')

    ap.add_argument('-m', '--mode',
                    action='store',
                    help='operating mode; see menu')
    ap.add_argument('-d', '--display-menu',
                    action='store_true',
                    help='display the menu')

    parsed_args = ap.parse_args(argv[_sl])

    try:
        if parsed_args.display_menu:
            choice = timedmenu()
            print(f'=> Selected {BLUE}{choice}{RESET}')
            parsed_args.mode = choice

        main(parsed_args)
    except KeyboardInterrupt:
        exit(130)
