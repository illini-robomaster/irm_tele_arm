#!/usr/bin/env python3
############################################################################
#                                                                          #
#  Copyright (C) 2024 RoboMaster.                                          #
#  Illini RoboMaster @ University of Illinois at Urbana-Champaign          #
#                                                                          #
#  This program is free software: you can redistribute it and/or modify    #
#  it under the terms of the GNU General Public License as published by    #
#  the Free Software Foundation, either version 3 of the License, or       #
#  (at your option) any later version.                                     #
#                                                                          #
#  This program is distributed in the hope that it will be useful,         #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of          #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           #
#  GNU General Public License for more details.                            #
#                                                                          #
#  You should have received a copy of the GNU General Public License       #
#  along with this program. If not, see <http://www.gnu.org/licenses/>.    #
#                                                                          #
############################################################################
import argparse

from minipc import main, minipc_parser

from util.ansi import *

from communication.menu import timedmenu

if __name__ == '__main__':
    """Items in parsed_args:
    parsed_args.verbosity: {DEBUG, INFO, WARNING, ERROR, CRITICAL}
                                                      (default WARNING,
                                                       w/o arg. INFO)
    parsed_args.test: 'octal'                         (d. '0o4')
    parsed_args.skip_tests: boolean
    parsed_args.test_only: boolean
    parsed_args.mode: str
    parsed_args.display_menu: boolean
    """
    #print(dir())
    parent_ap = minipc_parser()
    ap = argparse.ArgumentParser(
        parents=[parent_ap],
        add_help=False,
        description=parent_ap.description,
        prog=parent_ap.prog,
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=parent_ap.epilog)

    ap.add_argument('-d', '--display-menu',
                    action='store_true',
                    help='display the menu')

    parsed_args = ap.parse_args()

    try:
        if parsed_args.display_menu:
            choice = timedmenu()
            print(f'=> Selected {BLUE}{choice}{RESET}')
            parsed_args.mode = choice
        main(parsed_args)
    except KeyboardInterrupt:
        exit(130)
