import os
import sys
import logging
if __name__ == '__main__':
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from util.ansi import *


class ColorFormatter(logging.Formatter):
    format0 = "%(levelname)s - %(message)s (%(filename)s:%(lineno)d)"
    format1 = "%(asctime)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"
    format2 = "%(asctime)s - %(name)s: %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"

    FORMATS = {
        logging.DEBUG: LIGHTGREY + format1 + RESET,
        logging.INFO: LIGHTGREY + format0 + RESET,
        logging.WARNING: YELLOW + format1 + RESET,
        logging.ERROR: RED + format2 + RESET,
        logging.CRITICAL: BOLD + RED + format2 + RESET
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

if __name__ == '__main__':
    print(dir())
