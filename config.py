# ========== Communication ==========
PACK_START = b'ST'
PACK_END = b'ED'

GIMBAL_CMD_ID = 0x00
COLOR_CMD_ID = 0x01
CHASSIS_CMD_ID = 0x02
SELFCHECK_CMD_ID = 0x03
ARM_CMD_ID = 0x04

# mapping from cmd_id to data section length of the packet, unit: byte
# packet length = data length + 9
CMD_TO_LEN = {
    GIMBAL_CMD_ID: 10,
    COLOR_CMD_ID: 1,
    CHASSIS_CMD_ID: 12,
    SELFCHECK_CMD_ID: 2,
    ARM_CMD_ID: 24,
}
# length of Header + Tail = 9 bytes
HT_LEN = 9

# 0 for search target
# 1 for move yoke
GIMBAL_MODE = [
    'ST',
    'MY',
]

# 0 for echo
# 1 for ignore
SELFCHECK_MODE = [
    'FLUSH',
    'ECHO',
    'ID',
]

SEQNUM_OFFSET = 2
DATA_LENGTH_OFFSET = SEQNUM_OFFSET + 2
CMD_ID_OFFSET = DATA_LENGTH_OFFSET + 1
DATA_OFFSET = CMD_ID_OFFSET + 1

# `udevadm` short serial id of the small arm motors
ARM_ID_SERIAL_SHORT = 'FT8ISULT'

# ========== DEBUGGING ==========

DEBUG_DISPLAY = True
DEBUG_PRINT = False
DEFAULT_ENEMY_TEAM = 'red'
