import time

def default(UC):
    arm_only(UC)

def example(UC):
    UART = UC.UART
    USB = UC.USB
    BRD = UC.BRD
    SPM = UC.SPM
    ARM = UC.ARM

    cmd_id = UC.config.ARM_CMD_ID
    while True:
        try:
            data = UC.unified_state[ARM]
            packet = UC.create_packet(BRD, cmd_id, data)
            UC.push_to_send_queue(BRD, packet)

            print('ARM', *UC.unified_state[ARM]['floats'].items())
            print('BRD', *UC.unified_state[BRD]['floats'].items())
        except Exception:
            pass
        time.sleep(1)

def arm_only(UC):
    # Do nothing. Entirely handled by Arduino.
    hz = 1
    while True:
        time.sleep(1 / hz)

def spm_only(UC):
    example(UC)

def both_arm_priority(UC):
    example(UC)

def both_spm_priority(UC):
    example(UC)

if __name__ == '__main__':
    print(dir())
