import time

def default(UC):
    arm_only(UC)

def example(UC):
    UART = UC.UART
    USB = UC.USB
    BRD = UC.BRD
    SPM = UC.SPM
    ARM = UC.ARM
    PPM = UC.PPM

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
    UART = UC.UART
    USB = UC.USB
    BRD = UC.BRD
    SPM = UC.SPM
    ARM = UC.ARM
    PPM = UC.PPM

    hz = 40
    hz_print = 30

    cmd_id = UC.config.ARM_CMD_ID
    i = 0;
    while True:
        try:
            data = {'floats': UC.unified_state[ARM]['floats']}
            packet = UC.create_packet(PPM, cmd_id, data)
            UC.push_to_send_queue(PPM, packet)
            
            if i > hz/hz_print:
                i = 0
                print(f'=> {hz_print}Hz sample (of {hz}Hz actual):')
                print('PPM', *UC.unified_state[PPM]['floats'].items())
                print('BRD', *UC.unified_state[ARM]['floats'].items())
        except Exception:
            pass
        i += 1
        time.sleep(1 / hz)

def spm_only(UC):
    example(UC)

def both_arm_priority(UC):
    example(UC)

def both_spm_priority(UC):
    example(UC)

if __name__ == '__main__':
    print(dir())
