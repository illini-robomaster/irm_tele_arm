import time
import threading
# latency test by richard, flash example/minipc/latencytest.cc
# modified by austin.
# tests minipc <-> type c board circuit time
def test_board_latency(uart, logger,
                       rounds=15, timeout=1, hz=200,
                       listening=True, verbose=True):
    print('\nCommunicator beginning minipc <-> board latency test: '
          f'{rounds} rounds at {hz} hertz')
    cmd_id = uart.config.SELFCHECK_CMD_ID

    def send_packets(rounds, hz):
        send_time = [0] * rounds
        packet_status = [False] * rounds
        for i in range(rounds):
            logger.debug(f'Sending packet #{i} to stm32...')
            data = {'mode': 'ECHO', 'debug_int': i}

            send_time[i] = time.time()
            uart.create_and_send_packet(cmd_id, data)
            packet_status[i] = True

            time.sleep(1 / hz)

        return (send_time, packet_status)

    def receive_packets(rounds, timeout, listening, ret):  # async
        received = 0
        receive_time = [0] * rounds
        packet_status = [False] * rounds
        # receive loop
        current_time = time.time()
        while time.time() - current_time < timeout and received != rounds:
            if not listening:
                uart.try_read_one()
                if uart.packet_search():
                    received_data = uart.get_current_stm32_state()
                    received += 1
                else:
                    time.sleep(1 / hz)
                    continue
            else:
                received_data = uart.get_current_stm32_state()
            i = int(received_data['debug_int'])
            try:
                # debug_int acts as the index
                if not receive_time[i]:
                    receive_time[i] = time.time()
                    logger.debug(f'Received packet #{i} from stm32...')
            except IndexError:
                pass
            time.sleep(1 / hz)
        for i, t in enumerate(receive_time):
            if t != 0:
                packet_status[i] = True

        ret[0:1] = [receive_time, packet_status]
        return ret[0:1]  # if not run as thread.

    # start the receive thread first
    rt_return = []
    receive_thread = threading.Thread(target=receive_packets,
                                      args=(rounds, timeout, listening,
                                            rt_return))
    receive_thread.start()
    # send packets second
    send_time, send_packet_status = send_packets(rounds, hz)
    receive_thread.join()
    print(rt_return)
    receive_time, receive_packet_status = rt_return
    # flatten data
    not_all_received = not all(receive_packet_status)
    # 0 if packet not received
    latencies = [(tf or ti) - ti
                 for ti, tf in zip(send_time, receive_time)]
    statuses = [*zip(send_packet_status, receive_packet_status)]

    loss = latencies.count(0.0)
    average_latency = sum(latencies) / (len(latencies) - loss or 1)  # prevent 0/0

    for i in range(rounds):
        is_sent = statuses[i][0]
        is_received = statuses[i][1]
        logger.debug('Status of packet %d: send: %s, receive: %s' %
                     (i, ('UNSENT!', 'Sent')[is_sent],
                      ('NOT RECEIVED!', 'Received')[is_received]))
        logger.debug(f'Latency of packet #{i}: {latencies[i]}')

    print('Attempted to send', rounds, 'packets.',
          send_packet_status.count(True), 'packets transmitted,',
          rounds - loss, 'packets received.')
    print(f'Packets lost: {loss}/{loss/rounds*100}%. '
          f'Average latency: {average_latency}')
    if not_all_received:
        logger.warning('Latency test: not all packets were received.')

    return {'average': average_latency,
            'loss': (loss, loss / rounds),
            'detailed': [*zip(statuses, latencies)]}

#
# rx/tx test by yhy modified by richard, flash example/minipc/pingpongtest.cc
# this ping pong test first tries to send a packet
# and then attmepts to read the response from stm32 for 10 seconds
# then send a second packet
# after that entering ping pong mode:
#   receive packet from stm32, debug_int += 1 then immediately send back
# each "ping pong" has a id for differentiating during pingping-ing
# todo: this test shows the issue that a response can only be received after the data
# in circular_buffer is at least the maximum size of a packet (stj_max_packet_size).
# so if sending some small packets,
# they will stay in the circular_buffer waiting to be parsed,
# until new packets are received.
# for example, if stj_max_packet_size is 21 and gimbal data size is 19,
# then only after receiving 2 packets (2 * 19 > 21)
# then the first packet will be parsed.
# if a data type is 10 bytes long then sending a third packet is necessary
# before pingpong
#
# Modified by Austin.
#
def test_board_pingpong(uart, logger,
                        rounds=5, timeout=1, hz=200,
                        listening=True, verbose=True):
    print('\nCommunicator beginning minipc <-> board pingpong test: '
          f'{rounds} rounds at {hz} hertz')

    def receive_packet(j, timeout):
        current_time = time.time()
        while time.time() - current_time < timeout:
            if not listening:
                uart.try_read_one()
                if uart.packet_search():
                    return True
                else:
                    time.sleep(1 / hz)
                    continue
            else:
                received_data = uart.get_current_stm32_state()
                i = int(received_data['debug_int'])
                if i == j:
                    return True
            time.sleep(1 / hz)

        return False

    def send_recv_packets(rounds, timeout, hz):
        sent, received = 0, 0
        cmd_id = uart.config.SELFCHECK_CMD_ID
        flusher = uart.create_packet(cmd_id, {'mode': 'FLUSH', 'debug_int': 0})
        for i in range(rounds):
            print(f'Sending packet #{i} to stm32...')
            data = {'mode': 'ECHO', 'debug_int': i + 1}
            uart.create_and_send_packet(cmd_id, data)
            for _ in range(5):
                time.sleep(1 / hz)
                uart.send_packet(flusher)
            sent += 1

            received_data = receive_packet(i + 1, timeout)
            if received_data:
                received += 1
                print(f'Received packet #{i}')
            else:
                print(f'Lost packet #{i}.')

            time.sleep(1 / hz)
        return (sent, received)

    sent, received = send_recv_packets(rounds, timeout, hz)

#
# rate test by roger modified by richard, flash example/minipc/stresstesttypec.cc
#
# Modified by Austin.
#
def test_board_crc(uart, logger,
                   rounds=15, timeout=1, hz=200,
                   listening=True, verbose=True):
    print('\nCommunicator beginning minipc <-> board crc stress test: '
          f'{rounds} rounds at {hz} hertz')
    cmd_id = uart.config.SELFCHECK_CMD_ID

    def send_packets(rounds, hz):
        packet_status = [False] * rounds
        for i in range(rounds):
            logger.debug(f'Sending packet #{i} to stm32...')
            data = {'mode': 'ECHO', 'debug_int': 0}

            uart.create_and_send_packet(cmd_id, data)
            packet_status[i] = True

            time.sleep(1 / hz)

        return packet_status

    def receive_packets(rounds, timeout, ret):  # async
        received = 0
        packet_status = [False] * rounds
        # receive loop
        current_time = time.time()
        while time.time() - current_time < timeout and received != rounds:
            if not listening:
                uart.try_read_one()
                if uart.packet_search():
                    received_data = uart.get_current_stm32_state()
                    i = int(received_data['debug_int'])
                else:
                    time.sleep(1 / hz)
                    continue
            else:
                received_data = uart.get_current_stm32_state()
            try:
                # debug_int acts as the index
                i = int(received_data['debug_int'])
                if not packet_status[i]:
                    packet_status[i] = True
                    logger.debug(f'Received packet #{i} from stm32...')
                    received += 1
            except IndexError:
                pass
            time.sleep(1 / hz)

        ret[0] = packet_status
        return ret[0]  # if not run as thread.

    # send packets first
    print('This test should be run without a listening thread. '
          'Otherwise, expect only one packet.')
    send_packet_status = send_packets(rounds, hz)
    print(f'Packet sending test complete: sent {rounds} packets.')
    print('You should see the light change from blue to green on type c board.')
    print('When the led turns red the stm32 is sending data.')
    print('Starting packet receiving test.')
    # start the receive thread second
    rt_return = [None]
    receive_thread = threading.Thread(target=receive_packets,
                                      args=(rounds, timeout, rt_return))
    receive_thread.daemon = True
    receive_thread.start()
    receive_thread.join()
    receive_packet_status = rt_return[0]
    # flatten data
    not_all_received = not all(receive_packet_status)
    statuses = [*zip(send_packet_status, receive_packet_status)]

    loss = receive_packet_status.count(False)

    print(f'\nAttempted to send {rounds} packets: '
          f'{send_packet_status.count(True)} packets transmitted, '
          f'{rounds-loss} packets received.')
    print(f'Packets lost: {loss}/{loss/rounds*100}%.')

    if not_all_received:
        logger.warning('crc test: not all packets were received.')

    return {'loss': (loss, loss / rounds),
            'detailed': statuses}


if __name__ == '__main__':
    print(dir())
    print('Run communication/communicator.py for unit tests')
