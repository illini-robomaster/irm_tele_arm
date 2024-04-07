import signal


class Timeout(Exception):
    pass


class TimedOut():
    pass


def alarm_handler(signum, frame):
    raise Timeout

def timedinput(timeout, prompt='', default=TimedOut()):
    # set signal handler
    signal.signal(signal.SIGALRM, alarm_handler)
    signal.alarm(timeout) # produce SIGALRM in `timeout` seconds

    try:
        response = input(prompt)
    except Timeout:
        print()
        response = default
    finally:
        signal.alarm(0) # cancel alarm

    return response

if __name__ == '__main__':
    print('Waiting one second for response.')
    a = timedinput(1, '=> ')
    print('Your response:', a)
