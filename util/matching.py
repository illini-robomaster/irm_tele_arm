class MatchAll:
    def __init__(self, ret=True):
        self.ret = ret

    def __eq__(self, other):
        return self.ret

    def __lt__(self, other):
        return self.ret

    def __le__(self, other):
        return self.ret

    def __eq__(self, other):
        return self.ret

    def __ne__(self, other):
        return self.ret

    def __gt__(self, other):
        return self.ret

    def __ge__(self, other):
        return self.ret

class MatchIn:
    def __init__(self, val):
        self.val = val

    def __eq__(self, other):
        return self.val in other

if __name__ == '__main__':
    print(dir())
