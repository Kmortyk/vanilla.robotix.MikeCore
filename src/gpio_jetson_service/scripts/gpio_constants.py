import enum


class Command(enum.Enum):
    FULL_STOP = (0, 'full stop')
    LEFT_STOP = (1, 'left stop')
    RIGHT_STOP = (2, 'right stop')
    FORWARD_LOW = (3, 'forward low')
    FORWARD_MIDDLE = (4, 'forward middle')
    FORWARD_FAST = (5, 'forward fast')
    BACKWARD_LOW = (6, 'backward low')
    BACKWARD_MIDDLE = (7, 'backward middle')
    BACKWARD_FAST = (8, 'backward fast')
    LEFT_FORWARD_LOW = (9, 'left forward low')
    LEFT_FORWARD_MIDDLE = (10, 'left forward middle')
    LEFT_FORWARD_FAST = (11, 'left forward fast')
    LEFT_BACKWARD_LOW = (12, 'left backward low')
    LEFT_BACKWARD_MIDDLE = (13, 'left backward middle')
    LEFT_BACKWARD_FAST = (14, 'left backward fast')
    RIGHT_FORWARD_LOW = (15, 'right forward low')
    RIGHT_FORWARD_MIDDLE = (16, 'right forward middle')
    RIGHT_FORWARD_FAST = (17, 'right forward fast')
    RIGHT_BACKWARD_LOW = (18, 'right backward low')
    RIGHT_BACKWARD_MIDDLE = (19, 'right backward middle')
    RIGHT_BACKWARD_FAST = (20, 'right backward fast')

    def __init__(self, id, title):
        self.id = id
        self.title = title


class Motor(enum.Enum):
    left = (0, 'Left')
    right = (1, 'Right')

    def __init__(self, id, title):
        self.id = id
        self.title = title


class Direction(enum.Enum):
    forward = (0, 'Forward')
    backward = (1, 'Backward')

    def __init__(self, id, title):
        self.id = id
        self.title = title


class Speed(enum.Enum):
    low = (0, 'Low')
    middle = (1, 'Middle')
    fast = (2, 'Fast')

    def __init__(self, id, title):
        self.id = id
        self.title = title


class PinDirection(enum.Enum):
    d_in = (0, 'In')
    d_out = (1, 'Out')

    def __init__(self, id, title):
        self.id = id
        self.title = title
