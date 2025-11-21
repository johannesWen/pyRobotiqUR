from enum import IntEnum


class GripperStatus(IntEnum):
    RESET = 0
    ACTIVATING = 1
    UNUSED = 2
    ACTIVE = 3


class ObjectStatus(IntEnum):
    MOVING = 0
    STOPPED_OUTER_OBJECT = 1
    STOPPED_INNER_OBJECT = 2
    AT_DEST = 3
