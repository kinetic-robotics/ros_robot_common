from enum import IntEnum, unique

@unique
class ColorType(IntEnum):
    # 红蓝主色
    RED_BLUE  = 0
    # 黄色
    YELLOW    = 1
    # 绿色
    GREEN     = 2
    # 橙色
    ORANGE    = 3
    # 紫红色
    FUCHSIA   = 4
    # 粉色
    PINK      = 5
    # 青色
    CYAN_BLUE = 6
    # 黑色
    BLACK     = 7
    # 白色
    WHITE     = 8