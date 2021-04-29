# 实时射击

Header header

# 子弹类型
# 17mm类型
uint8 BULLET_TYPE_17MM = 1
# 42mm类型
uint8 BULLET_TYPE_42MM = 2
uint8 bulletType

# 发射机构类型
# 1号17mm发射机构
uint8 SHOOTER_FIRST_17MM = 1
# 2号17mm发射机构
uint8 SHOOTER_SECOND_17MM = 2
# 42mm发射机构
uint8 SHOOTER_FIRST_42MM = 3
uint8 shooterID

# 射频,单位hz
int32 hz
# 射速,单位m/s
float64 speed
