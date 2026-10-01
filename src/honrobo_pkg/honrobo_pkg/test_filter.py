import math
min_ang = 0.0
inc = 0.01745 # 1 degree
count = 0
valid = 0
for i in range(360):
    angle = min_ang + i * inc
    norm_angle = math.atan2(math.sin(angle), math.cos(angle))
    if abs(norm_angle) < math.radians(90.0):
        pass # NaN
    else:
        valid += 1
    count += 1
print(f"Total: {count}, Valid: {valid}")
