# to calibrate
# just before robot first starts moving

gs.mode = 'GYRO-RATE'
gs.mode = 'GYRO-ANG'
while (not(gs.value() = 0)):
    pass
    
    
# to reset gyro to 0

gs.mode = 'GYRO-ANG'
