'''
 airsim 四旋翼飞圆形
'''
import airsim
import numpy as np
import math
import time
import pyxhook  #监听键盘

def kbevent(event):
    global running, client, speed, height
    #drivetrain = airsim.DrivetrainType.MaxDegreeOfFreedom
    #yaw_mode = airsim.YawMode(False, 0)
    if(event.ScanCode == 80):  # 8 forward
        client.moveByVelocityBodyFrameAsync(1,0,0,1)
    if(event.ScanCode == 88):  # 2 back
        client.moveByVelocityBodyFrameAsync(-1,0,0,1)
    if(event.ScanCode == 83):  # 4 left
        client.moveByVelocityBodyFrameAsync(0,-1,0,1)
    if event.ScanCode == 85:   # 6 right
        client.moveByVelocityBodyFrameAsync(0,1,0,1)
    if event.ScanCode == 79:   # 7 takeoff
        client.takeoffAsync().join()
    if event.ScanCode == 81:   # 9 land
        client.landAsync().join()
    if event.ScanCode == 87:   # 1 unwise clock
        client.rotateByYawRateAsync(-90,  0.1)
    if event.ScanCode == 89:   # 3 wise clock
        client.rotateByYawRateAsync(90, 0.1)
    if event.ScanCode == 82:   # - up
        height = height - 0.5
        client.moveToZAsync(height, 1)
    if event.ScanCode == 86:   # + down
        height = height + 0.5
        client.moveToZAsync(height, 1)
    if event.ScanCode == 91:   # . quit
        running = False

hookman = pyxhook.HookManager()
hookman.KeyDown = kbevent
hookman.HookKeyboard()
hookman.start()

client = airsim.MultirotorClient()  # connect to the AirSim simulator
client.enableApiControl(True)       # 获取控制权
client.armDisarm(True)              # 解锁
client.moveToZAsync(-3, 1).join()   # 第二阶段：上升到2米高度
speed = 1                        # 速度设置
pos_reserve = np.array([[0.], [0.], [-3.]])
state = client.simGetGroundTruthKinematics()
height = state.position.z_val

running = True
while running:
    time.sleep(0.1)

hookman.cancel()
client.armDisarm(False)             # 上锁
client.enableApiControl(False)      # 释放控制权
'''
 # 速度控制
for i in range(2000):
     # 获取无人机当前位置
     state = client.simGetGroundTruthKinematics()
     pos = np.array([[state.position.x_val], [state.position.y_val], [state.position.z_val]])
     # 计算径向速度的方向向量
     dp = pos[0:2] - center
     #np.linalg.norm求的是二范数，正数
     if np.linalg.norm(dp) - radius > 0.1:
         vel_dir_1 = -dp
     elif np.linalg.norm(dp) - radius < 0.1:
         vel_dir_1 = dp
     # 计算切向速度的方向向量
     theta = math.atan2(dp[1, 0], dp[0, 0])
     if clock_wise:
         theta += math.pi / 2
     else:
         theta -= math.pi / 2
     v_dir_2 = np.array([[math.cos(theta)], [math.sin(theta)]])
     # 计算最终速度的方向向量
     v_dir = 0.08 * vel_dir_1 + v_dir_2
     # 计算最终速度指令
     v_cmd = speed * v_dir/np.linalg.norm(v_dir)
     # 速度控制
     drivetrain = airsim.DrivetrainType.ForwardOnly
     yaw_mode = airsim.YawMode(False, 90)
     client.moveByVelocityZAsync(v_cmd[0, 0], v_cmd[1, 0], -3, 1, drivetrain=drivetrain, yaw_mode=yaw_mode)
     # 画图
     point_reserve = [airsim.Vector3r(pos_reserve[0, 0], pos_reserve[1, 0], pos_reserve[2, 0])]
     point = [airsim.Vector3r(pos[0, 0], pos[1, 0], pos[2, 0])]
     point_end = pos + np.vstack((v_cmd, np.array([[0]])))
     point_end = [airsim.Vector3r(point_end[0, 0], point_end[1, 0], point_end[2, 0])]
     client.simPlotArrows(point, point_end, arrow_size=8.0, color_rgba=[0.0, 0.0, 1.0, 1.0])
     client.simPlotLineList(point_reserve+point, color_rgba=[1.0, 0.0, 0.0, 1.0], is_persistent=True)
     # 循环
     pos_reserve = pos
     time.sleep(0.02)
'''













