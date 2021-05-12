'''
 airsim 飞行器控制
'''
import threading
import airsim
import numpy as np
import math
import time
import pyxhook  #监听键盘

class FlightControl():
    def __init__(self):
        #开启键盘监听
        self.hookman = pyxhook.HookManager()
        self.hookman.KeyDown = self.kbevent
        self.hookman.HookKeyboard()
        self.hookman.start()

        #连接AirSim
        self.client = airsim.MultirotorClient()  # connect to the AirSim simulator
        self.client.enableApiControl(True)       # 获取控制权
        self.client.armDisarm(True)              # 解锁
        #client.moveToZAsync(-3, 1).join()   # 第二阶段：上升到2米高度

        self.running = True
        self.isCircle = False

        self.height = 0.0
        self.speed = 1.0
        self.center = np.array([[0.], [0.], [0.]])
        self.startpoint = np.array([[0.], [0.], [0.]])
        self.radius = 0.0
        self.clock_wise = True


    def run(self):
        while(self.running):
            if(self.isCircle):
                T = threading.Thread(target=self.flightcircle)
                T.start()
            self.isCircle = False   
            time.sleep(0.1)
        self.hookman.cancel()
        self.client.armDisarm(False)             # 上锁
        self.client.enableApiControl(False)      # 释放控制权

    def kbevent(self, event):
        if(event.ScanCode == 80):  # 8 forward
            self.client.moveByVelocityBodyFrameAsync(1,0,0,1)
        if(event.ScanCode == 88):  # 2 back
            self.client.moveByVelocityBodyFrameAsync(-1,0,0,1)
        if(event.ScanCode == 83):  # 4 left
            self.client.moveByVelocityBodyFrameAsync(0,-1,0,1)
        if event.ScanCode == 85:   # 6 right
            self.client.moveByVelocityBodyFrameAsync(0,1,0,1)
        if event.ScanCode == 79:   # 7 takeoff
            self.client.takeoffAsync().join()
            state = self.client.simGetGroundTruthKinematics()
            self.height = state.position.z_val
            print("takeoff finished.")
        if event.ScanCode == 81:   # 9 land
            self.client.landAsync().join()
            print("land finished.")
        if event.ScanCode == 87:   # 1 unwise clock
            self.client.rotateByYawRateAsync(-90,  0.1)
        if event.ScanCode == 89:   # 3 wise clock
            self.client.rotateByYawRateAsync(90, 0.1)
        if event.ScanCode == 82:   # - up
            self.height = self.height - 0.3
            self.client.moveToZAsync(self.height, 0.5)
        if event.ScanCode == 86:   # + down
            self.height = self.height + 0.3
            self.client.moveToZAsync(self.height, 0.5)
        if event.ScanCode == 106:   # / get center
            state = self.client.simGetGroundTruthKinematics()
            self.center[0, 0] = state.position.x_val
            self.center[1, 0] = state.position.y_val
            self.center[2, 0] = state.position.z_val
            self.height = state.position.z_val
            print("center positon is: ", self.center)
        if event.ScanCode == 63:   # / get startpoint
            state = self.client.simGetGroundTruthKinematics()
            self.startpoint[0, 0] = state.position.x_val
            self.startpoint[1, 0] = state.position.y_val
            self.startpoint[2, 0] = state.position.z_val
            self.height = state.position.z_val
            print("startpoint positon is: ", self.startpoint)
        if event.ScanCode == 104:   # Enter start flight circle
            self.radius = np.linalg.norm(self.startpoint[0:2] - self.center[0:2])
            print("current radius is: ", self.radius)
            if(self.radius < 1.0):
                print("current radius is too small, please set center and startpoint again !")
            else:
                self.isCircle = True
        if event.ScanCode == 91:   # . quit
            self.running = False

    def flightcircle(self):
        print("enter the thread of flightcircle.")
        while(True):
            # 获取无人机当前位置

            state = self.client.simGetGroundTruthKinematics()
            pos = np.array([[state.position.x_val], [state.position.y_val], [state.position.z_val]])
            # 计算径向速度的方向向量
            dp = pos[0:2] - self.center[0:2]
            #np.linalg.norm求的是二范数，正数
            if np.linalg.norm(dp) - self.radius > 0.5:
                vel_dir_1 = -dp
            elif np.linalg.norm(dp) - self.radius < 0.5:
                vel_dir_1 = dp
            # 计算切向速度的方向向量
            theta = math.atan2(dp[1, 0], dp[0, 0])
            if self.clock_wise:
                theta += math.pi / 2
            else:
                theta -= math.pi / 2
            v_dir_2 = np.array([[math.cos(theta)], [math.sin(theta)]])
            # 计算最终速度的方向向量
            #0.01这个参数需要自己调整以便使得飞行的圆轨迹更加丝滑
            v_dir = 0.01 * vel_dir_1 + v_dir_2
            # 计算最终速度指令
            v_cmd = self.speed * v_dir/np.linalg.norm(v_dir)
            # 速度控制
            drivetrain = airsim.DrivetrainType.ForwardOnly
            yaw_mode = airsim.YawMode(False, 90)
            #self.client.moveByVelocityZAsync(1, 0, self.height, 1, drivetrain=drivetrain, yaw_mode=yaw_mode)
            self.client.moveByVelocityZAsync(v_cmd[0, 0], v_cmd[1, 0], self.height, 1, drivetrain=drivetrain, yaw_mode=yaw_mode).join
            #self.client.moveByVelocityZAsync(v_cmd[0, 0], v_cmd[1, 0], self.height, 1)
            # 画图
            '''
            point_reserve = [airsim.Vector3r(self.startpoint[0, 0], self.startpoint[1, 0], self.startpoint[2, 0])]
            point = [airsim.Vector3r(pos[0, 0], pos[1, 0], pos[2, 0])]
            point_end = pos + np.vstack((v_cmd, np.array([[0]])))
            point_end = [airsim.Vector3r(point_end[0, 0], point_end[1, 0], point_end[2, 0])]
            self.client.simPlotArrows(point, point_end, arrow_size=8.0, color_rgba=[0.0, 0.0, 1.0, 1.0])
            self.client.simPlotLineList(point_reserve+point, color_rgba=[1.0, 0.0, 0.0, 1.0], is_persistent=True)
            # 循环
            pos_reserve = pos
            '''
            time.sleep(0.02) 
            if(self.running == False):
                break

if __name__ == '__main__':

    FC = FlightControl()
    FC.run()














