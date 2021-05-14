'''
 利用AirSim生成的数据构建SMAKE所需要的训练数据
'''
import numpy as np
import math
import time
import os
import cv2

'''
    这个class的目的是读取AirSim中的图像和关于图像的信息文件airsim_rec.txt
    将其转化为适合SMAKE训练的图像命名方式和labels
    params：
        self.airsim_data_path --- AirSim点击record所生成数据的文件目录
        self.train_images_path --- 训练数据之图像数据的保存目录
        self.train_labels_path --- 训练数据之label数据的保存目录
        self.start = 5 --- 这是对image和label命名的起始数字 比如 5 代表着image和label从 000005 开始命名

    process_rec 这个函数的目的是根据输入的list数据转换成SMAKE训练的label数据，返回值是使用空格字符串
'''

class DatasetCreation():
    def __init__(self):
        self.airsim_data_path = "/home/freddy/Documents/AirSim/test"
        self.train_images_path = "/home/freddy/Documents/AirSim/test/trainimages"
        self.train_labels_path = "/home/freddy/Documents/AirSim/test/labels"
        self.start = 5
    def read_airsim_data_and_save(self):
        if(os.path.exists(self.airsim_data_path)):
            airsim_images_path = self.airsim_data_path + '/images'
            airsim_rec = self.airsim_data_path + '/airsim_rec.txt'
            imgs = os.listdir(airsim_images_path)
            print(imgs)
            i = 0
            f_read = open(airsim_rec,"r")
            for str_line in f_read:
                line = str_line.split('	')
                print(line[0])
                if(line[0] != "SimpleFlight"):
                    continue
                #处理rec中的数据并写入统一的训练label
                post_line = self.process_rec(line)
                f_save = open(self.train_labels_path+'/'+str(i+self.start).zfill(6)+'.txt', 'w')
                f_save.write(post_line)
                f_save.write("\n")
                f_save.close()
                #读取图像并重命名保存
                img = cv2.imread(airsim_images_path + '/' + imgs[i])
                cv2.imwrite(self.train_images_path+'/'+str(i+self.start).zfill(6)+'.png', img)
                i = i + 1
            f_read.close()
        else:
            print("airsim_data_path do not exist.")

    def process_rec(self, line):
        return "1.1 2.2 3.3"

if __name__ == '__main__':
    DC = DatasetCreation()
    DC.read_airsim_data_and_save()
    










