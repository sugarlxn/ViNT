#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import cv2

from PIL import Image as PILImage
from PIL import ImageDraw

# 读取naction41.csv文件
df = pd.read_csv('/root/vint_ws/ros_ws/scripts/vis/naction35.csv')

#第一列为x，第二列为y
x = df.iloc[:,0]
y = df.iloc[:,1]
print(x)
print(y)
# 访第一个元素
print(df.iloc[0][0])

# #绘制散点图 
# plt.scatter(y, x)
# plt.show()
# #保存图片
# plt.savefig('/root/vint_ws/ros_ws/scripts/vis/naction35.png')

# img = cv2.imread('/root/vint_ws/ros_ws/topomaps/images/topomap_loop7/49.png')
# print(img.shape)
# max_y,max_x = img.shape[0],img.shape[1] # 长，宽
# # 在图片上画散点图，参数分别为图片，x坐标，y坐标，颜色，大小
# for i in range(len(x)):
#     cv2.circle(img, (int((df.iloc[i][1]+5)*max_x/10),max_y - int(df.iloc[i][0]*max_y/10)), 5, (0, 0, 255), -1)

# # 保存图片
# cv2.imwrite('/root/vint_ws/ros_ws/scripts/vis/naction35.jpg', img)

# 使用PIL 读取一张图片
image = PILImage.open("/root/vint_ws/ros_ws/topomaps/images/topomap_loop7/49.png")

imagedraw = ImageDraw.Draw(image)

imagedraw.point((20,20),(255,0,0))
image.show()
image.save("testPIL.jpg")