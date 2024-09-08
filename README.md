# Fork from ViNT
该工程是开源项目[ViNT](https://github.com/robodhruv/visualnav-transformer)的一个简单的实现，为了和Intel的Hololens2 MR眼镜相互配合工作，我们将该开源项目从ROS1移植到ROS2 foxy版本上，视觉导航模型使用开源的模型参数nomad。
在测试过程中，视觉导航的效果并不很理想。我们添加一系列position sequence作为check point，引导我们的机器人执行这个导航过程，但是由于该position sequence是一个local的position，我们目前还未通过某种方法对global position 和 
local position 进行转换transform，所以该方案对初始的位姿很敏感。我们目前还是摆脱不了对IMU、里程计等传感器的依赖。事实上，基于纯视觉的nvigation还有很多的工作需要完善。

![image](https://github.com/user-attachments/assets/85ebde5b-cf1c-4b47-8a47-eaf5945697d0)


# 视频展示
[基于visualnav-transformer的视觉导航项目](https://www.bilibili.com/video/BV19CpYewENw/?vd_source=20ab5b37b7810aa07786f9a4d46b180a)
