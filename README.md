# HuoLanZong_CoppeliasimCode（Lua/Python in Coppeliasim and Python Remote）B站ID：[火兰宗_烂好人](https://space.bilibili.com/253086292?spm_id_from=333.1007.0.0)
Coppeliasim code of videos(from Bilibili)
与B站视频内容相关的代码，会陆续上传
## 所使用的Coppeliasim（V-rep）版本如下：
* 1.windows ：4.6.0 / 4.7.0（由于4.7.0的一些API更新，目前已更新为4.7.0版本）
* 2.Ubuntu 22.04 ：4.7.0
* 3.python 3.11.0
## 手眼标定项目
### 手动设计机械臂位姿，以采集标定板数据用于后续标定
https://github.com/user-attachments/assets/65124cf4-8029-4f67-9952-a5bc790212f4
### Reference 感谢大佬[hong3731](https://blog.csdn.net/hong3731)的基础思路方案，这是她的[手眼标定代码](https://github.com/hong3731/Handeyecalibration)
* 1.场景基于UR5机械臂、深度相机和棋盘标定板搭建，存放于**scene**中的UR5_eyeinhand.ttt
* 2.标定版checkerboard.jpg、关节角度信息和末端姿态信息csv文件存放于**data**中
* 3.标定版图片可以在[标定板生成网站](https://calib.io/pages/camera-calibration-pattern-generator)生成并保存
* 4.在UR5_eyeinhand.ttt中进行关节和姿态的数据制作和保存时，请**打开(勾选)**UR5下的Script的enabled使能属性，在coppeliasim中开启仿真时，出现UI界面说明自定义脚本**正常运作**
* 5.在运行calibrate_hand_eye.py时，请**关闭**上述Script的enabled使能属性，防止保存的关节和姿态数据被清空
## 关于其他py文件
* IK_robotarm.py和main.py为B站视频配套相关代码，均在视频中出现过
## 关于配置文件，最好根据自己的Coppeliasim版本自行替换（本项目中所使用为4.7.0版本）
* depth_image_encoding.py,sim.py,simConst.py为python进行远程连接仿真环境的配置文件，在CoppeliaSim_V4_7_0/programming/legacyRemoteApi/remoteApiBindings/python中可以找到，复制到项目**根目录**下即可
* remoteApi.so/remoteApi.dll为远程连接的动态链接库，**.so**在ubuntu系统下使用，**.dll**在windows系统下使用，其位置在CoppeliaSim_V4_7_0/programming/legacyRemoteApi/remoteApiBindings/lib/lib/Ubuntu以及～/Windows下找到，复制到项目**根目录**即可
* coppeliasim_zmqremoteapi_client文件为运行手眼标定的配置文件，在CoppeliaSim_V4_7_0/programming/zmqRemoteApi/clients/python/src中可以找到，同样复制文件夹到项目**根目录**即可
