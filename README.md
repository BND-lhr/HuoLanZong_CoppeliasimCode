# HuoLanZong_CoppeliasimCode（Lua/Python in Coppeliasim and Python Remote）B站ID：火兰宗_烂好人
Coppeliasim code of videos(from Bilibili)
与B站视频内容相关的代码，会陆续上传
## 所使用的Coppeliasim（V-rep）版本如下：
* 1.windows ：4.6.0 / 4.7.0（由于4.7.0的一些API更新，自4月起已全面更新为4.7.0版本）
* 2.Ubuntu 22.04 ：4.7.0
* 3.python 3.11.0
## 手眼标定项目
### 在此感谢大佬**hong3731**的帮助 可以关注她的主页：<https://blog.csdn.net/hong3731>
* 1.场景基于UR5机械臂、深度相机和棋盘标定板搭建，存放于**scene**中的UR5_eyeinhand.ttt
* 2.标定版checkerboard.jpg、关节角度信息和末端姿态信息csv文件存放于**data**中
* 3.标定板生成网站：<https://calib.io/pages/camera-calibration-pattern-generator>
* 4.**hong3731**的手眼标定代码在<https://github.com/hong3731/Handeyecalibration>
