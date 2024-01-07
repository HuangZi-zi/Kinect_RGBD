# Kinect_RGBD
使用Kinect深度相机的特征提取

## 1208
完成相机图像数据的读出

## 1209
删除了部分不需要的函数，修改了深度信息读取的流程，增加了每一步的成功判断。修改了16位图亮度显示不正常的问题。

## 1220
增加了保存截图的程序。修复了深度图像亮度断层的问题。

## 1228
进行了一次大版本更新，使用了PCL，预期读取点云数据，暂时未测试。之前的程序已全部删除。参考文献：https://blog.csdn.net/weixin_43220219/article/details/129907863

## 1229
改为使用vcpkg安装的opencv。测试了1228的程序。增加了滤波部分，滤除点云中的离群点，滤波部分暂时没有运行成功。

## 1230
增加了基于邻域生长法的平面检测程序。发现opencv和pcl中的flann会冲突报错，因此把pcl的配置放在前，opencv的配置放在后。程序可以编译，未测试运行。1229的滤波部分暂时没有运行成功，考虑是计算机运行速度限制。新增了点云保存功能，用于测试滤波部分的代码。

## 0103
修改代码为使用multisource而非分别读取彩色帧和深度帧。将部分常用功能封装成函数。完成了滤波部分的测试。完成了邻域生长法分割的测试。目前的问题在于随机数生成的颜色造成视频识别结构色彩闪烁，考虑通过修改库文件实现。

## 0104
定义了新的函数，避免了色彩闪烁的问题。完成了邻域生长法识别立方体的实验，分割效果较好，但运算速度过慢。与李赫同学交流，连通域算法，邻域一次性生长多个点。

## 0105
测试了多线程，但目前仍存在问题。完成了聚类分析部分的编程，还未完成测试。邻域生长法在点云数量小的情况下无法很好分割目标，需要考虑其他方法。重置了项目文件，使得gitignore生效。

## 0106
完成了聚类分析部分的测试，问题在于无法完全指定提取出的是不是地面。新思路：利用红外传感器特性，在无数据点处存在边界，可以简化线提取。完成了深度图中边界提取的编程和测试，但输出结果意义不明显，可能需要调参。问题：深度图可视化效果差，窗口太小不可调节。

## 0107
新增了根据色彩进行邻域生长的代码，目前未通过测试。测试了以原图生成深度图，效果不理想。考虑进一步加强邻域生长法，从底部中心开始，向外生长，标线处颜色变化即停止。