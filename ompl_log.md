# 配置过程

首先参考一篇博客[CSDN:
ZPC8210 MoveIt!运动规划库OMPL和路径规划算法](https://blog.csdn.net/ZPC8210/article/details/143575784)
该文章提供了OMPL相关的配置过程，包含一些相关的配置教程。

首先是说明了运动规划的过程：

- 机器人抽象到状态空间，利用moveit将机器人的URDF文件抽象到构型空间
- 调用几大运动规划库（前处理、规划、后处理）自动生成机器人的运动轨迹
- 运动规划器通过插件机制加载到moveit中（通过Planning Pipeline定义规划器接口）
- Pipeline通过planning scene导入octomap作为机器人环境信息

## 构建
