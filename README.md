# APA_QT_Assistant


## 概述
项目主要验证APA的规划和控制算法，主要涉及车辆信息的解码与车辆控制、超声波的车位检测和障碍物定位、泊车轨迹规划和轨迹跟踪等。

## 开发环境

- [qt-5.12(free)](https://download.qt.io/official_releases/qt/5.12/)

  the office realease version for free is v5.12.

- Eigen3 

  ```shell
  $ sudo apt-get install libeigen3-dev"
  ```

- GL

    if need, you will be install gl
    
    ```shell
    $ sudo apt-get install -y mesa-common-dev
    $ sudo apt-get install -y libgl1-mesa-dev
    ```



##  软件功能说明
如下图所示，是目前软件界面的功能划分。从图中可以看出，目前主要分为控制算法、感知检测和轨迹规划三个功能模块。
<img src="https://i.loli.net/2021/08/30/h3eosIvUwYTzJ2E.gif" style="zoom: 67%;" />

## 感知算法
### 超声波重定位
关于超声波的重定位算法采用三角定位数据和数据集的线段拟合算法，进行库位的重定位。
#### 垂直泊车入库过程中的重定位
如下图所示，是垂直泊车入库过程。重定位算法在此过程中采集数据点，当车辆停止后根据采集的数据计算库位的外边沿角点，并拟合前车斜率信息。

![车辆入库过程描述](https://i.loli.net/2021/08/30/mC9TIKJhZdqGfMB.png)

如下图所示，根据实际采集的超声数据进行库位角点的重定位，拟合前车边沿数据，进行前车斜率信息的计算。

<img src="https://i.loli.net/2021/08/30/UpMQTewvg84aiJG.gif" alt="实际数据采集和计算" style="zoom:67%;" />

#### 垂直车位进库重定位
如下图所示，是垂直泊车进库过程。重定位算法在此过程中采集数据点，当车辆停止后根据采集的边沿数据，计算两边车辆的斜率信息。

![](https://i.loli.net/2021/08/30/4jnRvUAaD8HeOSm.png)

如下图所示，根据实际采集的超声波库位两边数据，重新计算库位信息。

<img src="https://i.loli.net/2021/08/30/eid7hEnjwZMOTJU.gif" alt="车辆入库过程描述" style="zoom:67%;" />

### 环视定位

## 轨迹规划算法
### 平行规划
#### 一次入库情况
<img src="https://i.loli.net/2021/08/30/iW7QtNeo9CyxKYJ.gif" alt="一次入库" style="zoom: 80%;" />

#### 多次尝试入库情况
<img src="https://i.loli.net/2021/08/30/pc5fGNVzn4h1KwP.gif" alt="多次尝试入库" style="zoom:80%;" />



### 垂直规划
#### 一次入库情况
<img src="https://i.loli.net/2021/08/30/Eb8gdueCjKaFWPk.gif" alt="一次入库" style="zoom:80%;" />

#### 多次尝试入库情况
![多次尝试入库](https://i.loli.net/2021/08/30/Dp45TdYwSqZzNk9.gif)



## 路径跟踪
### 概述
路径跟踪算法主要分为基于几何、基于后轴、基于前轴和基于模型的优化控制算法。
### 后轴反馈控制
后轴反馈控制原理如下：

![](https://i.loli.net/2021/08/30/YbBKN6kUmxnAuWL.png)

#### 圆弧前进跟踪
![](https://i.loli.net/2021/08/30/AxUkEeVp1ZBI7P4.gif)
#### 圆弧倒车跟踪
![](https://i.loli.net/2021/08/30/12UJCNKTZV5tsro.gif)
