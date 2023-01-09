# 8-axis-rov

### 介绍
8轴ROV简易控制程序，可实现所有遥控功能


### 环境

1.  Python3
2.  Pymavlink
    具体安装过程参考[Pymavlink](https://www.ardusub.com/developers/pymavlink.html)
3.  其他（包括OpenCV等）

### 使用说明

1.  basicFunction.py: 包含解锁、锁定、平移等基本功能
2.  closeBiBi.py: 每隔3min启动一次所有电机以防止电调因长时间未收到信号发出bibi声
3.  test.py: basicFunction.py 使用示例
4. 其他文件均为basicFunction.py中功能的单独测试

### basicFunction.py

包含一个class：Hyxt，其中包含7个实例方法：
1. arm(self)
2. disArm(self)
3. setRcChannelPWM(self, channel_id: int, pwm: int)
4. startMotor(self, du: int)
5. changeFlightMode(self, mode: str)
6. sendRcTransSignal(self, speed1: int, speed2: int)
7. translation(self, angle: float, speed: int)

*具体功能及参数兼文件内注释*

