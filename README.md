# 8-axis-rov

### 介绍
+ 8轴ROV简易控制程序接口，可使用程序控制机器人全向移动
+ 配合配套上位机可实现所有遥控功能及图传等扩展功能（尚未开发完成）


### 环境
*飞控使用Ardusub固件*
1.  Python3
2.  Pymavlink
    具体安装过程参考[Installation](https://www.ardusub.com/developers/pymavlink.html)
3.  Socket（用于进行和上位机间的通信）
4.  其他（包括OpenCV等）

### 文件说明

1.  basicFunction.py: 包含解锁、锁定、平移等基本功能
2.  closeBiBi.py: 每隔3min启动一次所有电机以防止电调因长时间未收到信号发出bibi声
3.  socketServer.py: 机器人的Socket服务端程序，用于接收上位机发送的控制信息
3.  test.py: basicFunction.py 使用示例
4. 其他文件均为basicFunction.py中功能的单独测试

### basicFunction.py

包含一个class：Hyxqt，其中包含1个构造方法和9个实例方法：
1. arm(self)
2. disArm(self)
3. setRcChannelPWM(self, channel_id: int, pwm: int)
4. startMotor(self, du: int)
5. changeFlightMode(self, mode: str)
6. sendRcTransSignal(self, speed1: int, speed2: int)
7. translation(self, angle: float, speed: int)
8. rotation(self, dir: str, speed: int)
9. floatingAndDiving(self, dir: str, speed: int)

### socketServer.py

功能为启动Socket服务端，接收上位机的控制信息，包含1个构造方法和2个实例方法：
1. getData(self)
2. sendData(self)

*具体功能及参数见文件内注释*

### 示例程序

使用程序控制：

``` py
from basicFunction import Hyxqt

if __name__ == '__main__':
    xqt = Hyxqt('/dev/ttyACM0', 115200)     #创建实例并建立连接
    xqt.arm()                               #解锁电机
    xqt.changeFlightMode("MANUAL")          #设置飞行模式为手动模式
    while True:
        xqt.translation(45, 200)            #向右前方45°以200速度平移
        # xqt.rotation('right', 100)        #向右以100速度自转
        # xqt.floatingAndDiving('up', 200)  #以200速度上浮

    # xqt.disArm()                          #锁定电机
```

使用上位机控制：
```py
from basicFunction import Hyxqt
from socketServer import SocketServer

if __name__ == '__main__':
    sock = SocketServer("localhost", 8001)  #启动Socket服务器
    xqt = Hyxqt('/dev/ttyACM0', 115200)     #创建实例并建立连接
    xqt.arm()                               #解锁电机   
    xqt.changeFlightMode("STABILIZE")       #设置飞行模式为自稳模式
    
    while True:
        xqt.controlViaUpperComputer(sock.getData())
        
    # xqt.disArm()

```

*可使用上位机按键切换控制模式*

### closeBiBi.py

功能为每隔一段时间启动所有电机，防止电调因长时间接收不到信号发出响声，可独立运行。


