### 生成deb功能包
在功能包目录下，运行下面指令：
```
bloom-generate rosdebian --os-name ubuntu --ros-distro melodic

fakeroot debian/rules binary
```
### 使用教程
修改串口端口（默认：/dev/ttyUSB0），修改mqtt的ip和端口
```bash
roslaunch td-palletizing TD_Palletizing.launch

roslaunch td-palletizing ZY_TD.launch
```

### 说明
0x00： 坐标控制
0x01： 当前位置
0x02： 开始
0x03： 暂停
0x04： 回原点
0x05： 吸
0x06： 放
0x07： 复位x
0x08： 复位y

a: x-
d: x+
w: y+
s: y-
q: z+
e: z-
f: 吸/放
1： x复位
2： y复位
3： 退出键盘控制

/Pall_Running_Topic： 岸吊开始（1）和暂停（0）控制话题

/Pall_Grasp_Topic： 爪的吸（1）和放（0）控制

/Pall_Reset_Topic： 复位（1）

/Pall_POS_SET： 坐标控制[x,y,z,u]

/Pall_GRAB_STATUS: 发布抓取状态

/Pall_CURR_POS: 发布当前位姿话题








