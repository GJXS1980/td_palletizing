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
