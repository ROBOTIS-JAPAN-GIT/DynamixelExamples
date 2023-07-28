# DynamixelExamples

## Expected environment

### Directory hierarchy
The following environment is assumed.
```
catkin_ws/
    |- DynamixelSDK_jp_custom
    |- realsense-ros_jp_custom
    └- DynamixelExamples
```

## Examples

- `DynamixelPanTilt/`
- `other_example_to_be_released_later/`


### DynamixelPanTilt

- `pantilt_bringup` : is used for starting up a real machine.
- `pantilt_gazebo` : is used for simulating in gazebo.
- `pantilt_description` : defines 3D model for gazebo simulation or visualization.
- `pantilt_control` : is used for controlling pantilt.
  
#### usage in simulation

```
$ roslaunch pantilt_gazebo simulation_with_coke.launch
```

```
$ roslaunch pantilt_control tracking_target_color.launch
```

#### usage in real world

```
$ roslaunch pantilt_bringup {comming soon}
```

```
$ roslaunch pantilt_control tracking_target_color.launch
```

#### WSL2越しにU2D2と接続するために
以下のコマンドでU2D2が刺さっているUSBを特定して，`BUSID`を控える
```power shell
PS C:\Windows\system32> usbipd wsl list
```

以下のコマンドでWSLが認識するようにする
```power shell
PS C:\Windows\system32> usbipd wsl attach --distribution {Linux Distro} --busid {BUSID}
```
上記コマンドの反対で，WSLとの接続を解除する
```power shell
PS C:\Windows\system32> usbipd wsl detach --busid {BUSID}
```

以下のコマンドなどでwsl側からUSBデバイスが見えていることを確認する
```wsl
$ ls /dev/tty* -la
$ lsusb
$ dmesg
```

以下のコマンドでUSBデバイスに権限を与える <- ここで詰まった．
```wsl
$ sudo chmod 777 /dev/ttyUSB0 
```

参考
https://qiita.com/baggio/items/28c13ed8ac09fc7ebdf1#usb%E3%83%87%E3%83%90%E3%82%A4%E3%82%B9%E3%81%AE%E6%8E%A5%E7%B6%9A%E6%96%B9%E6%B3%95
ただし，最後のWSL kernelのBuildは不要だった．
