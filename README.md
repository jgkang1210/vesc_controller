# vesc_controller

## repository for vesc velocity controller

- Contains vesc firmware description
- ROS2 based velocity publisher

vesc tool 링크

vesc tool free

https://vesc-project.com/vesc_tool


vesc firmware 원본

https://github.com/vedderb/bldc

### Prerequisites

Install the gcc-arm-embedded toolchain. Recommended version gcc-arm-none-eabi-7-2018-q2

1. link : https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

2. Locate and Download version gcc-arm-none-eabi-7-2018-q2 for your machine

Linux 64-bit version downloads click : https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2018q2/gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2?revision=bc2c96c0-14b5-4bb4-9f18-bceb4050fee7?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,7-2018-q2-update

3. Unpack the archive in the file manager by right-clicking on it and select "extract here"

4. Change directory to the unpacked folder, unpack it in /usr/local by execute the following command

```
cd gcc-arm-none-eabi-7-2018-q2-update-linux  
sudo cp -RT gcc-arm-none-eabi-7-2018-q2-update/ /usr/local
```

## ROS2 setting

```
cd mc_ros2_ws
colcon build
source install/setup.bash
ros2 run mc_vesc mc_vesc
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

cpp  파일에서 uart 통신을 위한 디바이스 세팅 가능.

## Bin file

```
cd vesc_firmware
make build/VESC_fw.bin
```

이 후 VESC tool 에서 오른쪽 메뉴 firmware 에서 custom file 에서 해당 bin file 불러옴.

conf_general.h 파일에서 fw 버전을 설정할 수 있다.
현재 5.2 version

custom app 에서 하나 문제 였던 점, 타임아웃을 무한대로 해두어서 문제가 발생했었다.
적당히 작은 시간으로 해주어야 한다.

## FOC contorl setting

Configuration file 을 받아오는 방법

```
cd conf
```

하고 나면 두가지 config file들 볼 수 있다.

![](/img/setting1.png)

![](/img/setting2.png)

---

잘 안되면 그냥 직접 캘리브레이션

Wizards -> Setup Motor FOC

Medium outrunner

battery settings : Not done


setup ->  Run detection and wait

Direction 에서 forward 혹은 reverse 로 체크

---
Motor setting

motor settings/general/general/Sensor port mode
Encoder : AS5047 Encoder

additional info/Setup/Motor poles

Poles : 40

---

APP setting 에서는 

general 에서 app to use custom user app
PPM 에서  pid speed control

