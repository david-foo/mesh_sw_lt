#use zephyr


## install requirements on ubuntu16
- sudo apt-get install --no-install-recommends git cmake ninja-build gperf  ccache dfu-util device-tree-compiler wget  python3-pip python3-setuptools python3-wheel xz-utils file make gcc  gcc-multilib

- pip3 install --user cmake



## install SDK/toolchain
wget https://github.com/zephyrproject-rtos/meta-zephyr-sdk/releases/download/0.9.3/zephyr-sdk-0.9.3-setup.run

./zephyr-sdk-0.9.3-setup.run install it to /your/path (example /home/david/zephyrsdk-0.9.3)


## clone zephyr code
git clone https://github.com/zephyrproject-rtos/zephyr.git

git checkout tags/v1.12.0

## clone mesh demo code
git clone this repo

## compile
- export ZEPHYR_GCC_VARIANT=zephyr
- export ZEPHYR_SDK_INSTALL_DIR=/home/david/zephyrsdk-0.9.3/


- cd zephyr, **. setzephyrsdk.sh setup env**
- cd blue_mesh_demo/mesh-lt/, **mkdir build && cd build**
- **cmake -GNinja -DBOARD=bbc_microbit ..**
- **ninja**

the hex is blue_mesh_demo/mesh-lt/build/zephyr/zephyr.hex


## What the sw(switch) and lt(light) do?
sw contains a gen on/off server and lt contains a gen on/off client. press A and B on sw will excute genericOnOffSetUnAck or genericOnOffGet to lt, and lt will print msg via its serial port. 

first all, we neet to use meshctl(from bluez) or nRFmesh app to do provition and configure to let the client to pub on a address and let the server to sub this address.




