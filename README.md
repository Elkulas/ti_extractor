# ti_extractor

本ros包用于解thermal数据以及imu数据,其中thermal的topic为/optris/thermal_image, imu的topic为/imu/data.

## Usage
```
rosrun ti_extractor ti_extract out_root=/home/jjj/NGCLAB/catkin_ws/src/ti_extractor/data
```

其中out_root是最后输出的文件夹的位置,thermal是八位的thermal数据,thermalRaw是原始16位数据,thermalSpec是thermal数据加上DFT变换之后的数据
