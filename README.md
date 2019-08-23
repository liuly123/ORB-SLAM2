### 编译

```sh
git clone https://github.com/raulmur/ORB_SLAM2
cd ORB_SLAM2
./build.sh
#依赖：Pangolin、OpenCV、Eigen3
#DBoW2和g2o（ORB-SLAM自带）
```

### 单目测试

```sh
#下载TMU数据集https://vision.in.tum.de/data/datasets/rgbd-dataset/download
cd ~/ORB_SLAM
./Examples/Monocular/mono_tum ./Vocabulary/ORBvoc.txt ./Examples/Monocular/TUM1.yaml ~/dataset/rgbd_dataset_freiburg1_xyz
```

### 双目测试

```sh
#EuRoC数据集https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
cd ORB-SLAM
./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/dataset/mav0/cam0/data ~/dataset/mav0/cam1/data ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt
```



### RGB-D测试

```sh
#下载TUM数据集https://vision.in.tum.de/data/datasets/rgbd-dataset/download
cd ~/ORB_SLAM2
./Examples/RGB-D/rgbd_tum ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/TUM2.yaml ~/dataset/rgbd_dataset_freiburg2_pioneer_360 ~/dataset/rgbd_dataset_freiburg2_pioneer_360/associate.txt
```

