---
# Jekyll 'Front Matter' goes here. Most are set by default, and should NOT be
# overwritten except in special circumstances. 
# You should set the date the article was last updated like this:
date: 2021-12-02 # YYYY-MM-DD
# This will be displayed at the bottom of the article
# You should set the article's title:
title: Vision System Setup Readme
# The 'title' is automatically displayed at the top of the page
# and used in other parts of the site.
---

## Overview

In this readme, we will do the following things for vision system:

- Set up 2D YOLO v3 with ROS (Required for the whole system compilation and demo)
- Set up GPU for YOLO (Better to have)
- Set up 3D YOLO with ROS and surface normal (Optional)
- Run goal getter and integrate whole vision system (Required)

---
## Integrating 2D YOLO with ROS (Required for compiling and running our full use case)

To install YOLO in ROS, we will use a YOLO ROS wrapper GitHub repository [darknet_ros](https://github.com/leggedrobotics/darknet_ros). You can simply follow their instructions in the README or follow the instructions below. 

Before you start the integration, make sure you have prepared your pre-trained YOLO model weights and configurations. Based on the detection task, the pre-trained model weights may differ. If your task requires objects that are not included in the default YOLO dataset (which uses [VOC](https://pjreddie.com/projects/pascal-voc-dataset-mirror/) or [COCO](https://cocodataset.org/#home) dataset to train), you will need to search for other pre-trained open-source projects and download their model weights and configurations to your local machine. Otherwise, you would need to train YOLO from scratch with your own dataset. The details will not be included in this article, but you may find this article helpful in learning how to do so: [Tutorial](https://blog.roboflow.com/training-yolov4-on-a-custom-dataset/)

### Requirements

- Ubuntu: 18.04

- ROS: Melodic

- OpenCV: 3.4.6 (if using OpenCV 4+, follow this [link](https://github.com/leggedrobotics/darknet_ros/issues/290#issuecomment-762345858))

- CUDA: 10.2 

- cuDNN: 7.6.x

- numpy: 1.19.0

- If use NVIDIA Jetson, JetPack 4.3 will automatically install CUDA, cuDNN for you (default: CUDA 10.0 + cuDNN 7.6), but you need to downgrade default OpenCV from 4.1.1 to 3.4.6 (compile OpenCV from source, using this [link](https://jkjung-avt.github.io/opencv-on-nano/))

- YOLO: The official YOLO ROS wrapper GitHub repo [darknet_ros](https://github.com/leggedrobotics/darknet_ros) currently only supports YOLOv3 and below. If you are using YOLOv4, try this repo instead [yolo_v4](https://github.com/tom13133/darknet_ros/tree/yolov4)

### Steps
1. #### Download the ROS wrapper package we are using:
   
    ```cd catkin_workspace/src```

    ```git clone --recursive git@github.com:leggedrobotics/darknet_ros.git```

    **Note: make sure you have `--recursive` tag when downloading the darknet package**

    ```cd ../```

2. #### Build:

    ```catkin_make -DCMAKE_BUILD_TYPE=Release```

3. #### Using your own model: 

   **Note: our repo only contains the hand detection config file, the weight file has to be downloaded using the .sh under /weight folder!**
   
    Within `/darknet_ros/yolo_network_config`:

      1. Add .cfg and .weights (YOLO detection model weights and configs) into /cfg and /weights folder
      
      2. Within /cfg, run `dos2unix your_model.cfg` (convert it to Unix format if you have problem with Windows to Unix format transformation)

    Within `/darknet_ros/config`:

      1. Modify "ros.yaml" with the correct camera topic
      
      2. Create "your_model.yaml" to configure the model files and detected classes

    Within `/darknet_ros/launch`:

      1. Modify "darknet_ros.launch" with the correct YAML file ("your_model.yaml")

4. #### Run:

    ```catkin_make```

    ```source devel/setup.bash```

    ```roslaunch darknet_ros darknet_ros.launch```

    After launching the ROS node, a window will automatically appear that will show the RGB stream and detected objects. You can also check the stream in [RVIZ](http://wiki.ros.org/rviz).

### ROS Topics
* #### Published ROS topics:

   * object_detector (`std_msgs::Int8`) Number of detected objects
   * bounding_boxes (`darknet_ros_msgs::BoundingBoxes`) Bounding boxes (class, x, y, w, h) details are shown in `/darknet_ros_msgs`
   * detection_image (`sensor_msgs::Image`) Image with detected bounding boxes (Note: current version disabled the visualization to save computational power)

---
## Setting up YOLO with CUDA GPU Acceleration

You may find that running YOLO through the CPU is very slow. To increase run-time performance, you can accelerate it by using a CUDA enabled GPU. 

**Note: if you are using NVIDIA Jetson, CUDA and cuDNN are set up by using SDK manager. JetPack 4.3 contains CUDA 10.0 + cuDNN 7.6, which works fine with darknet_ros. You cannot install CUDA and cuDNN seperately since Jetson is ARM based. NVIDIA only supports fixed versions for CUDA and cuDNN on Jetson. Latest Jetpack(4.6) contains CUDA 10.2 + cuDNN 8.x, which you might seen a performance drop because of using cuDNN 8. The issue currently is still not resolved, so for now, use cuDNN 7.6 with darknet_ros.**

**Note: darknet currently only supports (last updated 2021) CUDA 10.2 with cuDNN 7.6.5 and below. If you are using CUDA 11+ or cuDNN 8.0+, you probably need to downgrade CUDA and cuDNN for darknet to work.** 

Here are the detailed instructions on installing CUDA 10.2 and cuDNN 7.6.5:

### Installing CUDA 10.2:
We will follow most of the instructions shown in this [tutorial](https://medium.com/@exesse/cuda-10-1-installation-on-ubuntu-18-04-lts-d04f89287130)

**Note: If there is a `usr/local/cuda` directory in your local machine, remove it (`sudo rm -rf /usr/local/cuda`) before proceeding with the following steps below.** \
*Also, the first step will remove your display driver. This is ok, as when CUDA is reinstalled your display driver will also reinstall automatically.*

1. Remove all CUDA related files already in the machine:
   
    ```
    sudo rm /etc/apt/sources.list.d/cuda*
    sudo apt remove --autoremove nvidia-cuda-toolkit
    sudo apt remove --autoremove nvidia-*
    ```
    
2. Install CUDA 10.2:
   
    ```
    sudo apt update
    sudo apt install cuda-10-2
    sudo apt install libcudnn7
    ```

3. Add CUDA into path:
   
    ```sudo vi ~/.profile```

    Add below at the end of .profile:
    ```
    # set PATH for cuda installation
    if [ -d "/usr/local/cuda/bin/" ]; then
        export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
        export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
    fi
    ```

4. Check CUDA version (make sure it is 10.2):\
  ```nvcc -V```

### Installing cuDNN separately:
1. Go to this [page](https://developer.nvidia.com/rdp/cudnn-archive), you may need to register an account with NVIDIA to access that link.

2. Download all three .deb: runtime/developer/code-sample (make sure that it's the correct version: `cuDNN 7.6.5 with CUDA 10.2`)
   
3. In Terminal:
   
    Go to the package location and install the runtime library, developer library, and (optional) code samples:

    ```
    sudo dpkg -i libcudnn7_7.6.5.32???1+cuda10.2_amd64.deb
    sudo dpkg -i libcudnn7-dev_7.6.5.32???1+cuda10.2_amd64.deb
    sudo dpkg -i libcudnn7-doc_7.6.5.32???1+cuda10.2_amd64.deb
    ``` 

4. Check cuDNN version:
   
    ```/sbin/ldconfig -N -v $(sed 's/:/ /' <<< $LD_LIBRARY_PATH) 2>/dev/null | grep libcudnn```

5. Optional: 
   
    If you cannot locate cudnn.h, or the later compilation fails with `not found cudnn.h` message:

    Copy `cudnn.h` (in `/usr/include`) to (`/usr/local/cuda/include`):\
    `sudo cp /usr/include/cudnn.h /usr/local/cuda/include`

    Copy `libcudnn*` (in `/usr/lib/x86_64-linux-gnu`) to (`/usr/local/cuda/lib64`):\
     `sudo cp /usr/lib/x86_64-linux-gnu/libcudnn* /usr/local/cuda/lib64`


## Running YOLO with GPU Acceleration

The process listed below will work whether you are using YOLO through the darknet_ros package or as a standalone program: 

1. Modify /darnet_ros/darknet/Makefile:
   ```
   GPU = 1
   CUDNN =1
   OPENCV = 1
   ```

   Add your GPU Architecture (ARCH) value. 
   **Note: you can find your ARCH value online [here](https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/).**

   The values specified below correspond to a NVIDIA RTX 2070:

   ```
   -gencode=arch=compute_75,code=compute_75
   ```

2. Run `make` in `/darknet_ros/darknet`

3. Modify `/darknet_ros/darknet_ros/CmakeList.txt`:
   
   ```
   -gencode=arch=compute_75,code=compute_75
   ```

4. Run `catkin_make` in the `/catkin_ws` containing the `darknet_ros` package

**GPU Acceleration is Ready!**

---
## Set Up YOLO 3D with ROS and Get Surface Normal

**Note: if you download our repo, you can skip this setup! You already have everthing to run the full use case. Below are the steps for us to track our implementation.**


### Within catkin_ws/src, clone these repo below

**Note: clone the repo below and checkout to "melodic" branch before going further!!**

[darknet_ros_3d](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d)

**Note: download msg repo and checkout to "melodic" branch before catkin_make!!**

[darknet_ros_3d_msgs](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d_msgs)


### Within catkin_ws/src/gb_visual_detection_3d/darknet_ros_3d, change config and launch files

#### /config:
   1. modify `darknet_3d.yaml` with correct camera *depth-registed* pointcloud topic and "interested_classes" (i.e. "hand")

#### /launch:
   1. modify `darknet_ros_3d.launch` with correct yaml file (i.e. `yolov3-hand.yaml`)


### Run the darknet_ros_3d:
1. Run `catkin_make` in `catkin_ws/` folder:  

    **Note: if ```catkin_make``` doesn't work, try  ```catkin_make -j1```**

2. Run darknet_ros_3d node:
   
    *You don't need to relaunch darknet_ros node, because the 3D repo already have 2D YOLO embed in it.*

    `roslaunch darknet_ros_3d darknet_ros_3d.launch`

3. Check the results in rviz:

    Open `rviz` in terminal

    Check point cloud topic and markers topic to see the visualization of the 3D bboxes.

4. Check the surface normal algorithm in `Darknet3D.cpp` file

    Post-processing the 3D bounding box and get the averaged middle point's surface normal by checking the KNN of radius 15cm.

5. Hyperameters:
   
   In `config/darknet_3d.yaml`:

   **minimum_probability: 0.35**   (hand detection confidence over 0.35 to be treated as a valid hand detection)

   **mininum_detection_thereshold: 0.025**  (The maximum depth of the 3D bounding boxes in meters)

   
## Post-processing and Vision System Integration

**Note: if you only use one single camera, then you can skip this post-processing, and directly use the output from darknet_ros_3d as the goal position, surface normal**

### Goal Getter Node 

In the goal getter, we post-process all of the position and surface normals from different cameras (currently only support this setting: two D435i cameras and 1 t265 camera). It will grab one goal msg at a time from each cameras and convert them to the `/t265_odom` frame, then take the moving average logic and post-process both transformed pose before it converts to the `/world` frame. The goal getter node will keep publishing the final goal pose (tf posestamped msg) to the `/goal` topic. From rviz, you could check the calculated `/goal_pose` by looking at the specific tf frame. 


### Run the whole vision demo:

1. `roslaunch realsense2_camera <corresponding camera launch file: t265, rgbd> camera:=<name> serial_no:=<number>` with three different cameras (2 D435i, 1 T265). You could check our main.launch to see their launch parameter settings.
   
2. `roslaunch darknet_ros_3d darknet_ros_3d.launch` (this launch file will launch all YOLO stuff)
   
3. `roslaunch goal_getter goal_getter.launch` (this launch file will launch the post-processing in multi-cameras settings)

4. Check the rostopic results in terminal:
   
   `rostopic echo /goal`
   
5. Check rviz: tf frame `/goal_pose`

## References
- https://github.com/leggedrobotics/darknet_ros
- https://medium.com/@exesse/cuda-10-1-installation-on-ubuntu-18-04-lts-d04f89287130
- https://pjreddie.com/projects/pascal-voc-dataset-mirror/
- https://cocodataset.org/#home
- https://github.com/tom13133/darknet_ros/tree/yolov4
- https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d

