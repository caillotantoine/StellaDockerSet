# Stella Docker Set
The Stella Docker set is a collection of docker files to easily use Stella VSLAM. 

**Stella VSLAM:**
```bash
git clone --recursive https://github.com/stella-cv/stella_vslam.
cd stella_vslam
```

**Socket Viewer:**
```
git clone --recursive https://github.com/stella-cv/socket_viewer.git
cd socket_viewer
```

**Stella VSLAM ROS**
```
git clone --recursive -b ros https://github.com/stella-cv/stella_vslam_ros.git
cd stella_vslam_ros
```

> Note to delete: sudo docker compose exec rosbag bash

## Built images

### Socket Viewer
*Image name: stella_vslam-viewer*

Socket Viewer (necessary for most following images).
```
docker build -t stella_vslam-viewer .
```

> Run on MacOS: `docker run --rm -it --name stella_vslam-viewer -p 3001:3001 stella_vslam-viewer`
> Run on Linux (not tested) : `docker run --rm -it --name stella_vslam-socket --net=host stella_vslam-socket`



### Stella VSLAM (with socket Viewer)

*Image name: stella_vslam-socket*

Basic Socket Image without ROS. Ideal to test examples
```
docker build -t stella_vslam-socket -f Dockerfile.socket . --build-arg NUM_THREADS=4
```

> Run : `docker run --rm -it --name stella_vslam-socket stella_vslam-socket`\
> **⚠ Must run the Socket Viewer docker ⚠**

>**Note:** You can access the visualizer from any machine connected to the same network. e.g. for a machine called `slam-pc`, you may connect to the viewer at the following: `http://slam-pc.local:3001`


### Stella VSLAM (with socket Viewer) with ROS 1
*Image name : stella_vslam-socket-ros*

Basic Socket Image with ROS 1. 
```
docker build -t stella_vslam-socket-ros -f Dockerfile.socket . --build-arg NUM_THREADS=4
```


> It is possible to run Stella VSLAM with ROS without docker compose, but there won't have any viewer. This is usefull if using stella VSLAM only using ROS (e.g. vizualizing the trajectory with rviz).
> - Start the container : `docker run --rm -it --net=host -v `pwd`/stella_data:/stella_data -v `pwd`/datasets:/datasets --name stella_vslam-socket-ros stella_vslam-socket-ros`\
> - Load ROS `source devel/setup.bash`\
> - Start stella_vslam : `rosrun stella_vslam_ros run_slam -v /stella_data/vocab/orb_vocab.fbow --mask /stella_data/masks/maskTray.jpg -c /stella_data/yaml/insta360_1X2.yaml --viewer none`
> 
> To use the visualizer, you may want to use docker compose. This also enable the creation of a full ROS environement in Docker.


## Docker Compose

### Stella VSLAM + Socket Viewer
Basic docker compose to run the socket viewer and the `stella_vslam-socket` image.
1. In a terminal, run:
```
docker compose -f ./docker_compose/docker-compose-socket.yaml up
```
2. In another terminal:
- Run `docker compose -f ./docker_compose/docker-compose-socket.yaml exec stella_vslam bash` to connect to the `stella_vslam-socket` image container
- Inside the container, an example command would be : `./run_video_slam -v /stella_data/vocab/orb_vocab.fbow -c /stella_data/yaml/iphone_UWC_HD.yaml -m /datasets/test_1920x1080_59.95FPS_UWC13mm_AE_NoHDR.mp4`


### Stella VSLAM & Socket Viewer in a ROS environement
In an existing ROS 1 environemnt, this docker compose is ideal. However, there is a few things that should be setup on the host machine. 
> Here, the hypothesis is that the roscore is runing on the host machine.

#### Compulsory environement variables
You may want to add the following lines to your `.bashrc` as they are needed in **every related terminal**:
```bash
export HOSTNAME=$HOSTNAME
export ROS_HOSTNAME="$HOSTNAME.local"
export ROS_MASTER_URI="http://$HOSTNAME.local:11311"
```

#### Starting and using Stella VSLAM
- Start the containers : `docker compose -f ./docker_compose/docker-compose-socket-ros.yaml up`\
This will start the visualizer server that you can connect to at : [http://localhost:3001](http://localhost:3001).
- Connect a terminal to Stella VSLAM container : `docker compose -f ./docker_compose/docker-compose-socket-ros.yaml exec stella_vslam bash`\
In the connected terminal: 
    - Setup the ros environement : `source devel/setup.bash`
    - Start Stella VSLAM: `rosrun stella_vslam_ros run_slam -v /stella_data/vocab/orb_vocab.fbow --mask /stella_data/masks/maskTray.jpg -c /stella_data/yaml/insta360_1X2.yaml`

### Stella VSLAM & Socket Viewer and ROS in the containers
- Start the containers : `docker compose -f ./docker_compose/docker-compose-socket-rosfull.yaml up`\
This will start the visualizer server that you can connect to at : [http://localhost:3001](http://localhost:3001).
- Connect a terminal to the container to play the rosbags : `docker compose -f ./docker_compose/docker-compose-socket-rosfull.yaml exec rosbag bash`\
- Connect a terminal to the container for Stella VSLAM : `docker compose -f ./docker_compose/docker-compose-socket-rosfull.yaml exec stella_vslam bash`\
In the connected terminal: 
    - Setup the ros environement : `source devel/setup.bash`
    - Start Stella VSLAM: `rosrun stella_vslam_ros run_slam -v /stella_data/vocab/orb_vocab.fbow --mask /stella_data/masks/maskTray.jpg -c /stella_data/yaml/insta360_1X2.yaml`


## Stella VSLAM

### Run from ROS 1:
```
rosrun stella_vslam_ros run_slam 
```

### Minimal arguments

#### Vocabulary
Vocabulary file:
```
-v /stella_data/vocab/orb_vocab.fbow
```

#### Configuration
Configuration file:
```
-c /stella_data/yaml/insta360_1X2.yaml
```

#### Mask
Optional but strongly recommended:
```
--mask /stella_data/masks/maskTray.jpg
```

### Usage of a map
To import a map:
```
--map-db-in /stella_data/maps/mocapZone1_insta360_1x2.msg
```

To export a map:
```
--map-db-out /stella_data/maps/newMap.msg
```

### Export trajectory
The directory must exist first.
```
--eval-log-dir /outputs
```

3 files are generated (and overwritten if not moved properly):
- frame_trajectory.txt
- keyframe_trajectory.txt
- track_times.txt

The two former one are stored in TUM format:
<table>
    <thead>
        <tr>
            <th>Time (s)</th>
            <th colspan=3>Location (m)</th>
            <th colspan=4>Rotation (quaternion)</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td>timestamp</td>
            <td>x</td>
            <td>y</td>
            <td>z</td>
            <td>q<sub>x</sub></td>
            <td>q<sub>y</sub></td>
            <td>q<sub>z</sub></td>
            <td>q<sub>w</sub></td>
        </tr>
    </tbody>
</table>