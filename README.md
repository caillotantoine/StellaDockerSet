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

### stella_vslam-viewer

Socket Viewer (necessary for most following images).
```
docker build -t stella_vslam-viewer .
```

> Run on MacOS: `docker run --rm -it --name stella_vslam-viewer -p 3001:3001 stella_vslam-viewer`
> Run on Linux (not tested) : `docker run --rm -it --name stella_vslam-socket --net=host stella_vslam-socket`



### stella_vslam-socket

Basic Socket Image without ROS. Ideal to test examples
```docker build -t stella_vslam-socket -f Dockerfile.socket . --build-arg NUM_THREADS=4```

> Run : `docker run --rm -it --name stella_vslam-socket stella_vslam-socket`\
> **⚠ Must run the Socket Viewer docker ⚠**


### stella_vslam-socket-ros

```
git clone --recursive -b ros https://github.com/stella-cv/stella_vslam_ros.git
```
Basic Socket Image with ROS 1. 
```docker build -t stella_vslam-socket-ros -f Dockerfile.socket . --build-arg NUM_THREADS=4```
> Run : `docker run --rm -it --net=host -v `pwd`/stella_data:/stella_data -v `pwd`/datasets:/datasets --name stella_vslam-socket-ros stella_vslam-socket-ros`\
> Load ROS `source devel/setup.bash`\
> Start stella_vslam : `rosrun stella_vslam_ros run_slam -v /stella_data/vocab/orb_vocab.fbow --mask /stella_data/masks/maskTray.jpg -c /stella_data/yaml/insta360_1X2.yaml --viewer none`



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


### Stella VSLAM + Socket VIWER + ROS
- `export HOSTNAME=$HOSTNAME; sudo docker compose -f ./docker_compose/docker-compose-socket-ros.yaml exec stella_vslam bash`
- `export HOSTNAME=$HOSTNAME; sudo docker compose -f ./docker_compose/docker-compose-socket-ros.yaml exec stella_vslam bash`
- `export ROS_MASTER_URI="http://$HOSTNAME.local:11311"`
- `export ROS_HOSTNAME="$HOSTNAME.local"`

