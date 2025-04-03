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

sudo docker compose exec rosbag bash

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
