# DOMINANCE Team Won Drone Code

## Hardware Requirements
* Jetson Nano
* CSI camera connected to the Nano

## Software Requirements
* JetPack (NVIDIA Ubuntu)
* OpenCV 4 (this comes with JetPack, you can reinstall it from [source here](https://github.com/mdegans/nano_build_opencv). This will take a few hours.)
* pip
* [Tensorflow](https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html)
* Tensorflow model repository: 
    1. `git clone https://github.com/tensorflow/models.git`
    2. `cd models/research`
    3. `protoc object_detection/protos/*.proto --python_out=.`
    3. `pip install .`



## Connecting to Nano
The Nano, when in multi-user (non-graphical terminal mode) will still connect to the network. It may or may not use the same IP address as it used to use on that network.  
1. Depending on the network, you will need to ping the broadcast IP. Unix: `ping 192.168.1.255 -b` Windows: `ping 192.168.1.255` .
2. Then, `arp` should have the Nano listed. Unix: `arp` Windows: `arp -a`. 

## Running
Simply run `start.sh`, and make sure it has execute capabilities: `chmod +x start.sh`.  
The streaming server will automatically start at port `9090`, so simply navigate to `<ip>:9090` in Google Chrome to view it. For example, if the Nano connects to the wifi and is assigned
IP address 192.168.1.50, you would put `192.168.1.50:9090` in the addresss bar. You *can* use Firefox, but for some reason the stream sometimes freezes in the first few seconds.

Running `start.sh` will automatically run `git pull` making code updates easy.
