# realsense

The latest version of our system uses RGB-D data captured from an [Intel® RealSense™ D415 Camera](https://click.intel.com/intelr-realsensetm-depth-camera-d415.html). We provide a lightweight C++ executable that streams data in real-time using [librealsense SDK 2.0](https://github.com/IntelRealSense/librealsense) via TCP. This enables you to connect the camera to an external computer and fetch RGB-D data remotely over the network while training. This can come in handy for many real robot setups. Of course, doing so is not required -- the entire system can also be run on the same computer.

#### Installation Instructions:

1. Download and install [librealsense SDK 2.0](https://github.com/IntelRealSense/librealsense)
2. Compile `realsense.cpp`:

    ```shell
    cd realsense
    mkdir build
    cd build
    cmake ..
    make
    ```

Note: To enable verbose logging of device information, use `cmake -DLOG_INFO ..` instead.

#### To Run

1. Connect your RealSense camera with a USB 3.0 compliant cable (important: RealSense D400 series uses a USB-C cable, but still requires them to be 3.0 compliant to be able to stream RGB-D data).
To start the TCP server and RGB-D streaming, run the following:

    ```shell
    ./realsense
    ```

    Optionally, pick your own port

    ```shell
    ./realsense 12345
    ```

    Optionally, pick your own port and manual exposure
    ```shell
    ./realsense 12345 120
    ```

2. If there are multiple cameras, a prompt will print out their serial numbers and ask you to choose which device

    ```shell
    Found 2 devices.
    [0] 821212062745
    [1] 805212060035
    Pick one device:
    ```
    In this case, for instance, you can choose `0` or `1`.

3. Keep the executable running, and test a python TCP client that fetches RGB-D data from the active TCP server, run the following:

    ```shell
    python capture.py
    ```

4. You can change the camera parameter by changing the following lines in the ``realsense.cpp`` file:

    ```shell
    int stream_width = 640;
    int stream_height = 360;
    int depth_disparity_shift = 25;
    int stream_fps = 30;
    ```

    Also change the following lines in the ``camera.py`` file to match the cpp file:

    ```shell
    self.im_height = 720
    self.im_width = 1280
    self.tcp_host_ip = '127.0.0.1'     
    self.tcp_port = 50010
    ```

## NOTE: FIX FOR VERSIONS PRIOR TO V2.16 (V2.15 and earlier)
Change the below line:

```c
// Find and colorize the depth data
rs2::frame depth_colorized = color_map.colorize(aligned_depth);
```

to

```c
// Find and colorize the depth data
rs2::frame depth_colorized = color_map(aligned_depth);
```

## Dependencies

### Install GLFW
[GLFW](https://github.com/glfw/glfw) is required for capturing images. Check out [compiling guide](https://www.glfw.org/docs/latest/compile.html).

```bash
# Install dependencies for Ubuntu:
$ sudo apt install xorg-dev
```

```bash
# Compile and install GLFW
git clone git@github.com:glfw/glfw.git
cd glfw
mkdir build
cd build
cmake ..
make
sudo make install
```
