// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include "example.hpp" // Include short list of convenience functions for rendering
#include <signal.h>
#include <iomanip>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <chrono>

//------------------- TCP Server Code -------------------
//-------------------------------------------------------

typedef void *(*THREADFUNCPTR)(void *);

class Server
{

public:
    Server(int port);
    void *listener_thread();
    void init_listener_thread();
    void update_buffer(const unsigned char *data, int offset, unsigned long numbytes);

private:
    int init_sock, conn_sock;
    char *send_buffer;
    int buffer_size = 1024;
    char receive_buffer[1024];
    struct sockaddr_in serv_addr;
    struct sockaddr_storage serv_storage;
    socklen_t addr_size;
    pthread_mutex_t buffer_access_mutex;
    pthread_t listener_thread_id;
    unsigned long frame_size;
};

Server::Server(int port)
{
    init_sock = socket(PF_INET, SOCK_STREAM, 0);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    memset(serv_addr.sin_zero, '\0', sizeof(serv_addr.sin_zero));
    bind(init_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    send_buffer = new char[buffer_size];
}

void Server::init_listener_thread()
{
    pthread_create(&listener_thread_id, NULL, (THREADFUNCPTR)&Server::listener_thread, this);
    pthread_mutex_init(&buffer_access_mutex, NULL);
}

void *Server::listener_thread()
{
    while (true)
    {
        if (listen(init_sock, 5) == 0)
            printf("Listening...\n");
        else
            printf("Error.\n");

        // Creates new socket for incoming connection
        addr_size = sizeof(serv_storage);
        conn_sock = accept(init_sock, (struct sockaddr *)&serv_storage, &addr_size);
        printf("Connected to client.\n");

        while (true)
        {

            // Parse ping from client
            memset(receive_buffer, 0, sizeof(receive_buffer));
            int resp_msg_size = recv(conn_sock, receive_buffer, 64, 0);
            if (resp_msg_size <= 0)
                break;

            // Send buffer data
            pthread_mutex_lock(&buffer_access_mutex);
            int msg_size = send(conn_sock, send_buffer, buffer_size, MSG_MORE);
            if (msg_size == 0)
                printf("Warning: No data was sent to client.\n");
            int tmp = errno;
            if (msg_size < 0)
                printf("Errno %d\n", tmp);
            pthread_mutex_unlock(&buffer_access_mutex);
        }
    }
}

void Server::update_buffer(const unsigned char *data, int offset, unsigned long numbytes)
{
    pthread_mutex_lock(&buffer_access_mutex);

    // Update buffer size
    unsigned long new_buffer_size = numbytes + offset;
    if (new_buffer_size > buffer_size)
    {
        delete[] send_buffer;
        buffer_size = new_buffer_size;
        send_buffer = new char[buffer_size];
    }

    // Copy data
    memcpy(send_buffer + offset, data, numbytes);
    pthread_mutex_unlock(&buffer_access_mutex);
}

//-------------------------------------------------------
//-------------------------------------------------------

// Configure all streams to run at 1280x720 resolution at 30 frames per second
const int stream_width = 1280;
const int stream_height = 720;
const int stream_fps = 30;
const int depth_disparity_shift = 0;

// Capture color and depth video streams, render them to the screen, send them through TCP
int main(int argc, char *argv[])
try
{

    Server realsense_server(50010);
    realsense_server.init_listener_thread();

    // Create a simple OpenGL window for rendering:
    window app(stream_width * 2, stream_height, "RealSense Stream");

    // Check if RealSense device is connected
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return EXIT_FAILURE;
    }
    int num_devices = devices.size();

    // Configure streams
    rs2::config config_pipe;
    config_pipe.enable_stream(rs2_stream::RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16, stream_fps);
    config_pipe.enable_stream(rs2_stream::RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_RGB8, std::min(stream_fps, 60));

    // Declare two textures on the GPU, one for color and one for depth
    texture depth_image, color_image;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Start streaming
    std::vector<rs2::pipeline> pipelines;
    for (auto &&dev : ctx.query_devices())
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
    }

    // Print active device information
    rs2::pipeline_profile active_pipe_profile = pipelines[0].get_active_profile();
    rs2::device dev = active_pipe_profile.get_device();
    std::cout << "Device information: " << std::endl;
    for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++)
    {
        rs2_camera_info info_type = static_cast<rs2_camera_info>(i);
        std::cout << "  " << std::left << std::setw(20) << info_type << " : ";
        if (dev.supports(info_type))
            std::cout << dev.get_info(info_type) << std::endl;
        else
            std::cout << "N/A" << std::endl;
    }

    // Create advanced mode abstraction for RS400 device and set disparity shift
    rs400::advanced_mode advanced(dev);
    STDepthTableControl depth_table_control;
    if (advanced.is_enabled())
    {
        depth_table_control = advanced.get_depth_table();           // Initialize depth table control group
        depth_table_control.depthUnits = 100;                       // Interpret RMS error at 100 micrometers
        depth_table_control.disparityShift = depth_disparity_shift; // Set disparity shift
        advanced.set_depth_table(depth_table_control);
    }

    // Get active device sensors
    std::vector<rs2::sensor> sensors = dev.query_sensors();
    rs2::sensor depth_sensor = sensors[0];
    rs2::sensor color_sensor = sensors[1];

    // Disable auto exposure and enable auto white balancing for color sensor
    rs2_option ae_option_type = static_cast<rs2_option>(10);
    rs2_option wb_option_type = static_cast<rs2_option>(11);
    if (color_sensor.supports(ae_option_type))
        color_sensor.set_option(ae_option_type, 1);
    if (color_sensor.supports(wb_option_type))
        color_sensor.set_option(wb_option_type, 1);

    std::cout << "Sensor supports the following options:\n"
              << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (color_sensor.supports(option_type))
        {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char *description = color_sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = color_sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            //To change the value of an option, please follow the change_sensor_option() function
        }
        else
        {
            std::cout << " is not supported" << std::endl;
        }
    }

    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (int i = 0; i < 30; ++i)
        pipelines[0].wait_for_frames();

    // Get camera intrinsics of color sensor
    rs2::video_stream_profile color_stream_profile = active_pipe_profile.get_stream(rs2_stream::RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics color_intrinsics = color_stream_profile.get_intrinsics();
    float color_intrinsics_arr[9] = {color_intrinsics.fx, 0.0f, color_intrinsics.ppx,
                                     0.0f, color_intrinsics.fy, color_intrinsics.ppy,
                                     0.0f, 0.0f, 1.0f};

    // Get camera intrinsics of depth sensor
    rs2::video_stream_profile depth_stream_profile = active_pipe_profile.get_stream(rs2_stream::RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2_intrinsics depth_intrinsics = depth_stream_profile.get_intrinsics();
    float depth_intrinsics_arr[9] = {depth_intrinsics.fx, 0.0f, depth_intrinsics.ppx,
                                     0.0f, depth_intrinsics.fy, depth_intrinsics.ppy,
                                     0.0f, 0.0f, 1.0f};

    rs2_extrinsics depth_to_color_extrinsics = depth_stream_profile.get_extrinsics_to(color_stream_profile);
    float depth_to_color_extrinsics_arr[16] = {depth_to_color_extrinsics.rotation[0], depth_to_color_extrinsics.rotation[3], depth_to_color_extrinsics.rotation[6], depth_to_color_extrinsics.translation[0],
                                               depth_to_color_extrinsics.rotation[1], depth_to_color_extrinsics.rotation[4], depth_to_color_extrinsics.rotation[7], depth_to_color_extrinsics.translation[1],
                                               depth_to_color_extrinsics.rotation[2], depth_to_color_extrinsics.rotation[5], depth_to_color_extrinsics.rotation[8], depth_to_color_extrinsics.translation[2],
                                               0.0f, 0.0f, 0.0f, 1.0f};

    // Get depth scale for converting depth pixel values into distances in meters
    float depth_scale = depth_sensor.as<rs2::depth_sensor>().get_depth_scale();

    // Create alignment object (for aligning depth frame to color frame)
    rs2::align align(rs2_stream::RS2_STREAM_COLOR);

    float row_height = app.height() / num_devices;

    while (app) // Application still alive?
    {
        // Collect the new frames from all the connected devices
        std::vector<rs2::frameset> new_framesets;
        for (auto &&pipe : pipelines)
        {
            new_framesets.emplace_back(pipe.wait_for_frames());
        }
        // Wait for next set of RGB-D frames from the camera
        long epoch_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        int idx = 0;
        for (rs2::frameset &fs : new_framesets)
        {
            rs2::frame color = fs.get_color_frame();
            auto processed = align.process(fs);
            rs2::depth_frame aligned_depth = processed.get_depth_frame();
            // Find and colorize the depth data
            rs2::frame depth_colorized = color_map.colorize(aligned_depth);
            int depth_size = aligned_depth.get_width() * aligned_depth.get_height() * aligned_depth.get_bytes_per_pixel();

            realsense_server.update_buffer((unsigned char *)aligned_depth.get_data(), 9 * 4 + 9 * 4 + 16 * 4 + 4 + 8, depth_size);

            int color_size = fs.get_color_frame().get_width() * fs.get_color_frame().get_height() * fs.get_color_frame().get_bytes_per_pixel();
            realsense_server.update_buffer((unsigned char *)color.get_data(), 9 * 4 + 9 * 4 + 16 * 4 + 4 + 8 + depth_size, color_size);

            // Save header: color camera intrinsics, depth camera intrinsics, color to depth camera extrinsics, depth scale, and timestamp
            realsense_server.update_buffer((unsigned char *)color_intrinsics_arr, 0, 9 * 4);
            realsense_server.update_buffer((unsigned char *)depth_intrinsics_arr, 9 * 4, 9 * 4);
            realsense_server.update_buffer((unsigned char *)depth_to_color_extrinsics_arr, 9 * 4 + 9 * 4, 16 * 4);
            realsense_server.update_buffer((unsigned char *)&depth_scale, 9 * 4 + 9 * 4 + 16 * 4, 4);
            realsense_server.update_buffer((unsigned char *)&epoch_timestamp, 9 * 4 + 9 * 4 + 16 * 4 + 4, 8);

            // Render depth on to the first half of the screen and color on to the second

            depth_image.render(depth_colorized,
                               {0,
                                idx * row_height,
                                app.width() / 2,
                                row_height});
            color_image.render(color,
                               {app.width() / 2,
                                idx * row_height,
                                app.width() / 2,
                                row_height});
            idx += 1;
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
