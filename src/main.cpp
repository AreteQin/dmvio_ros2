/**
 * ROS driver for DM-VIO (converted to ROS2 Humble)
 * Original code by Lukas von Stumberg (http://vision.in.tum.de/dm-vio).
 * This file is in part based on the file main_dso_pangolin.cpp of the project DSO
 * as well as the ROS driver for DSO by Jakob Engel.
 *
 * DM-VIO is free software distributed under the GNU GPL.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

#include <thread>
#include <locale>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <cassert>
#include <vector>

// DM-VIO and DSO related includes
#include "IOWrapper/Output3DWrapper.h"
#include "util/Undistort.h"
#include <boost/thread.hpp>
#include "dso/util/settings.h"
#include "dso/util/globalCalib.h"
#include "util/TimeMeasurement.h"
#include "FullSystem/FullSystem.h"
#include <util/SettingsUtil.h>
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "util/MainSettings.h"
#include "live/FrameSkippingStrategy.h"
#include "live/IMUInterpolator.h"
#include "ROSOutputWrapper.h"
#include <live/FrameContainer.h>

// For rosbag reading -- note: ROS2 uses rosbag2 which has a different API.
// You will need to adapt this part if you wish to read from ROS2 bag files.
//#include <rosbag2_cpp/reader.hpp>

#ifdef STACKTRACE
#include <boost/stacktrace.hpp>
#include <signal.h>
#endif

using namespace dso;

#ifdef STACKTRACE
void segfault_handler(int sig)
{
  std::cerr << "Caught signal " << sig << ", printing stacktrace:" << std::endl;
  std::cerr << boost::stacktrace::stacktrace() << std::endl;
  exit(1);
}
#endif

// Global objects (as in your original code):
dmvio::FrameContainer frameContainer;
dmvio::IMUInterpolator imuInt(frameContainer, nullptr);
dmvio::MainSettings mainSettings;
dmvio::IMUCalibration imuCalibration;
dmvio::IMUSettings imuSettings;
dmvio::FrameSkippingSettings frameSkippingSettings;
std::unique_ptr<Undistort> undistorter;
bool stopSystem = false;
int start = 2;
double timeshift = 0.0;
std::string rosbagFile = "";
bool loadRosbagThread = false;
bool finishedLoading = false;

std::string imageTopic, imuTopic;

// Helper function to convert ROS2 header stamp to double (seconds)
double convertStamp(const builtin_interfaces::msg::Time& stamp)
{
    return stamp.sec + stamp.nanosec / 1e9;
}

// Callback for image messages
void vidCb(const sensor_msgs::msg::Image::ConstSharedPtr img)
{
    double stamp = convertStamp(img->header.stamp) + timeshift;

    // Convert the ROS2 image message to an OpenCV image (MONO8)
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr->image.type() == CV_8U);
    assert(cv_ptr->image.channels() == 1);

    MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,
                         (unsigned char*)cv_ptr->image.data);

    // The image message does not contain exposure.
    // Depending on your photometric mode, adjust the parameters accordingly.
    std::unique_ptr<ImageAndExposure> undistImg(
        undistorter->undistort<unsigned char>(&minImg, 1.0, stamp, 1.0f));

    imuInt.addImage(std::move(undistImg), stamp);
}

// Callback for IMU messages
void imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr imu)
{
    std::vector<float> accData{
        static_cast<float>(imu->linear_acceleration.x), static_cast<float>(imu->linear_acceleration.y),
        static_cast<float>(imu->linear_acceleration.z)
    };

    std::vector<float> gyrData{
        static_cast<float>(imu->angular_velocity.x), static_cast<float>(imu->angular_velocity.y),
        static_cast<float>(imu->angular_velocity.z)
    };

    double timestamp = convertStamp(imu->header.stamp);
    imuInt.addAccData(accData, timestamp);
    imuInt.addGyrData(gyrData, timestamp);
}

// Function to load data from a rosbag.
// Note: In ROS2, use the rosbag2 API (this is only a placeholder).
void loadFromRosbag()
{
    dmvio::TimeMeasurement meas("RosbagLoading");

    // ROS2: Replace this section with appropriate rosbag2_cpp reader code.
    RCLCPP_INFO(rclcpp::get_logger("main"), "Rosbag loading not implemented in this ROS2 conversion. Skipping.");

    // Indicate that loading is finished to let the main loop exit when using a bag.
    finishedLoading = true;
}

// Main processing function (runs in a separate thread)
void run(IOWrap::PangolinDSOViewer* viewer)
{
    bool linearizeOperation = rosbagFile != ""; // linearize operation (non-real time) if loading a rosbag.
    if (linearizeOperation && setting_minFramesBetweenKeyframes < 0)
    {
        setting_minFramesBetweenKeyframes = -setting_minFramesBetweenKeyframes;
        std::cout << "Using setting_minFramesBetweenKeyframes=" << setting_minFramesBetweenKeyframes
            << " because of non-realtime mode." << std::endl;
    }

    std::thread rosbagLoadThread;
    if (rosbagFile != "")
    {
        std::cout << "Loading images from ROSBAG instead of subscribing to the topics." << std::endl;
        if (loadRosbagThread)
        {
            rosbagLoadThread = std::thread(loadFromRosbag);
        }
        else
        {
            std::cout << "Loading all ROS messages..." << std::endl;
            loadFromRosbag();
            std::cout << "Finished loading." << std::endl;
        }
    }

    // Create the full system, passing the linearization flag and calibration/settings.
    auto fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);

    if (setting_photometricCalibration > 0 && undistorter->photometricUndist == nullptr)
    {
        printf("ERROR: don't have photometric calibration. Need to use commandline options mode=1 or mode=2 ");
        exit(1);
    }

    if (undistorter->photometricUndist != nullptr)
    {
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
    }

    if (viewer)
    {
        fullSystem->outputWrapper.push_back(viewer);
    }

    dmvio::FrameSkippingStrategy frameSkipping(frameSkippingSettings);
    // Frame skipping registers as an outputWrapper to get notified of system status changes.
    fullSystem->outputWrapper.push_back(&frameSkipping);

    // This will handle publishing to ROS topics.
    dmvio::ROSOutputWrapper rosOutput;
    fullSystem->outputWrapper.push_back(&rosOutput);

    int ii = 0;
    int lastResetIndex = 0;

    while (!stopSystem)
    {
        // Skip the first few frames if requested.
        if (start > 0 && ii < start)
        {
            auto pair = frameContainer.getImageAndIMUData(0);
            ++ii;
            continue;
        }

        int numSkipFrames = frameSkipping.getMaxSkipFrames(frameContainer.getQueueSize());
        if (rosbagFile != "")
        {
            numSkipFrames = 0; // never skip frames when loading from rosbag.
        }
        auto pair = frameContainer.getImageAndIMUData(numSkipFrames);
        if (!pair.first)
            continue;

        fullSystem->addActiveFrame(pair.first.get(), ii, &(pair.second), nullptr);

        if (fullSystem->initFailed || setting_fullResetRequested)
        {
            if (ii - lastResetIndex < 250 || setting_fullResetRequested)
            {
                printf("RESETTING!\n");
                std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                fullSystem.reset();
                for (IOWrap::Output3DWrapper* ow : wraps)
                    ow->reset();

                fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);
                if (undistorter->photometricUndist != nullptr)
                {
                    fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
                }
                fullSystem->outputWrapper = wraps;
                setting_fullResetRequested = false;
                lastResetIndex = ii;
            }
        }

        if (viewer != nullptr && viewer->shouldQuit())
        {
            std::cout << "User closed window -> Quit!" << std::endl;
            break;
        }

        if (fullSystem->isLost)
        {
            printf("LOST!!\n");
            break;
        }

        ++ii;

        // Exit automatically when using a rosbag and all messages are processed.
        if (rosbagFile != "" && finishedLoading && frameContainer.getQueueSize() == 0)
        {
            break;
        }
    }

    fullSystem->blockUntilMappingIsFinished();
    fullSystem->printResult(imuSettings.resultsPrefix + "result.txt", false, false, true);
    fullSystem->printResult(imuSettings.resultsPrefix + "resultScaled.txt", false, true, true);
    dmvio::TimeMeasurement::saveResults(imuSettings.resultsPrefix + "timings.txt");

    for (IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
    }

    if (rosbagFile != "")
    {
        // Join the rosbag loading thread (if running in a separate thread)
        if (rosbagLoadThread.joinable())
            rosbagLoadThread.join();
    }

    printf("DELETE FULLSYSTEM!\n");
    fullSystem.reset();

    // Shutdown the ROS2 system.
    rclcpp::shutdown();

    printf("EXIT NOW!\n");
}

int main(int argc, char** argv)
{
#ifdef STACKTRACE
  // Set up the segfault handler.
  struct sigaction sa;
  sa.sa_handler = segfault_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART | SA_SIGINFO;
  sigaction(SIGSEGV, &sa, NULL);
#endif

    // Initialize ROS2.
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("DMVIO_ros");

    setlocale(LC_ALL, "C");

#ifdef DEBUG
    std::cout << "DEBUG MODE!" << std::endl;
#endif

    auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();

    // Create Settings files and register arguments.
    imuSettings.registerArgs(*settingsUtil);
    imuCalibration.registerArgs(*settingsUtil);
    mainSettings.registerArgs(*settingsUtil);
    frameSkippingSettings.registerArgs(*settingsUtil);

    settingsUtil->registerArg("start", start);

    auto normalizeCamSize = std::make_shared<double>(0.0);
    settingsUtil->registerArg("normalizeCamSize", *normalizeCamSize, 0.0, 5.0);
    // timeshift from camera to imu: timestamp_imu = timestamp_cam + timeshift.
    settingsUtil->registerArg("timeshift", timeshift);
    // If set, data is loaded from rosbag instead of subscribing live.
    settingsUtil->registerArg("rosbag", rosbagFile);
    // If true, data is loaded from rosbag in a separate thread.
    settingsUtil->registerArg("loadRosbagThread", loadRosbagThread);

    // Parse command-line arguments (and optionally a YAML settings file).
    mainSettings.parseArguments(argc, argv, *settingsUtil);

    // Print settings to console and write to file.
    std::cout << "Settings:\n";
    settingsUtil->printAllSettings(std::cout);
    {
        std::ofstream settingsStream;
        settingsStream.open(imuSettings.resultsPrefix + "usedSettingsdso.txt");
        settingsUtil->printAllSettings(settingsStream);
    }

    undistorter.reset(Undistort::getUndistorterForFile(mainSettings.calib,
                                                       mainSettings.gammaCalib,
                                                       mainSettings.vignette));

    setGlobalCalib((int)undistorter->getSize()[0],
                   (int)undistorter->getSize()[1],
                   undistorter->getK().cast<float>());

    imuCalibration.loadFromFile(mainSettings.imuCalibFile);

    std::unique_ptr<IOWrap::PangolinDSOViewer> viewer;
    if (!disableAllDisplay)
    {
        viewer = std::make_unique<IOWrap::PangolinDSOViewer>(wG[0], hG[0], true, settingsUtil, normalizeCamSize);
    }

    // Resolve topic names.
    imageTopic = std::string(node->get_namespace()) + "/cam0/image_raw";
    imuTopic = std::string(node->get_namespace()) + "/imu0";
    std::cout << "Image topic: " << imageTopic << std::endl;
    std::cout << "IMU topic: " << imuTopic << std::endl;

    // Start the main processing thread.
    std::thread runThread(run, viewer.get());

    // Create subscriptions for live data (only if not loading from a rosbag).
    rclcpp::QoS imageQos(3);
    rclcpp::QoS imuQos(50);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
    if (rosbagFile == "")
    {
        imageSub = node->create_subscription<sensor_msgs::msg::Image>(
            "cam0/image_raw", imageQos, vidCb);
        imuSub = node->create_subscription<sensor_msgs::msg::Imu>(
            "imu0", imuQos, imuCb);
    }

    // Spin until rclcpp::shutdown() is called.
    rclcpp::spin(node);

    stopSystem = true;
    frameContainer.stop();

    // Ensure the processing thread finishes.
    runThread.join();

    return 0;
}
