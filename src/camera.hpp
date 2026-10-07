#pragma once
#include "common.hpp"
#include "liquid.hpp"
#include "options.hpp"
#include <OpenNI.h>
#include <opencv2/core.hpp>
#include <atomic>
#include <csignal>
#include <string>

struct CameraFrame {
    liquid::Obstacle obstacle;
    cv::Mat depth;
    cv::Mat mask;
    Clock::time_point captured;
    uint64_t seq = 0;
    double ms = 0.0;
    bool valid = true;
};

struct Counters {
    std::atomic<uint64_t> cameraFrames{0};
    std::atomic<uint64_t> submitted{0};
    std::atomic<uint64_t> laserErrors{0};
    std::atomic<double> cameraMs{0.0};
};

liquid::Grid signedDistance(const cv::Mat& mask,float extent);
void filterBody(cv::Mat& mask,bool largestOnly);

class Camera {
    openni::Device device_;
    openni::VideoStream stream_;
    bool initialized_ = false;
public:
    ~Camera();
    void open(const Options& o);
    bool read(cv::Mat& crop);
};

void captureLoop(const Options& o,
                 std::atomic<bool>& run,
                 Latest<CameraFrame>& shared,
                 Counters& counters,
                 std::string& error,
                 std::mutex& errorMutex,
                 volatile std::sig_atomic_t& interrupted);
