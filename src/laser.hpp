#pragma once
#include "camera.hpp"
#include "common.hpp"
#include "contours.hpp"
#include "options.hpp"
#include <atomic>
#include <csignal>
#include <string>

struct OutputFrame {
    std::vector<liquid::Sample> samples;
    Clock::time_point made;
    Clock::time_point cameraTime;
};

void laserLoop(const Options& o,
               std::atomic<bool>& run,
               const Latest<CameraFrame>& camera,
               const Latest<OutputFrame>& output,
               Counters& counters,
               std::string& error,
               std::mutex& errorMutex,
               volatile std::sig_atomic_t& interrupted);
