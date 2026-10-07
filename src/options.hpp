#pragma once
#include "liquid.hpp"
#include <string>

struct Options {
    liquid::Config sim;
    int minMM = 1000;
    int maxMM = 1100;
    int fps = 60;
    int brightness = 255;
    int cameraWidth = 160;
    int cameraHeight = 120;
    int cameraFPS = 30;
    bool gui = true;
    bool noLaser = false;
    bool demo = false;
    bool largestBody = false;
    bool flipX = false;
    bool flipY = false;
    float scale = .35f;
    double duration = 0.0;
    std::string csv;
};

void printHelp();
Options parseOptions(int argc,char** argv);
