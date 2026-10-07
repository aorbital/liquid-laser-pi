#include "options.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

void printHelp() {
    std::cout << R"(Liquid Laser Pi

Usage:
  laser [min_cm max_cm smoothing] [options]

Examples:
  laser 100 110 2
  laser 80 100 2 --particles 160 --field 96 --substeps 1
  laser 100 110 2 --no-laser

Core:
  --particles N       10..1000, default 200
  --field N           64..256, default 128
  --fps N             30 or 60, default 60
  --substeps N        1..4, default 2
  --iterations N      1..3, default 1
  --max-loops N       1..8, default 6
  --points N          120..400, default 280

Liquid:
  --cohesion X        0..2, default .95
  --viscosity X       0..1, default .45
  --repulsion X       0..2, default .65
  --influence X       .03..0.5, default .14
  --flow X            0..0.1, default .010
  --threshold X       .1..0.9, default .34

Laser:
  --scale X           .05..0.8, default .35
  --brightness N      0..255, default 255
  --flip-x
  --flip-y
  --no-laser

Camera / UI:
  --largest-body
  --bounds X0 Y0 X1 Y1
  --camera-width N
  --camera-height N
  --camera-fps N
  --no-gui

Utility:
  --seconds N
  --csv PATH
  --demo
  --help

GUI:
  q / Esc   exit
  r         reset liquid
)";
}

Options parseOptions(int argc,char** argv) {
    Options o;
    std::vector<std::string> positional;

    auto number=[](const std::string& s){
        size_t used=0; double n=std::stod(s,&used);
        if(used!=s.size()||!std::isfinite(n))throw std::invalid_argument("invalid number: "+s);
        return n;
    };
    auto integer=[&](const std::string& s){
        double n=number(s);
        if(n!=std::floor(n))throw std::invalid_argument("invalid integer: "+s);
        return int(n);
    };

    for(int i=1;i<argc;++i){
        std::string key=argv[i];
        auto value=[&](){
            if(i+1>=argc)throw std::invalid_argument("missing value for "+key);
            return std::string(argv[++i]);
        };

        if(key=="--help"||key=="-h"){printHelp();std::exit(0);}
        else if(key=="--no-gui")o.gui=false;
        else if(key=="--no-laser")o.noLaser=true;
        else if(key=="--demo")o.demo=true;
        else if(key=="--largest-body")o.largestBody=true;
        else if(key=="--flip-x")o.flipX=true;
        else if(key=="--flip-y")o.flipY=true;
        else if(key=="--particles")o.sim.particles=integer(value());
        else if(key=="--field")o.sim.field=integer(value());
        else if(key=="--fps")o.fps=integer(value());
        else if(key=="--substeps")o.sim.substeps=integer(value());
        else if(key=="--iterations")o.sim.iterations=integer(value());
        else if(key=="--max-loops")o.sim.maxLoops=integer(value());
        else if(key=="--points")o.sim.points=integer(value());
        else if(key=="--cohesion")o.sim.cohesion=float(number(value()));
        else if(key=="--viscosity")o.sim.viscosity=float(number(value()));
        else if(key=="--repulsion")o.sim.repulsion=float(number(value()));
        else if(key=="--influence")o.sim.influence=float(number(value()));
        else if(key=="--flow")o.sim.flow=float(number(value()));
        else if(key=="--threshold")o.sim.threshold=float(number(value()));
        else if(key=="--scale")o.scale=float(number(value()));
        else if(key=="--brightness")o.brightness=integer(value());
        else if(key=="--seconds")o.duration=number(value());
        else if(key=="--csv")o.csv=value();
        else if(key=="--camera-width")o.cameraWidth=integer(value());
        else if(key=="--camera-height")o.cameraHeight=integer(value());
        else if(key=="--camera-fps")o.cameraFPS=integer(value());
        else if(key=="--bounds"){
            float x0=float(number(value())),y0=float(number(value())),
                  x1=float(number(value())),y1=float(number(value()));
            if(x0<0||y0<0||x1>1||y1>1||x1-x0<.2f||y1-y0<.2f)
                throw std::invalid_argument("invalid --bounds");
            float e=o.sim.extent;
            o.sim.lower={x0*2*e-e,y0*2*e-e};
            o.sim.upper={x1*2*e-e,y1*2*e-e};
        }
        else if(key.rfind("--",0)==0)throw std::invalid_argument("unknown option: "+key);
        else positional.push_back(key);
    }

    if(positional.size()>3)throw std::invalid_argument("expected at most 3 positional arguments");
    if(!positional.empty())o.minMM=int(std::lround(number(positional[0])*10.0));
    if(positional.size()>1)o.maxMM=int(std::lround(number(positional[1])*10.0));
    if(positional.size()>2)o.sim.smoothing=integer(positional[2]);

    if(o.minMM<=0||o.minMM>=o.maxMM||
       o.sim.particles<10||o.sim.particles>1000||
       o.sim.field<64||o.sim.field>256||
       (o.fps!=30&&o.fps!=60)||
       o.sim.substeps<1||o.sim.substeps>4||
       o.sim.iterations<1||o.sim.iterations>3||
       o.sim.maxLoops<1||o.sim.maxLoops>8||
       o.sim.points<120||o.sim.points>400||
       o.sim.smoothing<0||o.sim.smoothing>8||
       o.sim.cohesion<0||o.sim.cohesion>2||
       o.sim.viscosity<0||o.sim.viscosity>1||
       o.sim.repulsion<0||o.sim.repulsion>2||
       o.sim.influence<.03f||o.sim.influence>.5f||
       o.sim.flow<0||o.sim.flow>.1f||
       o.sim.threshold<.1f||o.sim.threshold>.9f||
       o.scale<.05f||o.scale>.8f||
       o.brightness<0||o.brightness>255||
       o.cameraWidth<16||o.cameraHeight<16||o.cameraFPS<1)
        throw std::invalid_argument("argument out of range; run laser --help");

    if(o.demo)o.noLaser=true;
    o.sim.radius=std::min(o.sim.radius,.4f*std::min(o.sim.upper.x-o.sim.lower.x,
                                                    o.sim.upper.y-o.sim.lower.y));
    return o;
}
