#pragma once
#include "liquid.hpp"
#include <vector>
#include <cstdint>

namespace liquid {
using Loop = std::vector<V>;

float area(const Loop& p);
float perimeter(const Loop& p);
Loop resample(const Loop& p,int count);
void smooth(Loop& p,int iterations);

class Contours {
    std::vector<int> edgeVertex_;
    std::vector<V> vertices_;
    std::vector<std::array<int,2>> adjacent_;
    std::vector<uint8_t> visited_;
public:
    std::vector<Loop> extract(const Grid& field,float level,float minArea,
                              int maxLoops,int budget,int smoothing);
};

struct Sample { V xy; bool lit=false; };
struct ScanConfig { int samples=500; float scale=.35f; bool flipX=false,flipY=false; };

V toScan(V world,float extent,const ScanConfig& c);
V fromScan(V p,float extent,const ScanConfig& c);
bool clearSegment(V a,V b,const Grid& sdf,float margin=.012f);
std::vector<Sample> planScan(std::vector<Loop> loops,const Grid& sdf,
                             const ScanConfig& cfg,V& anchor);
} // namespace liquid
