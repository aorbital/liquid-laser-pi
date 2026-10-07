#pragma once
#include <array>
#include <vector>

namespace liquid {

constexpr float PI = 3.14159265358979323846f;

struct V {
    float x = 0, y = 0;
    V operator+(V b) const { return {x+b.x, y+b.y}; }
    V operator-(V b) const { return {x-b.x, y-b.y}; }
    V operator*(float b) const { return {x*b, y*b}; }
    V operator/(float b) const { return {x/b, y/b}; }
    V& operator+=(V b) { x += b.x; y += b.y; return *this; }
    V& operator-=(V b) { x -= b.x; y -= b.y; return *this; }
};

float dot(V a, V b);
float norm(V a);
V unit(V a);
V limited(V a, float m);
bool finite(V a);
V box(V a, float e);

struct Grid {
    int size = 0;
    float extent = 1.4f;
    std::vector<float> data;

    Grid() = default;
    Grid(int n, float e, float fill = 0);
    float cell() const;
    V position(int x, int y) const;
    float sample(V p) const;
    V normal(V p) const;
};

struct Obstacle {
    Grid sdf, previous;
    float frameDt = 1.f/30.f;
    bool hasPrevious = false;
    V velocity(V p) const;
};

struct Config {
    int particles = 200;
    int field = 128;
    int substeps = 2;
    int iterations = 1;
    int smoothing = 2;
    int maxLoops = 6;
    int points = 280;

    float extent = 1.4f;
    float radius = .62f;
    float cohesion = .95f;
    float viscosity = .45f;
    float repulsion = .65f;
    float influence = .14f;
    float flow = .010f;
    float threshold = .34f;
    float minArea = .004f;

    V lower{-1.4f,-1.4f};
    V upper{ 1.4f, 1.4f};
};

struct Particle { V p, v, old; };
struct Pair { int a, b; V direction; float q; };

class Simulation {
    Config cfg_;
    float spacing_ = 0, h_ = 0, rest_ = 0, time_ = 0;
    int bins_ = 0;
    std::vector<int> head_, next_;
    std::vector<Pair> pairs_;
    std::vector<float> rho_, nearRho_, pressure_, nearPressure_;
    std::vector<V> correction_, dv_;
    std::array<float,1025> gaussian_{};

    V clampContainer(V p, float margin) const;
    void neighbors();
    void densities();
    void collide(V& p, const Obstacle& o) const;

public:
    std::vector<Particle> particles;
    explicit Simulation(Config c);
    void step(float dt, const Obstacle& o);
    void density(Grid& out, const Obstacle& o) const;
};

} // namespace liquid
