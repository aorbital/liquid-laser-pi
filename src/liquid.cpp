#include "liquid.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace liquid {

float dot(V a, V b) { return a.x*b.x + a.y*b.y; }
float norm(V a) { return std::sqrt(dot(a,a)); }
V unit(V a) { return a / std::max(norm(a), 1e-8f); }
V limited(V a, float m) { return a * std::min(1.f, m/std::max(norm(a), 1e-8f)); }
bool finite(V a) { return std::isfinite(a.x) && std::isfinite(a.y); }
V box(V a, float e) { return {std::clamp(a.x,-e,e), std::clamp(a.y,-e,e)}; }

Grid::Grid(int n,float e,float fill): size(n),extent(e),data(n*n,fill) {}
float Grid::cell() const { return 2*extent/(size-1); }
V Grid::position(int x,int y) const { return {x*cell()-extent, y*cell()-extent}; }

float Grid::sample(V p) const {
    if(size < 2) return 10.f;
    const float c = cell();
    float x = std::clamp((p.x+extent)/c, 0.f, float(size-1));
    float y = std::clamp((p.y+extent)/c, 0.f, float(size-1));
    int ix = std::min(int(x), size-2), iy = std::min(int(y), size-2);
    float a=x-ix, b=y-iy;
    float top = data[iy*size+ix]*(1-a) + data[iy*size+ix+1]*a;
    float bot = data[(iy+1)*size+ix]*(1-a) + data[(iy+1)*size+ix+1]*a;
    return top*(1-b) + bot*b;
}
V Grid::normal(V p) const {
    float d=cell();
    return unit({
        sample({p.x+d,p.y})-sample({p.x-d,p.y}),
        sample({p.x,p.y+d})-sample({p.x,p.y-d})
    });
}

V Obstacle::velocity(V p) const {
    if(!hasPrevious || frameDt<.005f || frameDt>.2f) return {};
    float change = previous.sample(p)-sdf.sample(p);
    if(std::abs(change)>.4f) return {};
    return sdf.normal(p)*std::clamp(change/frameDt,-3.f,3.f);
}

V Simulation::clampContainer(V p,float margin) const {
    return {
        std::clamp(p.x,cfg_.lower.x+margin,cfg_.upper.x-margin),
        std::clamp(p.y,cfg_.lower.y+margin,cfg_.upper.y-margin)
    };
}

void Simulation::neighbors() {
    std::fill(head_.begin(),head_.end(),-1);
    for(int i=0;i<int(particles.size());++i) {
        int x=std::clamp(int((particles[i].p.x+cfg_.extent)/h_),0,bins_-1);
        int y=std::clamp(int((particles[i].p.y+cfg_.extent)/h_),0,bins_-1);
        int k=y*bins_+x; next_[i]=head_[k]; head_[k]=i;
    }
    pairs_.clear();
    for(int i=0;i<int(particles.size());++i) {
        int x=std::clamp(int((particles[i].p.x+cfg_.extent)/h_),0,bins_-1);
        int y=std::clamp(int((particles[i].p.y+cfg_.extent)/h_),0,bins_-1);
        for(int yy=std::max(0,y-1);yy<=std::min(bins_-1,y+1);++yy)
            for(int xx=std::max(0,x-1);xx<=std::min(bins_-1,x+1);++xx)
                for(int j=head_[yy*bins_+xx];j>=0;j=next_[j]) if(j>i) {
                    V d=particles[j].p-particles[i].p;
                    float r2=dot(d,d);
                    if(r2<h_*h_) {
                        float r=std::sqrt(std::max(r2,1e-12f));
                        V dir=r2>1e-12f?d/r:V{((i+j)&1)?1.f:-1.f,0};
                        pairs_.push_back({i,j,dir,1-r/h_});
                    }
                }
    }
}

void Simulation::densities() {
    std::fill(rho_.begin(),rho_.end(),0.f);
    std::fill(nearRho_.begin(),nearRho_.end(),0.f);
    for(const auto& pair:pairs_) {
        float q2=pair.q*pair.q, q3=q2*pair.q;
        rho_[pair.a]+=q2; rho_[pair.b]+=q2;
        nearRho_[pair.a]+=q3; nearRho_[pair.b]+=q3;
    }
}

void Simulation::collide(V& p,const Obstacle& o) const {
    float margin=std::max(.012f,spacing_*.12f);
    p=clampContainer(p,margin);
    for(int k=0;k<4;++k) {
        float d=o.sdf.sample(p);
        if(d>=margin) break;
        V n=o.sdf.normal(p);
        if(dot(n,n)<.1f) {
            float best=d; V bestP=p; float step=std::max(spacing_,.03f);
            for(V a:std::array<V,4>{{{step,0},{-step,0},{0,step},{0,-step}}}) {
                V q=clampContainer(p+a,margin);
                float value=o.sdf.sample(q);
                if(value>best){best=value;bestP=q;}
            }
            if(best<=d) break;
            p=bestP;
        } else {
            p=clampContainer(p+n*std::min(margin-d,.25f),margin);
        }
    }
}

Simulation::Simulation(Config c): cfg_(c) {
    if(c.particles<10||c.particles>1000||c.field<16||c.field>512||
       c.substeps<1||c.iterations<1||c.upper.x-c.lower.x<.2f||c.upper.y-c.lower.y<.2f)
        throw std::invalid_argument("invalid simulation configuration");

    spacing_=std::sqrt(PI*c.radius*c.radius/(c.particles*.8660254f));
    h_=spacing_*2.2f;
    bins_=int(std::ceil(2*c.extent/h_))+1;
    head_.resize(bins_*bins_); next_.resize(c.particles);
    particles.resize(c.particles); rho_.resize(c.particles); nearRho_.resize(c.particles);
    pressure_.resize(c.particles); nearPressure_.resize(c.particles);
    correction_.resize(c.particles); dv_.resize(c.particles);
    pairs_.reserve(c.particles*40);

    for(int i=0;i<c.particles;++i) {
        float r=c.radius*std::sqrt((i+.5f)/c.particles), a=i*2.39996323f;
        particles[i].p=V{r*std::cos(a),r*std::sin(a)}+(c.lower+c.upper)*.5f;
    }

    neighbors(); densities();
    auto sorted=rho_; std::sort(sorted.begin(),sorted.end());
    rest_=sorted[sorted.size()*7/10];
    for(int i=0;i<=1024;++i) gaussian_[i]=std::exp(-4.5f*i/1024);
}

void Simulation::step(float dt,const Obstacle& o) {
    time_+=dt;
    for(auto& a:particles) {
        a.old=a.p;
        V flow={std::sin(a.p.y*2.8f+time_*.43f),std::cos(a.p.x*2.3f-time_*.37f)};
        a.v+=flow*(cfg_.flow*dt);

        float distance=o.sdf.sample(a.p);
        if(distance<cfg_.influence) {
            V n=o.sdf.normal(a.p), bodyV=o.velocity(a.p);
            float w=std::clamp(1-distance/cfg_.influence,0.f,1.f); w*=w;
            a.v+=n*(cfg_.repulsion*2.1f*w*dt);
            float push=std::max(0.f,dot(bodyV-a.v,n));
            a.v+=n*(push*(1-std::exp(-14.f*dt))*w);
        }

        a.v=limited(a.v*std::exp(-(.35f+cfg_.viscosity*.8f)*dt),2.5f);
        a.p+=a.v*dt;
        collide(a.p,o);
    }

    for(int iteration=0;iteration<cfg_.iterations;++iteration) {
        neighbors(); densities(); std::fill(correction_.begin(),correction_.end(),V{});
        for(int i=0;i<int(particles.size());++i) {
            pressure_[i]=std::max(100.f*(rho_[i]-rest_),-44.f*cfg_.cohesion);
            nearPressure_[i]=35.f*nearRho_[i];
        }
        for(const auto& pair:pairs_) {
            float amount=dt*dt*((pressure_[pair.a]+pressure_[pair.b])*pair.q
                +(nearPressure_[pair.a]+nearPressure_[pair.b])*pair.q*pair.q)*.5f;
            V d=pair.direction*std::clamp(amount,-spacing_*.08f,spacing_*.08f);
            correction_[pair.a]-=d; correction_[pair.b]+=d;
        }
        for(int i=0;i<int(particles.size());++i) {
            particles[i].p+=limited(correction_[i],spacing_*.2f);
            collide(particles[i].p,o);
        }
    }

    for(auto& a:particles) a.v=limited((a.p-a.old)/dt,2.5f);

    neighbors(); std::fill(dv_.begin(),dv_.end(),V{});
    for(const auto& pair:pairs_) {
        V d=(particles[pair.b].v-particles[pair.a].v)*(pair.q*cfg_.viscosity*.035f);
        dv_[pair.a]+=d; dv_[pair.b]-=d;
    }
    for(int i=0;i<int(particles.size());++i) {
        auto& a=particles[i];
        a.v=limited(a.v+dv_[i],2.5f);
        if(!finite(a.p)||!finite(a.v)) throw std::runtime_error("non-finite particle state");
    }
}

void Simulation::density(Grid& out,const Obstacle& o) const {
    std::fill(out.data.begin(),out.data.end(),0.f);
    float sigma=spacing_*.65f, support=3*sigma, step=out.cell();
    float mass=PI*cfg_.radius*cfg_.radius/cfg_.particles;
    float amplitude=mass/(2*PI*sigma*sigma);

    for(const auto& a:particles) {
        if(o.sdf.sample(a.p)<0) continue;
        int x0=std::max(0,int(std::floor((a.p.x-support+out.extent)/step)));
        int x1=std::min(out.size-1,int(std::ceil((a.p.x+support+out.extent)/step)));
        int y0=std::max(0,int(std::floor((a.p.y-support+out.extent)/step)));
        int y1=std::min(out.size-1,int(std::ceil((a.p.y+support+out.extent)/step)));
        for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
            V d=out.position(x,y)-a.p;
            float u=dot(d,d)/(support*support);
            if(u<1) out.data[y*out.size+x]+=amplitude*gaussian_[std::min(1024,int(u*1024))];
        }
    }

    for(int y=0;y<out.size;++y) for(int x=0;x<out.size;++x) {
        V p=out.position(x,y);
        float body=o.sdf.sample(p)-.012f;
        float wall=std::min({p.x-cfg_.lower.x,cfg_.upper.x-p.x,p.y-cfg_.lower.y,cfg_.upper.y-p.y});
        out.data[y*out.size+x]*=std::clamp(std::min(body,wall)/std::max(step,.012f),0.f,1.f);
    }
}

} // namespace liquid
