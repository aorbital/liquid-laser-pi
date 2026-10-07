#include "contours.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace liquid {
float area(const Loop& p){double s=0;for(size_t i=0;i<p.size();++i){V a=p[i],b=p[(i+1)%p.size()];s+=a.x*b.y-a.y*b.x;}return float(s*.5);}
float perimeter(const Loop& p){float s=0;for(size_t i=0;i<p.size();++i)s+=norm(p[(i+1)%p.size()]-p[i]);return s;}
Loop resample(const Loop& p,int count){Loop out;if(p.size()<3||count<3)return out;std::vector<float> c(p.size()+1,0);for(size_t i=0;i<p.size();++i)c[i+1]=c[i]+norm(p[(i+1)%p.size()]-p[i]);if(c.back()<1e-6f)return out;out.reserve(count);size_t seg=0;for(int i=0;i<count;++i){float target=c.back()*i/count;while(seg+1<p.size()&&c[seg+1]<target)++seg;float t=(target-c[seg])/std::max(1e-9f,c[seg+1]-c[seg]);out.push_back(p[seg]+(p[(seg+1)%p.size()]-p[seg])*t);}return out;}
void smooth(Loop& p,int iterations){if(p.empty())return;Loop tmp(p.size());int n=int(p.size());for(int k=0;k<iterations;++k){for(int i=0;i<n;++i)tmp[i]=(p[(i+n-1)%n]+p[i]*2+p[(i+1)%n])*.25f;p.swap(tmp);}}

std::vector<Loop> Contours::extract(const Grid& field,float level,float minArea,int maxLoops,int budget,int smoothing){
 int n=field.size,H=n*(n-1);edgeVertex_.assign(2*H,-1);vertices_.clear();adjacent_.clear();
 auto connect=[&](int a,int b){for(int k=0;k<2;++k)if(adjacent_[a][k]<0){adjacent_[a][k]=b;return;}};
 for(int y=0;y<n-1;++y)for(int x=0;x<n-1;++x){
  std::array<float,4> v{{field.data[y*n+x],field.data[y*n+x+1],field.data[(y+1)*n+x+1],field.data[(y+1)*n+x]}};
  std::array<V,4> p{{field.position(x,y),field.position(x+1,y),field.position(x+1,y+1),field.position(x,y+1)}};
  std::array<int,4> key{{y*(n-1)+x,H+y*n+x+1,(y+1)*(n-1)+x,H+y*n+x}};
  int code=0;for(int k=0;k<4;++k)if(v[k]>=level)code|=1<<k;if(code==0||code==15)continue;
  auto vertex=[&](int e){int& id=edgeVertex_[key[e]];if(id<0){float den=v[(e+1)%4]-v[e];float t=std::abs(den)>1e-9f?(level-v[e])/den:.5f;id=int(vertices_.size());vertices_.push_back(p[e]+(p[(e+1)%4]-p[e])*std::clamp(t,0.f,1.f));adjacent_.push_back({-1,-1});}return id;};
  auto link=[&](int a,int b){int ia=vertex(a),ib=vertex(b);connect(ia,ib);connect(ib,ia);};
  switch(code){case 1:case 14:link(3,0);break;case 2:case 13:link(0,1);break;case 3:case 12:link(3,1);break;case 4:case 11:link(1,2);break;case 6:case 9:link(0,2);break;case 7:case 8:link(3,2);break;case 5:case 10:{bool high=(v[0]+v[1]+v[2]+v[3])*.25f>=level;if((code==5)==high){link(0,1);link(2,3);}else{link(3,0);link(1,2);}break;}}
 }
 visited_.assign(vertices_.size(),0);std::vector<Loop> loops;
 for(int start=0;start<int(vertices_.size());++start)if(!visited_[start]){Loop loop;int cur=start,prev=-1;bool closed=false;for(size_t k=0;k<=vertices_.size();++k){if(cur<0)break;if(visited_[cur]){closed=cur==start;break;}visited_[cur]=1;loop.push_back(vertices_[cur]);auto a=adjacent_[cur];int next=a[0]!=prev?a[0]:a[1];prev=cur;cur=next;}if(closed&&loop.size()>=8&&std::abs(area(loop))>=minArea)loops.push_back(std::move(loop));}
 std::sort(loops.begin(),loops.end(),[](const Loop&a,const Loop&b){return std::abs(area(a))>std::abs(area(b));});if(int(loops.size())>maxLoops)loops.resize(maxLoops);if(loops.empty())return loops;
 float total=0;for(const auto&l:loops)total+=perimeter(l);int free=std::max(0,budget-int(loops.size())*12),remaining=budget;for(size_t i=0;i<loops.size();++i){int count=i+1==loops.size()?remaining:12+int(free*perimeter(loops[i])/std::max(total,1e-6f));remaining-=count;loops[i]=resample(loops[i],std::max(12,count));smooth(loops[i],smoothing);}return loops;
}
V toScan(V world,float extent,const ScanConfig& c){V p=box(world/extent,1)*c.scale;if(c.flipX)p.x=-p.x;if(c.flipY)p.y=-p.y;return p;}
V fromScan(V p,float extent,const ScanConfig& c){if(c.flipX)p.x=-p.x;if(c.flipY)p.y=-p.y;return p*(extent/c.scale);}
bool clearSegment(V a,V b,const Grid&sdf,float margin){if(sdf.size<2)return true;int steps=std::max(1,int(std::ceil(norm(b-a)/(sdf.cell()*.45f))));for(int k=0;k<=steps;++k)if(sdf.sample(a+(b-a)*(float(k)/steps))<margin)return false;return true;}
std::vector<Sample> planScan(std::vector<Loop> loops,const Grid&sdf,const ScanConfig&cfg,V&anchor){std::vector<Sample> r;r.reserve(cfg.samples);loops.erase(std::remove_if(loops.begin(),loops.end(),[](const Loop&p){return p.size()<3||perimeter(p)<.025f;}),loops.end());if(loops.empty()){r.assign(cfg.samples,{anchor,false});return r;}std::vector<Loop> ordered;V current=fromScan(anchor,sdf.extent,cfg);while(!loops.empty()){size_t best=0,index=0;float dist=std::numeric_limits<float>::max();for(size_t i=0;i<loops.size();++i)for(size_t j=0;j<loops[i].size();++j){float d=norm(loops[i][j]-current);if(d<dist){dist=d;best=i;index=j;}}Loop sel=std::move(loops[best]);loops.erase(loops.begin()+best);std::rotate(sel.begin(),sel.begin()+index,sel.end());current=sel.front();ordered.push_back(std::move(sel));}constexpr float blankStep=.025f;std::vector<int> travels;float total=0;int overhead=0;V last=anchor;for(const auto&l:ordered){V start=toScan(l.front(),sdf.extent,cfg);int t=std::max(2,int(std::ceil(norm(start-last)/blankStep))+1);travels.push_back(t);overhead+=t+4;last=start;total+=perimeter(l);}int returning=std::max(2,int(std::ceil(norm(last-anchor)/blankStep))+1);overhead+=returning;if(overhead+int(ordered.size())*12>cfg.samples){ordered.pop_back();return planScan(std::move(ordered),sdf,cfg,anchor);}int available=cfg.samples-overhead;int extra=available-int(ordered.size())*12;auto travel=[&](V to,int count){V start=r.empty()?anchor:r.back().xy;for(int k=1;k<=count;++k)r.push_back({start+(to-start)*(float(k)/count),false});};for(size_t i=0;i<ordered.size();++i){int count=i+1==ordered.size()?available:12+int(extra*perimeter(ordered[i])/std::max(total,1e-6f));available-=count;auto curve=resample(ordered[i],count-1);V start=toScan(curve.front(),sdf.extent,cfg);travel(start,travels[i]);r.push_back({start,false});r.push_back({start,false});V prev=curve.front();for(int j=0;j<count;++j){V p=curve[j%curve.size()],scan=toScan(p,sdf.extent,cfg);V prevScan=r.back().xy;bool lit=norm(scan-prevScan)<=.03f&&clearSegment(prev,p,sdf);r.push_back({scan,lit});prev=p;}r.push_back({start,false});r.push_back({start,false});}travel(anchor,returning);if(int(r.size())!=cfg.samples)throw std::runtime_error("scan budget accounting error");return r;}
} // namespace liquid
