#include "passagelab/core.hpp"

#include <fcl/fcl.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>

namespace pl {
namespace {
bool finite(const Vec3& v) { return v.array().isFinite().all(); }
bool valid_part(const Part& p) {
    if (!valid_pose(p.pose)) return false;
    if (p.shape == Shape::Box) return finite(p.half) && (p.half.array() > 0).all();
    return std::isfinite(p.radius) && p.radius > 0;
}
Pose normalized(Pose p) { p.q.normalize(); return p; }
Vec3 world_center(const Part& p,const Pose& parent) { return compose(parent,p.pose).t; }
Vec3 half_extents(const Part& p) { return p.shape==Shape::Box ? p.half : Vec3::Constant(p.radius); }
double aabb_distance(const Aabb& a,const Aabb& b) {
    Vec3 d=(a.min-b.max).cwiseMax(b.min-a.max).cwiseMax(Vec3::Zero()); return d.norm();
}
bool aabb_overlap(const Aabb& a,const Aabb& b,double m=0) {
    return (a.min.array() <= b.max.array()+m).all() && (b.min.array() <= a.max.array()+m).all();
}
QueryResult result(Relation r,const Part& a,const Part& b,const char* backend) {
    QueryResult q; q.relation=r; q.body_id=a.id; q.obstacle_id=b.id; q.backend=backend; return q;
}
double sat(const Part& a,const Pose& ap,const Part& b,double tol,Relation& rel) {
    const Pose ta=compose(ap,a.pose),tb=b.pose;
    Eigen::Matrix3d A=ta.q.toRotationMatrix(),B=tb.q.toRotationMatrix();
    const Vec3 ea=a.half, eb=b.half, delta=tb.t-ta.t;
    double min_gap=std::numeric_limits<double>::infinity(); bool touch=false;
    auto axis_check=[&](Vec3 axis) {
        const double n=axis.norm(); if(n<1e-12) return true; axis/=n;
        const double ra=(A.transpose()*axis).cwiseAbs().dot(ea);
        const double rb=(B.transpose()*axis).cwiseAbs().dot(eb);
        const double gap=std::abs(delta.dot(axis))-ra-rb; min_gap=std::min(min_gap,gap);
        if(gap>tol) return false; if(gap>=-tol) touch=true; return true;
    };
    for(int i=0;i<3;i++) if(!axis_check(A.col(i))||!axis_check(B.col(i))) { rel=Relation::Separated; return min_gap; }
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) if(!axis_check(A.col(i).cross(B.col(j)))) { rel=Relation::Separated; return min_gap; }
    rel=touch?Relation::Touching:Relation::Overlapping; return min_gap;
}
QueryResult sphere_pair(const Part& a,const Pose& ap,const Part& b,double tol) {
    Pose ta=compose(ap,a.pose); Vec3 d=b.pose.t-ta.t; double n=d.norm(),gap=n-a.radius-b.radius;
    Relation r=gap>tol?Relation::Separated:(gap>=-tol?Relation::Touching:Relation::Overlapping);
    auto q=result(r,a,b,"analytic-sphere");
    if(r==Relation::Separated) { Vec3 u=d/n; q.distance=gap; q.on_body=ta.t+u*a.radius; q.on_obstacle=b.pose.t-u*b.radius; }
    return q;
}
QueryResult sphere_box(const Part& s,const Pose& sp,const Part& box,bool sphere_first,double tol) {
    Pose ts=compose(sp,s.pose); Eigen::Matrix3d R=box.pose.q.toRotationMatrix();
    Vec3 local=R.transpose()*(ts.t-box.pose.t), closest=local.cwiseMax(-box.half).cwiseMin(box.half);
    Vec3 diff=local-closest; double d=diff.norm();
    Relation r; double gap;
    if(d>tol) { gap=d-s.radius; r=gap>tol?Relation::Separated:(gap>=-tol?Relation::Touching:Relation::Overlapping); }
    else { double face=(box.half-local.cwiseAbs()).minCoeff(); gap=-(face+s.radius); r=Relation::Overlapping; }
    auto q=result(r,sphere_first?s:box,sphere_first?box:s,"analytic-sphere-box");
    if(r==Relation::Separated) {
        Vec3 u=R*(diff/d); Vec3 ps=ts.t-u*s.radius, pb=box.pose.t+R*closest;
        q.distance=gap; q.on_body=sphere_first?ps:pb; q.on_obstacle=sphere_first?pb:ps;
    }
    return q;
}
struct Node { Aabb box; int left=-1,right=-1; std::vector<size_t> ids; };
}

Pose compose(const Pose& a,const Pose& b) { return {a.t+a.q*b.t,(a.q*b.q).normalized()}; }
Pose inverse(const Pose& a) { Quat q=a.q.normalized().conjugate(); return {q*(-a.t),q}; }
Pose interpolate(const Pose& a,const Pose& b,double u) { Quat q=b.q; if(a.q.dot(q)<0) q.coeffs()*=-1; return {(1-u)*a.t+u*b.t,a.q.slerp(u,q).normalized()}; }
double rotation_distance(const Quat& a,const Quat& b) { return 2*std::acos(std::clamp(std::abs(a.normalized().dot(b.normalized())),0.0,1.0)); }
double pose_distance(const Pose& a,const Pose& b,double w) { return std::hypot((a.t-b.t).norm(),w*rotation_distance(a.q,b.q)); }
bool valid_pose(const Pose& p) { double n=p.q.norm(); return finite(p.t)&&p.q.coeffs().array().isFinite().all()&&std::isfinite(n)&&n>1e-12&&std::abs(n-1)<0.1; }
Aabb world_aabb(const Part& p,const Pose& parent) {
    Pose t=compose(parent,p.pose); Vec3 e;
    if(p.shape==Shape::Sphere) e=Vec3::Constant(p.radius);
    else e=t.q.toRotationMatrix().cwiseAbs()*p.half;
    return {t.t-e,t.t+e};
}
double body_radius(const Scene& s) { double r=0; for(const auto& p:s.body) { Pose t=p.pose; r=std::max(r,t.t.norm()+(p.shape==Shape::Box?p.half.norm():p.radius)); } return r; }
std::vector<std::string> scene_errors(const Scene& s) {
    std::vector<std::string> e; if(s.body.empty()) e.push_back("body is empty");
    for(const auto& p:s.body) if(!valid_part(p)) e.push_back("invalid body part: "+p.id);
    for(const auto& p:s.obstacles) if(!valid_part(p)) e.push_back("invalid obstacle: "+p.id);
    if(!valid_pose(s.start)||!valid_pose(s.goal)) e.push_back("invalid start or goal pose");
    if(!finite(s.sampling.min)||!finite(s.sampling.max)||!finite(s.physical.min)||!finite(s.physical.max)||
       (s.sampling.min.array()>=s.sampling.max.array()).any()||(s.physical.min.array()>=s.physical.max.array()).any()) e.push_back("invalid bounds");
    return e;
}
QueryResult query_pair(const Part& a,const Pose& ap,const Part& b,double tol) {
    if(!valid_part(a)||!valid_part(b)||!valid_pose(ap)||!std::isfinite(tol)||tol<0) return result(Relation::Indeterminate,a,b,"invalid-input");
    if(a.shape==Shape::Sphere&&b.shape==Shape::Sphere) return sphere_pair(a,ap,b,tol);
    if(a.shape==Shape::Sphere) return sphere_box(a,ap,b,true,tol);
    if(b.shape==Shape::Sphere) { Part world_body=a; world_body.pose=compose(ap,a.pose); return sphere_box(b,Pose{},world_body,false,tol); }
    Relation rel; sat(a,ap,b,tol,rel); auto q=result(rel,a,b,"obb-sat");
    if(rel==Relation::Separated) {
        auto ga=std::make_shared<fcl::Boxd>(2*a.half.x(),2*a.half.y(),2*a.half.z());
        auto gb=std::make_shared<fcl::Boxd>(2*b.half.x(),2*b.half.y(),2*b.half.z());
        Pose ta=compose(ap,a.pose); fcl::Transform3d xa= fcl::Transform3d::Identity(), xb=fcl::Transform3d::Identity();
        xa.linear()=ta.q.toRotationMatrix(); xa.translation()=ta.t; xb.linear()=b.pose.q.toRotationMatrix(); xb.translation()=b.pose.t;
        fcl::DistanceRequestd req(true); fcl::DistanceResultd res; double d=fcl::distance(ga.get(),xa,gb.get(),xb,req,res);
        if(std::isfinite(d)&&d>=0) { q.distance=d; q.on_body=res.nearest_points[0]; q.on_obstacle=res.nearest_points[1]; }
        else { q.relation=Relation::Indeterminate; q.reason="FCL distance failed"; }
    }
    return q;
}

struct Validator::Impl {
    Scene scene; QueryStats stats; std::vector<Node> nodes; int root=-1;
    explicit Impl(const Scene& s):scene(s) {
        std::vector<size_t> ids(s.obstacles.size()); std::iota(ids.begin(),ids.end(),0);
        std::function<int(std::vector<size_t>)> build=[&](std::vector<size_t> v)->int {
            if(v.empty()) return -1; Node n; n.box=world_aabb(s.obstacles[v[0]]);
            for(size_t i:v) { auto b=world_aabb(s.obstacles[i]); n.box.min=n.box.min.cwiseMin(b.min); n.box.max=n.box.max.cwiseMax(b.max); }
            int idx=(int)nodes.size(); nodes.push_back(n);
            if(v.size()<=4) nodes[idx].ids=std::move(v);
            else { Vec3 extent=n.box.max-n.box.min; Eigen::Index ax; extent.maxCoeff(&ax);
                std::sort(v.begin(),v.end(),[&](size_t a,size_t b){return world_aabb(s.obstacles[a]).min[ax]+world_aabb(s.obstacles[a]).max[ax] < world_aabb(s.obstacles[b]).min[ax]+world_aabb(s.obstacles[b]).max[ax];});
                auto mid=v.begin()+v.size()/2; std::vector<size_t> l(v.begin(),mid),r(mid,v.end()); nodes[idx].left=build(std::move(l)); nodes[idx].right=build(std::move(r)); }
            return idx;
        }; root=build(std::move(ids));
    }
    std::vector<size_t> candidates(const Aabb& box,double margin,bool bvh) {
        std::vector<size_t> out; if(!bvh||root<0) { for(size_t i=0;i<scene.obstacles.size();++i) out.push_back(i); return out; }
        std::vector<int> st{root}; while(!st.empty()) { int i=st.back();st.pop_back(); ++stats.broad; const auto& n=nodes[i]; if(!aabb_overlap(box,n.box,margin)) continue;
            if(n.left<0&&n.right<0) out.insert(out.end(),n.ids.begin(),n.ids.end()); else {if(n.left>=0)st.push_back(n.left);if(n.right>=0)st.push_back(n.right);} }
        return out;
    }
};
Validator::Validator(const Scene& s):impl_(std::make_unique<Impl>(s)){} Validator::~Validator()=default;
QueryStats Validator::stats() const { return impl_->stats; }
StateResult Validator::state(const Pose& input,const ValidationConfig& c,bool want_distance) {
    auto& x=*impl_; ++x.stats.states; StateResult out;
    if(x.scene.body.empty()||!valid_pose(input)||!std::isfinite(c.margin)||c.margin<0||!std::isfinite(c.tolerance)||c.tolerance<0) {out.reason="invalid input";out.query.relation=Relation::Indeterminate;return out;}
    Pose pose=normalized(input); if((pose.t.array()<x.scene.sampling.min.array()).any()||(pose.t.array()>x.scene.sampling.max.array()).any()){out.reason="sampling bounds";return out;}
    double nearest=std::numeric_limits<double>::infinity(); bool have=false;
    for(const auto& p:x.scene.body) { Aabb wb=world_aabb(p,pose); if((wb.min.array()<x.scene.physical.min.array()).any()||(wb.max.array()>x.scene.physical.max.array()).any()){out.reason="physical bounds";return out;}
        auto ids=x.candidates(wb,c.margin,c.use_bvh);
        if(want_distance) { ids.resize(x.scene.obstacles.size());std::iota(ids.begin(),ids.end(),0);
            std::stable_sort(ids.begin(),ids.end(),[&](size_t a,size_t b){return aabb_distance(wb,world_aabb(x.scene.obstacles[a]))<aabb_distance(wb,world_aabb(x.scene.obstacles[b]));}); }
        for(size_t oi:ids) { double lower=aabb_distance(wb,world_aabb(x.scene.obstacles[oi]));
            if(want_distance&&have&&lower>nearest) continue;
            ++x.stats.narrow; auto q=query_pair(p,pose,x.scene.obstacles[oi],c.tolerance);
            if(q.relation==Relation::Indeterminate){out.reason=q.reason.empty()?"indeterminate geometry":q.reason;out.query=q;return out;}
            if(q.relation!=Relation::Separated || (q.distance&&*q.distance<=c.margin+c.tolerance)){out.reason=q.relation==Relation::Separated?"clearance margin":"collision or contact";out.query=q;return out;}
            if(q.distance&&*q.distance<nearest){nearest=*q.distance;out.query=q;have=true;}
        }
    }
    if(want_distance&&have) out.query.distance=nearest;
    out.valid=true; out.reason="valid"; return out;
}
MotionResult Validator::motion(const Pose& a,const Pose& b,const ValidationConfig& c,std::stop_token stop) {
    auto& x=*impl_; ++x.stats.motions; MotionResult m;
    if(!valid_pose(a)||!valid_pose(b)||!std::isfinite(c.translation_step)||c.translation_step<=0||!std::isfinite(c.rotation_step)||c.rotation_step<=0||!std::isfinite(c.surface_step)||c.surface_step<=0||!std::isfinite(c.margin)||c.margin<0||!std::isfinite(c.tolerance)||c.tolerance<0||c.max_samples<2){m.reason="invalid motion configuration or pose";return m;}
    double L=(a.t-b.t).norm(),theta=rotation_distance(a.q,b.q),travel=L+body_radius(x.scene)*theta;
    double nd=std::max({1.0,std::ceil(L/c.translation_step),std::ceil(theta/c.rotation_step),std::ceil(travel/c.surface_step)});
    if(!std::isfinite(nd)||nd+1>double(c.max_samples)){m.reason="sample limit exceeded";m.status=CheckStatus::Incomplete;return m;}
    auto n=static_cast<std::uint64_t>(nd);
    for(std::uint64_t i=0;i<=n;++i) { if(stop.stop_requested()){m.status=CheckStatus::Cancelled;m.reason="cancelled";return m;}
        auto s=state(interpolate(a,b,double(i)/n),c); ++m.samples; ++x.stats.samples;
        if(!s.valid){m.status=CheckStatus::Failed;m.failed_u=double(i)/n;m.failure=s;m.reason=s.reason;return m;} }
    m.status=CheckStatus::Passed; return m;
}
std::optional<double> ray_box(const Vec3& o,const Vec3& d,const Part& p,const Pose& parent) {
    if(!finite(o)||!finite(d)||d.squaredNorm()<1e-24||p.shape!=Shape::Box||!valid_part(p)||!valid_pose(parent)) return std::nullopt;
    Pose t=compose(parent,p.pose); Vec3 lo=t.q.conjugate()*(o-t.t),ld=t.q.conjugate()*d; double tmin=0,tmax=std::numeric_limits<double>::infinity();
    for(int i=0;i<3;i++){ if(std::abs(ld[i])<1e-15){if(lo[i]<-p.half[i]||lo[i]>p.half[i])return std::nullopt;continue;}
        double a=(-p.half[i]-lo[i])/ld[i],b=(p.half[i]-lo[i])/ld[i];if(a>b)std::swap(a,b);tmin=std::max(tmin,a);tmax=std::min(tmax,b);if(tmin>tmax)return std::nullopt; }
    return tmin;
}
} // namespace pl
