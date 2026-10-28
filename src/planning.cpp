#include "passagelab/core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <numbers>
#include <random>

namespace pl {
namespace {
using Clock = std::chrono::steady_clock;
struct Node { Pose pose; std::size_t parent; };
struct Tree { std::vector<Node> nodes; bool start_root; };
enum class Extend { Trapped, Advanced, Reached, Stopped };

double distance(const Pose& a, const Pose& b, const PlannerConfig& c) {
    return pose_distance(a, b, c.rotation_weight);
}
Pose steer(const Pose& a, const Pose& b, double eta, const PlannerConfig& c) {
    const double d = distance(a, b, c);
    return d <= eta || d <= 1e-14 ? b : interpolate(a, b, eta / d);
}
std::size_t nearest(const Tree& t, const Pose& p, const PlannerConfig& c) {
    std::size_t best = 0;
    double bd = distance(t.nodes[0].pose, p, c);
    for (std::size_t i = 1; i < t.nodes.size(); ++i) {
        const double d = distance(t.nodes[i].pose, p, c);
        if (d < bd) { bd = d; best = i; }
    }
    return best;
}
bool inside(const Bounds& b, const Vec3& p) {
    return (p.array() >= b.min.array()).all() && (p.array() <= b.max.array()).all();
}
Pose sample(std::mt19937_64& rng, const Scene& s) {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    Pose p;
    for (int i = 0; i < 3; ++i)
        p.t[i] = s.sampling.min[i] + u(rng) * (s.sampling.max[i] - s.sampling.min[i]);
    // Shoemake's construction is uniform on SO(3).
    const double a = std::sqrt(1-u(rng)), b = std::sqrt(u(rng));
    const double x = a * std::sin(2*std::numbers::pi*u(rng)), y = a * std::cos(2*std::numbers::pi*u(rng));
    const double z = b * std::sin(2*std::numbers::pi*u(rng)), w = b * std::cos(2*std::numbers::pi*u(rng));
    p.q = Quat(w, x, y, z).normalized();
    return p;
}
std::vector<Pose> chain(const Tree& t, std::size_t i) {
    std::vector<Pose> p;
    while (true) { p.push_back(t.nodes[i].pose); if (i == t.nodes[i].parent) break; i = t.nodes[i].parent; }
    std::reverse(p.begin(), p.end()); return p;
}
std::vector<Pose> joined(const Tree& a, std::size_t ai, const Tree& b, std::size_t bi) {
    auto x = chain(a, ai), y = chain(b, bi);
    if (a.start_root) {
        std::reverse(y.begin(), y.end());
        if (!x.empty() && !y.empty() && distance(x.back(), y.front(), PlannerConfig{}) < 1e-14) y.erase(y.begin());
        x.insert(x.end(), y.begin(), y.end()); return x;
    }
    std::reverse(x.begin(), x.end());
    if (!x.empty() && !y.empty() && distance(x.back(), y.front(), PlannerConfig{}) < 1e-14) y.erase(y.begin());
    x.insert(x.end(), y.begin(), y.end()); return x;
}
void append_edges(PlanningResult& r, const Pose& a, const Pose& b, bool start_tree) {
    if (r.edges.size() < 4000) r.edges.push_back({a,b,start_tree});
}
std::string stop_reason(const std::stop_token& token) { return token.stop_requested() ? "cancelled" : "budget exhausted"; }

bool metadata_matches(const Scene& s, const Path& p) {
    return p.scene_hash == s.hash && p.scene_revision == s.revision;
}
double path_objective(const std::vector<Pose>& p, double rw, double* tr=nullptr, double* rot=nullptr) {
    double t=0, r=0;
    for (std::size_t i=1;i<p.size();++i) { t+=(p[i].t-p[i-1].t).norm(); r+=rotation_distance(p[i-1].q,p[i].q); }
    if(tr)*tr=t; if(rot)*rot=r; return t+rw*r;
}
}

std::string to_string(SolveStatus s) {
    switch(s) { case SolveStatus::Solved:return "solved"; case SolveStatus::InvalidStart:return "invalid_start";
    case SolveStatus::InvalidGoal:return "invalid_goal"; case SolveStatus::BudgetExhausted:return "budget_exhausted";
    case SolveStatus::Cancelled:return "cancelled"; case SolveStatus::ValidationIncomplete:return "validation_incomplete";
    case SolveStatus::Error:return "error"; } return "error";
}

MotionResult validate_path(const Scene& scene, const Path& path, const ValidationConfig& cfg, std::stop_token stop) {
    MotionResult out;
    if (!metadata_matches(scene,path)) { out.status=CheckStatus::Failed; out.reason="path scene hash or revision is stale"; return out; }
    if(path.poses.empty()) { out.status=CheckStatus::Failed; out.reason="path has no poses"; return out; }
    for(const auto& p:path.poses) if(!valid_pose(p)) { out.status=CheckStatus::Failed; out.reason="path contains non-finite or invalid pose"; return out; }
    if ((path.poses.front().t-scene.start.t).norm()>1e-8 || rotation_distance(path.poses.front().q,scene.start.q)>1e-8 ||
        (path.poses.back().t-scene.goal.t).norm()>1e-8 || rotation_distance(path.poses.back().q,scene.goal.q)>1e-8) {
        out.status=CheckStatus::Failed; out.reason="path endpoints do not match scene start and goal"; return out;
    }
    Validator validator(scene);
    auto first=validator.state(path.poses.front(),cfg);
    if(!first.valid) { out.status=CheckStatus::Failed; out.reason="path start is invalid: "+first.reason; out.failure=first; return out; }
    for(std::size_t i=1;i<path.poses.size();++i) {
        if(stop.stop_requested()) { out.status=CheckStatus::Cancelled; out.reason="cancelled"; break; }
        auto m=validator.motion(path.poses[i-1],path.poses[i],cfg,stop); out.samples+=m.samples;
        if(m.status!=CheckStatus::Passed) { out.status=m.status; out.reason="path edge "+std::to_string(i-1)+" failed: "+m.reason; out.failure=m.failure; break; }
        out.status=CheckStatus::Passed;
    }
    if(path.poses.size()==1) out.status=CheckStatus::Passed;
    return out;
}

PlanningResult solve(const Scene& scene,const PlannerConfig& cfg,std::stop_token stop,Progress progress) {
    PlanningResult out; const auto begin=Clock::now();
    auto finish=[&](){out.total_seconds=std::chrono::duration<double>(Clock::now()-begin).count();};
    if(!valid_pose(scene.start)) {out.status=SolveStatus::InvalidStart;out.reason="invalid start pose";finish();return out;}
    if(!valid_pose(scene.goal)) {out.status=SolveStatus::InvalidGoal;out.reason="invalid goal pose";finish();return out;}
    Validator validator(scene); auto a=validator.state(scene.start,cfg.validation);
    if(!a.valid){out.status=SolveStatus::InvalidStart;out.reason=a.reason;out.queries=validator.stats();finish();return out;}
    auto b=validator.state(scene.goal,cfg.validation);
    if(!b.valid){out.status=SolveStatus::InvalidGoal;out.reason=b.reason;out.queries=validator.stats();finish();return out;}
    auto make_path=[&](std::vector<Pose> ps){out.path.poses=std::move(ps);out.path.scene_hash=scene.hash;out.path.scene_revision=scene.revision;out.path.validation=cfg.validation;};
    auto check_candidate=[&](std::vector<Pose> ps)->bool {
        make_path(std::move(ps)); const auto v0=Clock::now(); out.verification=validate_path(scene,out.path,cfg.validation,stop);
        out.validation_seconds+=std::chrono::duration<double>(Clock::now()-v0).count();
        if(out.verification.status==CheckStatus::Passed){out.status=SolveStatus::Solved;out.reason="path independently validated";return true;}
        if(out.verification.status==CheckStatus::Cancelled){out.status=SolveStatus::Cancelled;out.reason=out.verification.reason;return true;}
        if(out.verification.status==CheckStatus::Incomplete){out.status=SolveStatus::ValidationIncomplete;out.reason=out.verification.reason;return true;}
        return false;
    };
    if(distance(scene.start,scene.goal,cfg)<1e-12) { check_candidate({scene.start});out.queries=validator.stats();finish();return out; }
    Tree ta{{Node{scene.start,0}},true}, tb{{Node{scene.goal,0}},false};
    std::mt19937_64 rng(cfg.seed); const auto search_begin=Clock::now();
    auto deadline=[&](){return cfg.time_limit>0 && std::chrono::duration<double>(Clock::now()-begin).count()>=cfg.time_limit;};
    auto budget=[&](){return out.iterations>=cfg.max_iterations || ta.nodes.size()+tb.nodes.size()>=cfg.max_nodes || validator.stats().motions>=cfg.max_queries || deadline() || stop.stop_requested();};
    auto extend=[&](Tree& t,const Pose& target)->std::pair<Extend,std::size_t> {
        if(budget()) return {Extend::Stopped,0};
        auto ni=nearest(t,target,cfg); auto d=distance(t.nodes[ni].pose,target,cfg);
        if(d<1e-12)return {Extend::Reached,ni};
        Pose next=steer(t.nodes[ni].pose,target,std::max(cfg.extension,1e-9),cfg);
        if(distance(t.nodes[ni].pose,next,cfg)<1e-12)return {Extend::Trapped,ni};
        if(out.edges.size()<4000)append_edges(out,t.nodes[ni].pose,next,t.start_root);
        auto m=validator.motion(t.nodes[ni].pose,next,cfg.validation,stop);
        if(m.status!=CheckStatus::Passed)return {m.status==CheckStatus::Cancelled?Extend::Stopped:Extend::Trapped,ni};
        auto idx=t.nodes.size();t.nodes.push_back({next,ni});++out.nodes;return {distance(next,target,cfg)<1e-10?Extend::Reached:Extend::Advanced,idx};
    };
    for(;!budget();) {
        ++out.iterations; if(progress)progress(out.iterations,out.nodes);
        Pose target=std::uniform_real_distribution<double>(0,1)(rng)<std::clamp(cfg.goal_bias,0.0,1.0)?(ta.start_root?scene.goal:scene.start):sample(rng,scene);
        auto [es,ei]=extend(ta,target);
        if(es==Extend::Stopped)break;
        if(es!=Extend::Trapped) {
            Extend cs=Extend::Advanced;std::size_t ci=0;
            while(cs==Extend::Advanced) { if(budget()){cs=Extend::Stopped;break;} auto x=extend(tb,ta.nodes[ei].pose);cs=x.first;ci=x.second; }
            if(cs==Extend::Reached) {
                auto ps=joined(ta,ei,tb,ci); // verify the actual remaining bridge too
                if(check_candidate(std::move(ps)))break;
            }
        }
        std::swap(ta,tb);
    }
    out.search_seconds=std::chrono::duration<double>(Clock::now()-search_begin).count();
    if(out.status!=SolveStatus::Solved && out.status!=SolveStatus::ValidationIncomplete) {
        out.status=stop.stop_requested()?SolveStatus::Cancelled:SolveStatus::BudgetExhausted;out.reason=stop_reason(stop);
    }
    out.nodes=ta.nodes.size()+tb.nodes.size();out.queries=validator.stats();finish();return out;
}

Path optimize(const Scene& scene,const Path& original,const PlannerConfig& cfg,std::stop_token stop) {
    if(validate_path(scene,original,cfg.validation,stop).status!=CheckStatus::Passed)return original;
    Path result=original;std::mt19937_64 rng(cfg.seed);const auto start=Clock::now();
    for(std::uint64_t k=0;k<cfg.shortcut_attempts && result.poses.size()>2;++k) {
        if(stop.stop_requested()||(cfg.time_limit>0&&std::chrono::duration<double>(Clock::now()-start).count()>=cfg.time_limit))break;
        std::uniform_int_distribution<std::size_t> pick(0,result.poses.size()-1);auto i=pick(rng),j=pick(rng);if(i>j)std::swap(i,j);if(j<=i+1)continue;
        std::vector<Pose> candidate(result.poses.begin(),result.poses.begin()+static_cast<std::ptrdiff_t>(i+1));candidate.insert(candidate.end(),result.poses.begin()+static_cast<std::ptrdiff_t>(j),result.poses.end());
        double before=path_objective(result.poses,cfg.rotation_weight),after=path_objective(candidate,cfg.rotation_weight);
        if(after+1e-12>=before)continue;
        Validator v(scene);auto m=v.motion(result.poses[i],result.poses[j],cfg.validation,stop);if(m.status!=CheckStatus::Passed)continue;
        Path trial=result;trial.poses=std::move(candidate);if(validate_path(scene,trial,cfg.validation,stop).status==CheckStatus::Passed)result=std::move(trial);
    }
    return result;
}

PathMetrics evaluate(const Scene& scene,const Path& path,const PlannerConfig& cfg,std::stop_token stop) {
    PathMetrics m; if(!metadata_matches(scene,path)||path.poses.empty())return m;
    m.objective=path_objective(path.poses,cfg.rotation_weight,&m.translation,&m.rotation);
    Validator v(scene);double best=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<path.poses.size();++i) {
        if(stop.stop_requested()){m.complete=false;return m;}
        if(i==0) {auto s=v.state(path.poses[i],cfg.validation,true);++m.samples;if(!s.query.distance){m.complete=false;return m;}if(*s.query.distance<best){best=*s.query.distance;m.min_clearance_pose=path.poses[i];}}
        else {
            const double len=distance(path.poses[i-1],path.poses[i],cfg);
            const auto n=static_cast<std::uint64_t>(std::ceil(len/std::max(cfg.validation.translation_step,1e-9)));
            if(m.samples>cfg.validation.max_samples || n>cfg.validation.max_samples-m.samples){m.complete=false;return m;}
            for(std::uint64_t j=1;j<=n;++j){if(stop.stop_requested()){m.complete=false;return m;}auto p=interpolate(path.poses[i-1],path.poses[i],double(j)/double(n));auto s=v.state(p,cfg.validation,true);++m.samples;if(!s.query.distance){m.complete=false;return m;}if(*s.query.distance<best){best=*s.query.distance;m.min_clearance_pose=p;}}
        }
    }
    if(std::isfinite(best))m.min_sampled_clearance=best;m.complete=true;return m;
}
}
