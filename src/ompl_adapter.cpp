#include "passagelab/core.hpp"

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <chrono>
#include <atomic>

namespace pl {
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace {
Pose from_state(const ob::State* state) {
    const auto* s=state->as<ob::SE3StateSpace::StateType>();
    Pose p; p.t=Vec3(s->getX(),s->getY(),s->getZ()); p.q=Quat(s->rotation().w,s->rotation().x,s->rotation().y,s->rotation().z);
    return p;
}
void to_state(const Pose& p,ob::State* state) {
    auto* s=state->as<ob::SE3StateSpace::StateType>();s->setXYZ(p.t.x(),p.t.y(),p.t.z());
    s->rotation().setQuaternion(p.q.x(),p.q.y(),p.q.z(),p.q.w());
}
class SharedMotionValidator final : public ob::MotionValidator {
public:
    SharedMotionValidator(const ob::SpaceInformationPtr& si,const Scene& scene,const ValidationConfig& cfg,std::stop_token st,
                          std::atomic<std::uint64_t>& count)
      : ob::MotionValidator(si),validator_(scene),cfg_(cfg),stop_(st),count_(count) {}
    bool checkMotion(const ob::State* a,const ob::State* b) const override { return checkMotion(a,b,nullptr); }
    bool checkMotion(const ob::State* a,const ob::State* b,std::pair<ob::State*,double>* last) const override {
        ++count_; if(stop_.stop_requested())return false;
        auto r=validator_.motion(from_state(a),from_state(b),cfg_,stop_);
        if(last && r.status!=CheckStatus::Passed) { si_->getStateSpace()->interpolate(a,b,std::clamp(r.failed_u,0.0,1.0),last->first);last->second=r.failed_u; }
        return r.status==CheckStatus::Passed;
    }
private:
    mutable Validator validator_;
    ValidationConfig cfg_;
    std::stop_token stop_;
    std::atomic<std::uint64_t>& count_;
};
}

PlanningResult solve_ompl(const Scene& scene,const PlannerConfig& cfg,std::stop_token stop) {
    PlanningResult out;const auto begin=std::chrono::steady_clock::now();
    auto finish=[&](){out.total_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();};
    if(!valid_pose(scene.start)){out.status=SolveStatus::InvalidStart;out.reason="invalid start pose";finish();return out;}
    if(!valid_pose(scene.goal)){out.status=SolveStatus::InvalidGoal;out.reason="invalid goal pose";finish();return out;}
    auto space=std::make_shared<ob::SE3StateSpace>();ob::RealVectorBounds bounds(3);
    for(int i=0;i<3;++i){bounds.setLow(i,scene.sampling.min[i]);bounds.setHigh(i,scene.sampling.max[i]);}space->setBounds(bounds);
    og::SimpleSetup setup(space);auto si=setup.getSpaceInformation();Validator state_validator(scene);std::atomic<std::uint64_t> checks{0};
    setup.setStateValidityChecker([&](const ob::State* s){return state_validator.state(from_state(s),cfg.validation).valid;});
    si->setMotionValidator(std::make_shared<SharedMotionValidator>(si,scene,cfg.validation,stop,checks));
    ob::ScopedState<ob::SE3StateSpace> start(space),goal(space);to_state(scene.start,start.get());to_state(scene.goal,goal.get());
    setup.setStartAndGoalStates(start,goal);auto planner=std::make_shared<og::RRTConnect>(si);
    planner->setRange(std::max(1e-9,cfg.extension));setup.setPlanner(planner);
    const auto search_begin=std::chrono::steady_clock::now();
    auto timed=[&](){return cfg.time_limit>0&&std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>=cfg.time_limit;};
    ob::PlannerTerminationCondition ptc([&](){return stop.stop_requested()||timed()||checks.load()>=cfg.max_queries;});
    auto status=setup.solve(ptc);out.search_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-search_begin).count();
    out.queries=state_validator.stats();
    if(status){auto& path=setup.getSolutionPath();std::vector<Pose> poses;poses.reserve(path.getStateCount());for(std::size_t i=0;i<path.getStateCount();++i)poses.push_back(from_state(path.getState(i)));
        out.path={std::move(poses),scene.hash,scene.revision,cfg.validation,"ompl-rrt-connect"};
        const auto v=std::chrono::steady_clock::now();out.verification=validate_path(scene,out.path,cfg.validation,stop);out.validation_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-v).count();
        if(out.verification.status==CheckStatus::Passed){out.status=SolveStatus::Solved;out.reason="OMPL RRT-Connect solution independently validated; OMPL uses sqrt(translation^2 + angle^2) for nearest-neighbour distance, while PassageLab uses translation + rotation_weight * angle.";}
        else {out.status=out.verification.status==CheckStatus::Cancelled?SolveStatus::Cancelled:SolveStatus::ValidationIncomplete;out.reason=out.verification.reason;}
    } else {out.status=stop.stop_requested()?SolveStatus::Cancelled:SolveStatus::BudgetExhausted;out.reason=stop.stop_requested()?"cancelled":"OMPL termination condition reached without a solution";}
    finish();return out;
}
}
