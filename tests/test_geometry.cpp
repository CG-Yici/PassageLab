#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "passagelab/core.hpp"
#include <limits>
#include <numbers>

using namespace pl;
namespace {
Part box(std::string id,Vec3 half,Pose p={}) { Part x; x.id=std::move(id);x.half=half;x.pose=p;return x; }
Scene base_scene() { Scene s; s.body={box("body",Vec3(.1,.1,.1))};s.sampling.min=Vec3(-5,-5,-5);s.sampling.max=Vec3(5,5,5);s.physical.min=Vec3(-10,-10,-10);s.physical.max=Vec3(10,10,10);return s; }
}
TEST_CASE("rigid transforms and shortest arc are stable") {
    Pose a{Vec3(1,2,3),Quat(Eigen::AngleAxisd(.6,Vec3::UnitZ()))},b{Vec3(0,1,0),Quat::Identity()};
    auto p=compose(a,b),back=compose(inverse(a),p); REQUIRE((back.t-b.t).norm()<1e-10);
    REQUIRE(rotation_distance(a.q,-a.q)<1e-8);
    Pose neg=a;neg.q.coeffs()*=-1;auto mid=interpolate(a,neg,.5);REQUIRE(rotation_distance(mid.q,a.q)<1e-8);
}
TEST_CASE("sphere distances and OBB SAT classify separation contact and containment") {
    Part s; s.id="sphere";s.shape=Shape::Sphere;s.radius=.5;
    Part t=s;t.id="other";t.pose.t=Vec3(2,0,0);
    auto q=query_pair(s,Pose{},t);REQUIRE(q.relation==Relation::Separated);REQUIRE(q.distance);CHECK(*q.distance==Catch::Approx(1.0));
    Part a=box("a",Vec3(1,1,1)),b=box("b",Vec3(1,1,1));b.pose.t=Vec3(2,0,0);
    CHECK(query_pair(a,Pose{},b).relation==Relation::Touching);
    b.pose.t=Vec3(.2,0,0);CHECK(query_pair(a,Pose{},b).relation==Relation::Overlapping);
    b.half=Vec3(.1,.1,.1);b.pose.t=Vec3::Zero();CHECK(query_pair(a,Pose{},b).relation==Relation::Overlapping);
}
TEST_CASE("validator rejects invalid state and physical boundary overflow") {
    auto s=base_scene();Validator v(s);ValidationConfig c;
    Pose p;p.t.x()=std::numeric_limits<double>::quiet_NaN();CHECK_FALSE(v.state(p,c).valid);
    p=Pose{};p.t.x()=9.95;CHECK(v.state(p,c).reason=="physical bounds");
}
TEST_CASE("discrete motion detects thin obstacle between clear endpoints") {
    auto s=base_scene(); Part wall=box("wall",Vec3(.015,.5,.5));s.obstacles.push_back(wall);Validator v(s);
    ValidationConfig c;c.translation_step=.01;c.surface_step=.01;c.rotation_step=.05;
    Pose a,b;a.t.x()=-1;b.t.x()=1;CHECK(v.motion(a,b,c).status==CheckStatus::Failed);
}
TEST_CASE("rotation samples the swept footprint") {
    auto s=base_scene();s.body={box("arm",Vec3(1,.04,.04))};Part wall=box("wall",Vec3(.06,.06,.2));wall.pose.t=Vec3(.72,.72,0);s.obstacles={wall};Validator v(s);
    ValidationConfig c;c.rotation_step=.01;c.surface_step=.01;c.translation_step=.1;
    Pose a,b;b.q=Quat(Eigen::AngleAxisd(std::numbers::pi/2,Vec3::UnitZ()));
    CHECK(v.motion(a,b,c).status==CheckStatus::Failed);
}
TEST_CASE("BVH and brute force agree including positive margins") {
    auto s=base_scene();for(int i=0;i<20;i++){Pose p;p.t=Vec3(1.0+i*.5,0,0);s.obstacles.push_back(box(std::to_string(i),Vec3(.1,.1,.1),p));}
    Validator v(s);ValidationConfig a;a.margin=.25;a.use_bvh=true;auto x=v.state(Pose{},a,true);a.use_bvh=false;auto y=v.state(Pose{},a,true);
    CHECK(x.valid==y.valid);CHECK(x.query.distance.has_value()==y.query.distance.has_value());if(x.query.distance&&y.query.distance)CHECK(*x.query.distance==Catch::Approx(*y.query.distance));
}
TEST_CASE("motion reports sample exhaustion and cancellation") {
    auto s=base_scene();Validator v(s);Pose a,b;b.t.x()=10;ValidationConfig c;c.translation_step=1;c.surface_step=1;c.max_samples=2;
    CHECK(v.motion(a,b,c).status==CheckStatus::Incomplete);
    c.max_samples=100;std::stop_source stop;stop.request_stop();CHECK(v.motion(a,b,c,stop.get_token()).status==CheckStatus::Cancelled);
}
