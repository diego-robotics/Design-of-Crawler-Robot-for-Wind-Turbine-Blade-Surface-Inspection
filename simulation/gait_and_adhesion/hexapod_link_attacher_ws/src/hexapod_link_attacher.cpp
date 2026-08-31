// hexapod_link_attacher.cpp
//
// A minimal Gazebo Classic WORLD plugin providing the same AttachLink /
// DetachLink service interface as IFRA_LinkAttacher, but with ONE
// deliberate difference: a <namespace> SDF parameter, so that loading six
// separate instances (one per leg, each with its own namespace) gives six
// fully independent pairs of services and six fully independent internal
// "currently attached joint" states -- instead of all six legs sharing the
// single global /ATTACHLINK, /DETACHLINK that IFRA_LinkAttacher hardcodes,
// which is what caused "Both links have already been attached" for every
// leg after the first, confirmed by extensive direct testing (unique
// links, unique models, swapped model1/model2 order all failed
// identically against the shared global service).
//
// Message types are unchanged (linkattacher_msgs/srv/AttachLink,
// linkattacher_msgs/srv/DetachLink) specifically so existing client code
// written against IFRA_LinkAttacher's interface (request/response fields:
// model1_name, link1_name, model2_name, link2_name, success, message)
// keeps working without changes beyond the service name itself.

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace gazebo
{

class HexapodLinkAttacher : public WorldPlugin
{
public:
  HexapodLinkAttacher() : WorldPlugin() {}

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    world_ = _world;

    std::string ns;
    if (_sdf->HasElement("namespace")) {
      ns = _sdf->Get<std::string>("namespace");
    }
    instance_name_ = ns.empty() ? std::string("linkattacher") : ns;

    // gazebo_ros::Node::Get(_sdf) is the standard way official gazebo_ros
    // plugins obtain a ROS 2 node from plugin SDF -- it also picks up any
    // <ros><namespace> block if present, but we read our own <namespace>
    // tag directly above instead, to keep the SDF for this plugin simple
    // and match what's used to build the service names below explicitly,
    // rather than relying on remapping.
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    std::string attach_srv_name = ns.empty() ? "ATTACHLINK" : (ns + "/ATTACHLINK");
    std::string detach_srv_name = ns.empty() ? "DETACHLINK" : (ns + "/DETACHLINK");

    attach_srv_ = ros_node_->create_service<linkattacher_msgs::srv::AttachLink>(
      attach_srv_name,
      std::bind(
        &HexapodLinkAttacher::OnAttach, this,
        std::placeholders::_1, std::placeholders::_2));

    detach_srv_ = ros_node_->create_service<linkattacher_msgs::srv::DetachLink>(
      detach_srv_name,
      std::bind(
        &HexapodLinkAttacher::OnDetach, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "HexapodLinkAttacher [%s] ready: %s, %s",
      instance_name_.c_str(), attach_srv_name.c_str(), detach_srv_name.c_str());
  }

private:
  void OnAttach(
    const std::shared_ptr<linkattacher_msgs::srv::AttachLink::Request> req,
    std::shared_ptr<linkattacher_msgs::srv::AttachLink::Response> res)
  {
    physics::ModelPtr model1 = world_->ModelByName(req->model1_name);
    physics::ModelPtr model2 = world_->ModelByName(req->model2_name);
    if (!model1 || !model2) {
      res->success = false;
      res->message = "model not found: " + req->model1_name + " or " + req->model2_name;
      RCLCPP_WARN(ros_node_->get_logger(), "[%s] attach failed: %s",
        instance_name_.c_str(), res->message.c_str());
      return;
    }

    physics::LinkPtr link1 = model1->GetLink(req->link1_name);
    physics::LinkPtr link2 = model2->GetLink(req->link2_name);
    if (!link1 || !link2) {
      res->success = false;
      res->message = "link not found: " + req->link1_name + " or " + req->link2_name;
      RCLCPP_WARN(ros_node_->get_logger(), "[%s] attach failed: %s",
        instance_name_.c_str(), res->message.c_str());
      return;
    }

    // This instance's own joint only -- if this specific leg's plugin
    // instance already holds an active attachment (e.g. attach called
    // again without an intervening detach), replace it cleanly rather
    // than erroring or leaking the old joint. This does NOT touch any
    // other instance's joint, which is the entire point.
    ReleaseJoint();

    // Fixed, reused name per instance (not per attempt) -- the earlier
    // incrementing-counter version was a workaround for a suspected name
    // collision from Detach() alone not fully unregistering the joint.
    // Now that ReleaseJoint() also calls RemoveJoint() (a more thorough
    // teardown), reusing one name should be safe, and avoids creating an
    // ever-growing number of uniquely-named joint objects across repeated
    // attach/detach cycles -- a plausible contributor to Gazebo freezing
    // after several gait cycles (confirmed physics-thread-level stall,
    // not a client-side hang, and confirmed unrelated to the leveling
    // controller by testing with it disabled).
    std::string joint_name = instance_name_ + "_fixed_joint";

    // CreateJoint/GetJoint/Init() still deliberately run with no lock
    // held around the Gazebo API calls themselves -- that's what caused
    // the earlier deadlock (see file header). What's now protected
    // separately is only the assignment of the resulting pointer to our
    // own member variables, matching ReleaseJoint()'s locking -- see its
    // docstring for why that's needed (a different bug: concurrent
    // OnAttach/OnDetach calls racing on these members directly, which is
    // what crashed gzserver with a null shared_ptr dereference).
    model1->CreateJoint(joint_name, "fixed", link1, link2);
    physics::JointPtr new_joint = model1->GetJoint(joint_name);
    if (new_joint) {
      new_joint->Init();
    }
    if (!new_joint) {
      res->success = false;
      res->message = "joint creation failed";
      RCLCPP_ERROR(ros_node_->get_logger(), "[%s] attach failed: joint '%s' not found on model after CreateJoint",
        instance_name_.c_str(), joint_name.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(member_mutex_);
      fixed_joint_ = new_joint;
      attached_model_ = model1;
      attached_joint_name_ = joint_name;
    }

    res->success = true;
    res->message = "attached";
    RCLCPP_INFO(ros_node_->get_logger(), "[%s] attached %s::%s to %s::%s",
      instance_name_.c_str(),
      req->model1_name.c_str(), req->link1_name.c_str(),
      req->model2_name.c_str(), req->link2_name.c_str());
  }

  void OnDetach(
    const std::shared_ptr<linkattacher_msgs::srv::DetachLink::Request> req,
    std::shared_ptr<linkattacher_msgs::srv::DetachLink::Response> res)
  {
    (void)req;
    {
      std::lock_guard<std::mutex> lock(member_mutex_);
      if (!fixed_joint_) {
        res->success = false;
        res->message = "nothing currently attached on this instance";
        return;
      }
    }
    ReleaseJoint();
    res->success = true;
    res->message = "detached";
    RCLCPP_INFO(ros_node_->get_logger(), "[%s] detached", instance_name_.c_str());
  }

  // Fully releases the current joint, if any: both severs the C++
  // parent/child references (Detach()) AND explicitly removes the joint
  // from the model (RemoveJoint()). Detach() alone was confirmed
  // insufficient by testing -- the service reported success and the C++
  // joint object was released, but legs stayed physically stuck to the
  // ground during actual gait walking, consistent with the underlying
  // physics-engine constraint not actually being destroyed by Detach()
  // alone. RemoveJoint() is what's documented to fully tear down a
  // joint's presence in the physics engine, not just the wrapper object.
  void ReleaseJoint()
  {
    // Capture-and-clear under a narrow lock, then act on local copies
    // AFTER releasing it. This is deliberately NOT the same lock removed
    // earlier (that one wrapped calls into Gazebo's own API and caused a
    // lock-ordering deadlock against Gazebo's physics thread, confirmed
    // by a direct gdb backtrace). This lock protects only our own member
    // variables and is never held while calling into Gazebo at all.
    //
    // It exists because of a SEPARATE bug, also confirmed directly: the
    // server runs a MultiThreadedExecutor, so OnAttach/OnDetach calls for
    // the same leg can run concurrently on different threads. Without
    // this lock, one thread could check "fixed_joint_ is non-null" and
    // then have a second, concurrent call reset it to null before the
    // first thread actually dereferences it -- a classic check-then-use
    // race, and the exact crash observed: "boost::shared_ptr<Joint>::
    // operator->(): Assertion `px != 0' failed", gzserver terminating
    // with SIGABRT. Capturing everything atomically under the lock and
    // clearing the members immediately closes that window: a second
    // concurrent call sees the already-cleared state and safely no-ops.
    physics::JointPtr local_joint;
    physics::ModelPtr local_model;
    std::string local_joint_name;
    {
      std::lock_guard<std::mutex> lock(member_mutex_);
      if (!fixed_joint_) {
        return;
      }
      local_joint = fixed_joint_;
      local_model = attached_model_;
      local_joint_name = attached_joint_name_;
      fixed_joint_.reset();
      attached_model_.reset();
      attached_joint_name_.clear();
    }
    local_joint->Detach();
    if (local_model) {
      local_model->RemoveJoint(local_joint_name);
    }
  }

  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Service<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_srv_;
  rclcpp::Service<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_srv_;
  std::mutex member_mutex_;  // protects fixed_joint_/attached_model_/attached_joint_name_ only -- never held while calling into Gazebo's API
  physics::JointPtr fixed_joint_;
  physics::ModelPtr attached_model_;
  std::string attached_joint_name_;
  std::string instance_name_;
};

GZ_REGISTER_WORLD_PLUGIN(HexapodLinkAttacher)

}  // namespace gazebo
