/* *****************************************************************
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/
/**
 * @file   conditions.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   02/2021
*
* @brief  Conditions for Behvaior tree people perception conditions
*/
#pragma once

#include "ros/ros.h"
#include <behaviortree_cpp_v3/condition_node.h>
#include "cmr_bt_generic/general.h"
#include <tf/transform_listener.h>
#include "spencer_tracking_msgs/TrackedPersons.h"
#include "cmr_os/cmr_util_tf.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace cmr_bt {
class PersonClose : public BT::ConditionNode
{

public:
  PersonClose(const std::string& name, const BT::NodeConfiguration& config)
    : PersonClose::ConditionNode(name, config)
  {
    _node = std::make_unique<ros::NodeHandle>();
    _sub = _node->subscribe("/spencer/perception/tracked_persons", 10, &PersonClose::cbPersons, this);

    //This pose is the origin of the frame we check for.
    _pose_base.orientation.w = 1.0;
  }
  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<double>("distance")
    };
  }
  BT::NodeStatus tick() override;
private:
  std::string _frame = "base_link";
  std::unique_ptr<ros::NodeHandle> _node;
  tf::TransformListener _listener;
  ros::Subscriber _sub;
  spencer_tracking_msgs::TrackedPersonsPtr _persons;
  geometry_msgs::Pose _pose_base;

  void cbPersons(const spencer_tracking_msgs::TrackedPersonsPtr &persons){
    this->_persons = persons;
  }
};
inline void registerPeopleConditions(BT::BehaviorTreeFactory &factory){
  factory.registerNodeType<PersonClose>("PersonClose");
}
};
