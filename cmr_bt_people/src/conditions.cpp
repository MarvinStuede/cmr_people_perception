#include <cmr_bt_people/conditions.h>

BT_REGISTER_NODES(factory)
{
  cmr_bt::registerPeopleConditions(factory);
}

BT::NodeStatus cmr_bt::PersonClose::tick()
{
  if(!_persons || _persons->tracks.empty()) return BT::NodeStatus::FAILURE;

  //Check all tracked persons
  for(const auto & person : _persons->tracks){
    auto pose = person.pose.pose;
    bool transformed = cmr_os::util_tf::transformPose(pose, _persons->header.frame_id, _frame, _listener);

    if(!transformed) return BT::NodeStatus::FAILURE;

    //Get distance from blackboard. If person is closer than this distance it is views as "close"
    double distance = cmr_bt::checkInput(getInput<double>("distance"));

    //Calc distance between person and center of robot
    if(cmr_os::util_tf::euclidianDistBetweenPoses(pose, _pose_base) < distance)
      return BT::NodeStatus::SUCCESS;
  }
  //If no person is closer than distance, fail
  return BT::NodeStatus::FAILURE;
}
