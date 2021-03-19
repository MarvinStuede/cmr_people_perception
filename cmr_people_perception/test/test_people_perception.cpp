#include "ros/subscriber.h"
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <set>

class PeopleTrackingTest: public ::testing::Test
{
public:
    PeopleTrackingTest(): spinner(0) {};
    ~PeopleTrackingTest() {};

    ros::NodeHandle* node;
    ros::AsyncSpinner* spinner;
    ros::Subscriber sub_tracked_pers_; 
    ros::Subscriber sub_detect_pers_; 
    unsigned int people_count = 0;
    std::set<int> people_ids;
    std::set<std::string> people_detectors;
    void SetUp() override
    {
	people_ids.clear();
        ::testing::Test::SetUp();
        this->node = new ros::NodeHandle("~");
        this->spinner = new ros::AsyncSpinner(0);
        this->spinner->start();
	this->sub_tracked_pers_ = this->node->subscribe("/spencer/perception/tracked_persons", 10, &PeopleTrackingTest::subPeopleCb, this);
	this->sub_detect_pers_ = this->node->subscribe("/spencer/perception/detected_persons", 10, &PeopleTrackingTest::subPeopleDectCb, this);
    };

    void TearDown() override
    {
        ros::shutdown();
        delete this->spinner;
        delete this->node;
        ::testing::Test::TearDown();
    }

    void subPeopleCb(const spencer_tracking_msgs::TrackedPersons &people){
    	people_count += people.tracks.size();
	for(const auto & person : people.tracks){
		people_ids.insert(person.track_id);
	}
    }



    void subPeopleDectCb(const spencer_tracking_msgs::DetectedPersons &people){
	for(const auto & person : people.detections){
		people_detectors.insert(person.modality);
	}
    }
};

TEST_F(PeopleTrackingTest, test_ok)
{
    ros::Duration(30).sleep();
    ROS_INFO_STREAM("Size of tracks: "<<people_count);
    ROS_INFO_STREAM("Size of ids: "<<people_ids.size());
    
    for(auto mod : people_detectors){
     ROS_INFO_STREAM("Detector: "<<mod);
    }
    
   // EXPECT_LE(people_ids.size(), 3); //Assert that less than this number of unique tracks was found    
    //Check if RGBD and 3D Laser detector were detecting
    EXPECT_TRUE(people_detectors.find("rgbd") != people_detectors.end());
    EXPECT_TRUE(people_detectors.find("laser3d") != people_detectors.end());
    EXPECT_TRUE(people_detectors.find("laser3d,rgbd") != people_detectors.end() || people_detectors.find("rgbd,laser3d") != people_detectors.end());
    EXPECT_GT(people_count, 100); //Assert that at least this number of people tracks was detected
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ppercept_test_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged(); // To show debug output in the tests
    }
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
