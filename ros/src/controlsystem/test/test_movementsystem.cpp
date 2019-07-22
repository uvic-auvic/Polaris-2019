//
// Created by avlec on 22/07/19.
//

// Responsible for testing the ../src/movementsystem.cpp

#include <ros/ros.h>
#include "movementsystem.hpp"

#include <gtest/gtest.h>

TEST(TestSuite, testCase1)
{
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_movementsystem");
	ros::NodeHandle nh;
	return RUN_ALL_TESTS();
}
