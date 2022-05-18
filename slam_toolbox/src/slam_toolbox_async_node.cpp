/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Steve Macenski
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include <chrono>
#include "slam_toolbox/slam_toolbox_async.hpp"

class TicToc {
private:
    std::chrono::_V2::system_clock::time_point start;
public:
    void tic() {
        start = std::chrono::system_clock::now();
    }

    // return in milliseconds
    double toc() const {
        auto end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        return static_cast<double>(duration.count()) / 1000.0;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_toolbox");
  ros::NodeHandle nh("~");
  ros::spinOnce();
  std::string traj_output_path = nh.param<std::string>("traj_output_path", "/home/stn/slam/trajectories/test/output");

  int stack_size;
  if (nh.getParam("stack_size_to_use", stack_size))
  {
    ROS_INFO("Node using stack size %i", (int)stack_size);
    const rlim_t max_stack_size = stack_size;
    struct rlimit stack_limit;
    getrlimit(RLIMIT_STACK, &stack_limit);
    if (stack_limit.rlim_cur < stack_size)
    {
      stack_limit.rlim_cur = stack_size;
    }
    setrlimit(RLIMIT_STACK, &stack_limit);
  }

  slam_toolbox::AsynchronousSlamToolbox sst(nh);

  double idle_interval = -2000.0;
  TicToc idle_timer;
  idle_timer.tic();
  ros::Rate rate(1000);
  while (ros::ok()) {
    ros::spinOnce();
    if (sst.idle_flag == false) {
      idle_interval = 0.0;
      sst.idle_flag = true;
    }
    else {
      rate.sleep();
      idle_interval += idle_timer.toc();
    }
    if (idle_interval > 3000.) {
      printf("Being idle for a very long time, exiting...\n");
      break;
    }
    idle_timer.tic();
  }
  slam_toolbox_msgs::SerializePoseGraph::Request req;
  slam_toolbox_msgs::SerializePoseGraph::Response res;
  req.filename = traj_output_path;
  sst.serializePoseGraphCallback(req, res);
  ros::shutdown();
  return 0;
}
