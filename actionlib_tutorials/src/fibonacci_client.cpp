#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

// the effect is the same as the version of python，is aim to use argument of server
// exploit just like: actionlib_tutorials::FibonacciResultConstPtr& result
// format：[pkg_name]::[Action_Name][Three_Argument_Name]ConstPtr& three_argument_name
// must be upper with the first letter
// use state and result below the function of doneCb

using namespace std;

void doneCb(const actionlib::SimpleClientGoalState& state,
const actionlib_tutorials::FibonacciResultConstPtr& result)
  {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    for(int i=0; i<=20; i++){
    ROS_INFO("got result output = %d", result->sequence[i]);
    }
  }


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal, &doneCb);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}