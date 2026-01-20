#pragma once


#include <ros/ros.h>
#include <vector>
#include <string>

namespace amrl {

template <typename TStateMachine>
class StateMachineBase 
{
public:

  StateMachineBase(const std::string &sm_name) :
    _sm_name(sm_name)
  {

  }


  typename TStateMachine::State state(void) const
  {
    return _state;
  }

  void raise_event(const typename TStateMachine::Event) 
  {
    ROS_INFO("[%s] Event Raised: %s", _sm_name.c_str(), kEventName.at(event).c_str());
    _events.push_back(event);
  }

  bool event_is_raised(const Event event) const
  {

  }

  void change_state(const State new_state)
  {

  }

private:

  typename TStateMachine::State _state;                //< Current state of the VLM node
  std::vector<typename TStateMachine::Event> _events;  //< State-Machine events raised for a cycle
  std::string _sm_name;                                //< Name of the state-machine


};

}