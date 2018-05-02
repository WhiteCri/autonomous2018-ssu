/*
 * first developer : Choi Kyu-Jin, Soongsil University. Thank you for your help!
 * edited by Park TaeWook.
 *
*/
#pragma once
#include <iostream>
#include <ros/ros.h>
#include <deque>

#define MAX_STATES 100
#define MAX_CONDCOUNT 10000
#define COND_RATE 0.8 //if one condition decide to use timed_contidion, it yeilds true 
//when the rate of true related to the total_size is larger than COND_RATE

using namespace std;

class HybridAutomata;
class Condition
{
public:
  virtual bool check(HybridAutomata *HA) = 0;
  virtual ~Condition() {}
  virtual std::string toString() const = 0;
protected:
};
//added by TaeWook
class TimedCondition : public Condition
{
public:
  TimedCondition(unsigned int condCount) : condCount_(condCount), condDeq(condCount)
  { 
    if(condCount_ == 0) condCount_ = 1;
    if(condCount_ >= MAX_CONDCOUNT) throw std::runtime_error("too big condCount!");
  }
  virtual bool check(HybridAutomata *HA)
  {
    //processing condition
    condDeq.pop_front();
    condDeq.push_back(timedCheck(HA));

    //is rate of true is bigger than COND_RATE?
    size_t trueCnt = 0;
    for(auto b : condDeq)
      if(b) trueCnt++;
    double rate = static_cast<double>(trueCnt) / condDeq.size();
    if (rate >= COND_RATE) return true;
    else return false;
  }
  virtual bool timedCheck(HybridAutomata *HA) = 0;
  virtual ~TimedCondition() {}
private:
  int condCount_;
  std::deque<bool> condDeq;
};
//added by TawWook End
class HybridAutomata
{
private:
  class State
  {
  public:
    unsigned int stateId;
    void (*aDo)();
    State(unsigned int id, void (*ah)())
    {
      stateId = id;
      aDo = ah;
    }
  };
  unsigned int initState, exitState;
  unsigned int nStates;
  bool stateMachine[MAX_STATES][MAX_STATES];
  State *states[MAX_STATES];
  Condition *conditions[MAX_STATES][MAX_STATES];
  void initStateMachineArr();
  void initConditionsArr();
  void initStateArr();
  bool checkStateMachine(unsigned int pre, unsigned int post);
  int checkConditions();

public:
  unsigned int curState;
  void setState(unsigned int id, void (*ah)());                                     // user sets states
  void setCondition(unsigned int preState, Condition* cDo, unsigned int postState); //user sets conditions
  void operate(); // check conditions and if condition has satisfied move to another state
  HybridAutomata(const unsigned int init, const unsigned int exit);
  HybridAutomata();
  ~HybridAutomata();
  //bool checkConditions(unsigned int post);
};