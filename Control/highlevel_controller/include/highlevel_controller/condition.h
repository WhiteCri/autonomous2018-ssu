#pragma once
#include "HybridAutomata.h"

class Init_to_toward_goal : public Condition{
public:
    virtual bool check(HybridAutomata *HA);
};

/* crosswalk class */
class Toward_goal_to_process_crosswalk: public TimedCondition{
public:
    Toward_goal_to_process_crosswalk(unsigned int condCount) : TimedCondition(condCount) {}
    virtual bool timedCheck(HybridAutomata *HA);
};

class Process_crosswalk_to_toward_goal : public Condition{
public:
    virtual bool check(HybridAutomata *HA);
};

/* movingobj class */
class Toward_goal_to_process_movingobj: public TimedCondition{
public:
    Toward_goal_to_process_movingobj(unsigned int condCount) : TimedCondition(condCount) {}
    virtual bool timedCheck(HybridAutomata *HA);
};

class Process_movingobj_to_toward_goal : public Condition{
public:
    virtual bool check(HybridAutomata *HA);
};

/* parking class */
class Toward_goal_to_process_parking: public TimedCondition{
public:
    Toward_goal_to_process_parking(unsigned int condCount) : TimedCondition(condCount) {}
    virtual bool timedCheck(HybridAutomata *HA);
};

class Process_parking_to_toward_goal : public Condition{
public:
    virtual bool check(HybridAutomata *HA);
};

/* recovery */
class Toward_goal_to_process_recovery: public TimedCondition{
public:
    Toward_goal_to_process_recovery(unsigned int condCount) : TimedCondition(condCount) {}
    virtual bool timedCheck(HybridAutomata *HA);
};

class Process_recovery_to_toward_goal : public Condition{
public:
    virtual bool check(HybridAutomata *HA);
};

/* Done */
class Toward_goal_to_done : public Condition{
public:
    virtual bool check(HybridAutomata *HA);
};