#ifndef RVCPP_RVMONITORDL_H
#define RVCPP_RVMONITORDL_H


#include "std_msgs/Float32.h"
#include "marti_common_msgs/Float32Stamped.h"
#include "ros/ros.h"

void monitorCallback_flow_controller_input(const marti_common_msgs::Float32Stamped::ConstPtr& monitored_msg);
void monitorCallback_current_level(const std_msgs::Float32::ConstPtr& monitored_msg);

namespace modelplex_generated
{
    typedef struct parameters {
      long double ep;
      long double m;
    } parameters;

    typedef struct state {
      long double c;
      long double f;
      long double l;
    } state;

    typedef struct input input;

    typedef struct verdict { int id; long double val; } verdict;


    /* Computes distance to safety boundary on prior and current state (>=0 is safe, <0 is unsafe) */
    verdict boundaryDist(state pre, state curr, const parameters* const params) {
      if ((-1.0L) <= curr.f) {
    if (curr.f <= ((params->m)-(pre.l))/(params->ep)) {
    if (0.0L <= pre.l) {
    if (0.0L <= params->ep) {
    if (curr.l == pre.l) {
    if (curr.c == 0.0L) {
    verdict result = { .id=1, .val=((((0.0L)+(-(((-1.0L))-(curr.f))))+(-((curr.f)-(((params->m)-(pre.l))/(params->ep)))))+(-((0.0L)-(pre.l))))+(-((0.0L)-(params->ep))) }; return result;
    } else {
    verdict result = { .id=-1, .val=-1.0L }; return result;
    }
    } else {
    verdict result = { .id=-2, .val=-1.0L }; return result;
    }
    } else {
    verdict result = { .id=-3, .val=((-1.0L))+(-((0.0L)-(params->ep))) }; return result;
    }
    } else {
    verdict result = { .id=-4, .val=((-1.0L))+(-((0.0L)-(pre.l))) }; return result;
    }
    } else {
    verdict result = { .id=-5, .val=((-1.0L))+(-((curr.f)-(((params->m)-(pre.l))/(params->ep)))) }; return result;
    }
    } else {
    verdict result = { .id=-6, .val=((-1.0L))+(-(((-1.0L))-(curr.f))) }; return result;
    };
    }

    /* Evaluates monitor condition in prior and current state */
    bool monitorSatisfied(state pre, state curr, const parameters* const params) {
      return boundaryDist(pre,curr,params).val >= 0.0L;
    }
};

#endif
