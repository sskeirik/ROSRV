#ifndef RVCPP_ROBIXCTRL_H
#define RVCPP_ROBIXCTRL_H

using namespace std;


namespace provided {

typedef struct parameters {
    long double A;
    long double B;
    long double T;
    long double eps;
} parameters;

typedef struct state {
    long double a;
    long double k;
    long double t;
    long double v;
    long double vh;
    long double vl;
    long double xg;
    long double yg;
} state;


struct MonitorState {
    parameters params = { .A = 1.0 };
    state prevState;
    state currState;
    bool isInitialized = false;

    void updateV(long double v, long double t) {
	if(!isInitialized) {
	    prevState.v = v;
	    prevState.t = t;
	    ROS_WARN("Current time is %Lf", prevState.t);
	} else {
	    currState.v = v;
	    currState.t = t;
	}
    }
};

}
#endif
