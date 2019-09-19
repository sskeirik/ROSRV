#ifndef RVCPP_ROBIXCTRL_H
#define RVCPP_ROBIXCTRL_H

using namespace std;


namespace provided {

namespace modelplex_generated {

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

typedef struct verdict { int id; long double val; } verdict;


/* Computes distance to safety boundary on prior and current state (>=0 is safe, <0 is unsafe) */
verdict boundaryDist(state pre, state curr, const parameters* const params) {
  verdict result = { .id=(((curr.yg > 0.0L) && (((fabsl(curr.k))*(params->eps) <= (100.0L)*(1.0L)) && ((((((curr.k)*((params->eps)*(params->eps)))-(((2.0L)*(100.0L))*(params->eps)))*((10.0L)*(10.0L)) < ((curr.k)*(((curr.xg)*(curr.xg))+((curr.yg)*(curr.yg))))-((((2.0L)*(curr.xg))*(100.0L))*(10.0L))) && (((curr.k)*(((curr.xg)*(curr.xg))+((curr.yg)*(curr.yg))))-((((2.0L)*(curr.xg))*(100.0L))*(10.0L)) < (((curr.k)*((params->eps)*(params->eps)))+(((2.0L)*(100.0L))*(params->eps)))*((10.0L)*(10.0L)))) && ((0.0L <= curr.vl) && ((curr.vl < curr.vh) && (((params->A)*(params->T) <= (10.0L)*((curr.vh)-(curr.vl))) && ((params->B)*(params->T) <= (10.0L)*((curr.vh)-(curr.vl))))))))) && ((((-(params->B) <= curr.a) && (curr.a <= params->A)) && ((((10.0L)*(pre.v))+((curr.a)*(params->T)) >= 0.0L) && ((((pre.v <= curr.vh) && (((10.0L)*(pre.v))+((curr.a)*(params->T)) <= (10.0L)*(curr.vh))) || (((((((1.0L)*(100.0L))*((1.0L)*(100.0L)))+(((((2.0L)*(params->eps))*(fabsl(curr.k)))*(1.0L))*(100.0L)))+(((params->eps)*(params->eps))*((curr.k)*(curr.k))))*(((params->B)*(((((2.0L)*(pre.v))*(params->T))*(10.0L))+((curr.a)*((params->T)*(params->T)))))+(((((pre.v)*(10.0L))+((curr.a)*(params->T)))*(((pre.v)*(10.0L))+((curr.a)*(params->T))))-(((10.0L)*(curr.vh))*((10.0L)*(curr.vh))))) <= ((((2.0L)*(params->B))*((fabsl(curr.xg))-((10.0L)*(params->eps))))*((100.0L)*(100.0L)))*((10.0L)*(10.0L))) || ((((((1.0L)*(100.0L))*((1.0L)*(100.0L)))+(((((2.0L)*(params->eps))*(fabsl(curr.k)))*(1.0L))*(100.0L)))+(((params->eps)*(params->eps))*((curr.k)*(curr.k))))*(((params->B)*(((((2.0L)*(pre.v))*(params->T))*(10.0L))+((curr.a)*((params->T)*(params->T)))))+(((((pre.v)*(10.0L))+((curr.a)*(params->T)))*(((pre.v)*(10.0L))+((curr.a)*(params->T))))-(((10.0L)*(curr.vh))*((10.0L)*(curr.vh))))) <= ((((2.0L)*(params->B))*((curr.yg)-((10.0L)*(params->eps))))*((100.0L)*(100.0L)))*((10.0L)*(10.0L))))) && (((curr.vl <= pre.v) && (((10.0L)*(pre.v))+((curr.a)*(params->T)) >= (10.0L)*(curr.vl))) || (((((((1.0L)*(100.0L))*((1.0L)*(100.0L)))+(((((2.0L)*(params->eps))*(fabsl(curr.k)))*(1.0L))*(100.0L)))+(((params->eps)*(params->eps))*((curr.k)*(curr.k))))*(((params->A)*(((((2.0L)*(pre.v))*(params->T))*(10.0L))+((curr.a)*((params->T)*(params->T)))))+((((curr.vl)*(10.0L))*((curr.vl)*(10.0L)))-((((pre.v)*(10.0L))+((curr.a)*(params->T)))*(((pre.v)*(10.0L))+((curr.a)*(params->T)))))) <= ((((2.0L)*(params->A))*((fabsl(curr.xg))-((10.0L)*(params->eps))))*((100.0L)*(100.0L)))*((10.0L)*(10.0L))) || ((((((1.0L)*(100.0L))*((1.0L)*(100.0L)))+(((((2.0L)*(params->eps))*(fabsl(curr.k)))*(1.0L))*(100.0L)))+(((params->eps)*(params->eps))*((curr.k)*(curr.k))))*(((params->A)*(((((2.0L)*(pre.v))*(params->T))*(10.0L))+((curr.a)*((params->T)*(params->T)))))+((((curr.vl)*(10.0L))*((curr.vl)*(10.0L)))-((((pre.v)*(10.0L))+((curr.a)*(params->T)))*(((pre.v)*(10.0L))+((curr.a)*(params->T)))))) <= ((((2.0L)*(params->A))*((curr.yg)-((10.0L)*(params->eps))))*((100.0L)*(100.0L)))*((10.0L)*(10.0L)))))))) && (((pre.v >= 0.0L) && (0.0L <= params->T)) && ((curr.a == curr.a) && ((curr.k == curr.k) && ((curr.t == 0.0L) && ((curr.v == pre.v) && ((curr.vh == curr.vh) && ((curr.vl == curr.vl) && ((curr.xg == curr.xg) && (curr.yg == curr.yg)))))))))) ? 1 : -1), .val=(((curr.yg > 0.0L) && (((fabsl(curr.k))*(params->eps) <= (100.0L)*(1.0L)) && ((((((curr.k)*((params->eps)*(params->eps)))-(((2.0L)*(100.0L))*(params->eps)))*((10.0L)*(10.0L)) < ((curr.k)*(((curr.xg)*(curr.xg))+((curr.yg)*(curr.yg))))-((((2.0L)*(curr.xg))*(100.0L))*(10.0L))) && (((curr.k)*(((curr.xg)*(curr.xg))+((curr.yg)*(curr.yg))))-((((2.0L)*(curr.xg))*(100.0L))*(10.0L)) < (((curr.k)*((params->eps)*(params->eps)))+(((2.0L)*(100.0L))*(params->eps)))*((10.0L)*(10.0L)))) && ((0.0L <= curr.vl) && ((curr.vl < curr.vh) && (((params->A)*(params->T) <= (10.0L)*((curr.vh)-(curr.vl))) && ((params->B)*(params->T) <= (10.0L)*((curr.vh)-(curr.vl))))))))) && ((((-(params->B) <= curr.a) && (curr.a <= params->A)) && ((((10.0L)*(pre.v))+((curr.a)*(params->T)) >= 0.0L) && ((((pre.v <= curr.vh) && (((10.0L)*(pre.v))+((curr.a)*(params->T)) <= (10.0L)*(curr.vh))) || (((((((1.0L)*(100.0L))*((1.0L)*(100.0L)))+(((((2.0L)*(params->eps))*(fabsl(curr.k)))*(1.0L))*(100.0L)))+(((params->eps)*(params->eps))*((curr.k)*(curr.k))))*(((params->B)*(((((2.0L)*(pre.v))*(params->T))*(10.0L))+((curr.a)*((params->T)*(params->T)))))+(((((pre.v)*(10.0L))+((curr.a)*(params->T)))*(((pre.v)*(10.0L))+((curr.a)*(params->T))))-(((10.0L)*(curr.vh))*((10.0L)*(curr.vh))))) <= ((((2.0L)*(params->B))*((fabsl(curr.xg))-((10.0L)*(params->eps))))*((100.0L)*(100.0L)))*((10.0L)*(10.0L))) || ((((((1.0L)*(100.0L))*((1.0L)*(100.0L)))+(((((2.0L)*(params->eps))*(fabsl(curr.k)))*(1.0L))*(100.0L)))+(((params->eps)*(params->eps))*((curr.k)*(curr.k))))*(((params->B)*(((((2.0L)*(pre.v))*(params->T))*(10.0L))+((curr.a)*((params->T)*(params->T)))))+(((((pre.v)*(10.0L))+((curr.a)*(params->T)))*(((pre.v)*(10.0L))+((curr.a)*(params->T))))-(((10.0L)*(curr.vh))*((10.0L)*(curr.vh))))) <= ((((2.0L)*(params->B))*((curr.yg)-((10.0L)*(params->eps))))*((100.0L)*(100.0L)))*((10.0L)*(10.0L))))) && (((curr.vl <= pre.v) && (((10.0L)*(pre.v))+((curr.a)*(params->T)) >= (10.0L)*(curr.vl))) || (((((((1.0L)*(100.0L))*((1.0L)*(100.0L)))+(((((2.0L)*(params->eps))*(fabsl(curr.k)))*(1.0L))*(100.0L)))+(((params->eps)*(params->eps))*((curr.k)*(curr.k))))*(((params->A)*(((((2.0L)*(pre.v))*(params->T))*(10.0L))+((curr.a)*((params->T)*(params->T)))))+((((curr.vl)*(10.0L))*((curr.vl)*(10.0L)))-((((pre.v)*(10.0L))+((curr.a)*(params->T)))*(((pre.v)*(10.0L))+((curr.a)*(params->T)))))) <= ((((2.0L)*(params->A))*((fabsl(curr.xg))-((10.0L)*(params->eps))))*((100.0L)*(100.0L)))*((10.0L)*(10.0L))) || ((((((1.0L)*(100.0L))*((1.0L)*(100.0L)))+(((((2.0L)*(params->eps))*(fabsl(curr.k)))*(1.0L))*(100.0L)))+(((params->eps)*(params->eps))*((curr.k)*(curr.k))))*(((params->A)*(((((2.0L)*(pre.v))*(params->T))*(10.0L))+((curr.a)*((params->T)*(params->T)))))+((((curr.vl)*(10.0L))*((curr.vl)*(10.0L)))-((((pre.v)*(10.0L))+((curr.a)*(params->T)))*(((pre.v)*(10.0L))+((curr.a)*(params->T)))))) <= ((((2.0L)*(params->A))*((curr.yg)-((10.0L)*(params->eps))))*((100.0L)*(100.0L)))*((10.0L)*(10.0L)))))))) && (((pre.v >= 0.0L) && (0.0L <= params->T)) && ((curr.a == curr.a) && ((curr.k == curr.k) && ((curr.t == 0.0L) && ((curr.v == pre.v) && ((curr.vh == curr.vh) && ((curr.vl == curr.vl) && ((curr.xg == curr.xg) && (curr.yg == curr.yg)))))))))) ? 1.0L : -1.0L) }; return result;
}

/* Evaluates monitor condition in prior and current state */
bool monitorSatisfied(state pre, state curr, const parameters* const params) {
  return boundaryDist(pre,curr,params).val >= 0.0L;
}

}

struct MonitorState {
    modelplex_generated::parameters params;
    modelplex_generated::state prevState;
    modelplex_generated::state currState;
    bool initialized;

    std::map<std::string, bool> statusMap;
    std::vector<std::string> variables;

    MonitorState(std::vector<std::string> variablesToMonitor) {
	initialized = false;
	variables = variablesToMonitor;
	for(auto var : variables) {
	    statusMap[var] = false;
	}
    }

    void uninitialize_current_state() {
	for(auto var : variables) {
	    statusMap[var] = false;
	}
    }

    void checkViolation() {
	bool status = true;
	if(!initialized) {
	    for(const auto &pair : statusMap) {
		status = status && pair.second;
	    }
	    initialized = true;
	} else {

	    for(const auto &pair : statusMap) {
		status = status && pair.second;
	    }

	    if(status) {
		// offset the time between states
		currState.t = currState.t - prevState.t;
		prevState.t = 0.0;

		if(!modelplex_generated::monitorSatisfied(prevState, currState, &params))
			ROS_WARN("[ROSRV-WARNING]  Controller Monitor Failure --- \n");

		prevState = currState;
		uninitialize_current_state();
	    }
	}
    }


    void updateV(long double v, long double t) {
	statusMap["v"] = true;
	if(!initialized) {
	    prevState.v = v;
	    prevState.t = t * 10;
	} else {
	    currState.v = v;
	    currState.t = (t * 10.0);
	    currState.a = (currState.v - prevState.v) / (currState.t) - (prevState.t);
	}
	checkViolation();
    }

    void updateK(long double k) {
	statusMap["k"] = true;
	if(!initialized) {
	    prevState.k = k * -100;
	} else {
	    currState.k = k * -100;
	}
	checkViolation();
    }

    void updateXgYg(long double xg, long double yg) {
	statusMap["xg"] = true;
	statusMap["yg"] = true;
	if(!initialized) {
	    prevState.xg = xg;
	    prevState.yg = yg;
	} else {
	    currState.xg = xg;
	    currState.yg = yg;
	}
	checkViolation();
    }
};

}
#endif
