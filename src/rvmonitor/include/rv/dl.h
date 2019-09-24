#ifndef RV_DL_H
#define RV_DL_H


namespace rv
{
namespace dl
{

template <typename S, typename P>
struct MonitorState
{
    P params;
    S prevState, currState;


    std::map<std::string, bool> prevStateMap;
    std::map<std::string, bool> currStateMap;

    std::vector<std::string> logicalVariables;

    bool prevStateExistsFlag;
    bool isInitializedFlag;


    void initialize(P params, std::vector<std::string> variables) {
	this->params = params;
	this->logicalVariables = variables;

	bool prevStateExistsFlag;
	bool isInitializedFlag;

	for(auto variable : variables) {
	    prevStateMap[variable] = false;
	    currStateMap[variable] = false;
	}
    }

    bool prevStateExists() {
	if(!prevStateExistsFlag) {
	    bool status = true;
	    for(auto variableStatus : prevStateMap) {
		status = status && variableStatus.second;
	    }
	    if(status)
		prevStateExistsFlag = true;
	    return status;
	}
	return true;
    }

    bool isInitialized() {
	if(!isInitializedFlag) {
	    if(!prevStateExists()) {
		return false;
	    } else {
	    bool status = true;
	    for(auto variableStatus : currStateMap) {
		status = status && variableStatus.second;
	    }
	    if(status)
		isInitializedFlag = true;
	    return status;
	    }
	}
	return true;
    }

    template <class T>
    bool check_violation( T* owner
		        , void (T::*pre_check)()
			, void (T::*post_check)(bool)
			) {
	if(isInitialized()) {
	    (owner->*pre_check)();
	    bool verdict = modelplex_generated::monitorSatisfied( prevState
								, currState
								, &params );
	    prevState = currState;
	    (owner->*post_check)(verdict);
	    return verdict;
	}
	return true;
    }
};

}
}

#endif
