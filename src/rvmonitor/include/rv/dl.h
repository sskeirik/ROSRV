#ifndef RV_DL_H
#define RV_DL_H

#include<vector>
#include<string>
#include<map>


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


    void initialize(std::vector<std::string> variables) {
	this->logicalVariables = variables;

	prevStateExistsFlag = false;
	isInitializedFlag = false;

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
};

}
}

#endif
