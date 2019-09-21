#include "monitor-dl-watertank.h"
#include <rv/monitor.h>
#include <rv/dl.h>

#include "std_msgs/Float32.h"
#include "marti_common_msgs/Float32Stamped.h"

using namespace std;
namespace rosmop_generated
{
    struct testDlWatertankSpec
    {
	rv::dl::MonitorState< modelplex_generated::state
			    , modelplex_generated::parameters> monitorState;

	modelplex_generated::parameters params;

	testDlWatertankSpec() {
	    params.m = 1.0;
	    params.ep = 5.0;

	    vector<string> logicalVariables {"f", "l", "c"};

	    monitorState.initialize(params, logicalVariables);
	}

	void update_l(float l) {
	    if(monitorState.prevStateExists()) {
		monitorState.currState.l = static_cast<long double>(l);
		monitorState.currStateMap["l"] = true;
	    } else {
		monitorState.prevState.l = static_cast<long double>(l);
		monitorState.prevStateMap["l"] = true;
	    }
	}

	void update_f_c(float f, double c) {
	    if(monitorState.prevStateExists()) {
		monitorState.currState.f = static_cast<long double>(f);
		monitorState.currState.c = static_cast<long double>(c);
		monitorState.currStateMap["f"] = true;
		monitorState.currStateMap["c"] = true;
	    } else {
		monitorState.prevState.f = static_cast<long double>(f);
		monitorState.prevState.c = static_cast<long double>(c);
		monitorState.prevStateMap["f"] = true;
		monitorState.prevStateMap["c"] = true;
	    }
	}


        /* current_level */
        void callback_current_level(std_msgs::Float32& message)
        {

            float& l = message.data;

            update_l(l);

        }

	void pre_check_actions() {
	    // action condition
	    monitorState.currState.c = 0;
	}
	void post_check_actions(bool verdict) {
	    if(!verdict)
		monitorState.currState.f = 0.0;
	}

	// Needs to be generated from @violation tag
	void fallback(marti_common_msgs::Float32Stamped& message)
	{
	    monitorState.currState.f = 0.0;
	    message.value = 0.0;
	}

        /* flow_controller_input */
        void callback_flow_controller_input(marti_common_msgs::Float32Stamped& message)
        {

            float& f = message.value;

	    ros::Time& cTime = message.header.stamp;

            update_f_c(f, cTime.toSec());
            if(!monitorState.check_violation<rosmop_generated::testDlWatertankSpec>
		    ( this,
		      &rosmop_generated::testDlWatertankSpec::pre_check_actions
	            , &rosmop_generated::testDlWatertankSpec::post_check_actions) ) {
		fallback(message);
	    }
        }
    };

    struct Monitor
    {
        ros::NodeHandle n;

        rv::monitor::MonitorTopic<marti_common_msgs::Float32Stamped> flow_control_cmd;
        rv::monitor::MonitorTopic<std_msgs::Float32> level_sensor;

        rosmop_generated::testDlWatertankSpec testDlWatertankSpec;

        Monitor()
            : flow_control_cmd(n, "/flow_control_cmd", 1000)
            , level_sensor(n, "/level_sensor", 1000)
        {
            level_sensor.registerEvent(&testDlWatertankSpec, &rosmop_generated::testDlWatertankSpec::callback_current_level);
            flow_control_cmd.registerEvent(&testDlWatertankSpec, &rosmop_generated::testDlWatertankSpec::callback_flow_controller_input);
        }
    };
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "rvmonitor");
    rosmop_generated::Monitor m;
    ros::spin();
    return 0;
}
