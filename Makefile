SHELL:=/bin/bash
CURRENT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
TEST_DIR:=$(CURRENT_DIR)/src/test
ROSMOP_DIR:=$(CURRENT_DIR)/rosmop
ROSMOP_TARGET_DIR:=$(CURRENT_DIR)/rosmop/target
TS:=$(CURRENT_DIR)/.build
LOGICPLUGINPATH:=${RVMONITOR}/target/release/lib/plugins

status=echo ==== $(@F) ==== | sed 's/\(.*\).timestamp/\1/'


LAUNCH-test : $(TS)/LAUNCH-test.timestamp

$(TS)/LAUNCH-test.timestamp : $(TS)/LAUNCH-test-publisher.timestamp  $(TS)/LAUNCH-test-subscriber.timestamp


LAUNCH-test-publisher : $(TS)/LAUNCH-test-publisher.timestamp

$(TS)/LAUNCH-test-publisher.timestamp : $(TS)/BUILD-src.timestamp
	@(source $(CURRENT_DIR)/devel/setup.bash \
	   && (rosrun rvmaster live_pub &))
	@$(status)


LAUNCH-test-subsriber : $(TS)/LAUNCH-test-subscriber.timestamp

$(TS)/LAUNCH-test-subscriber.timestamp : $(TS)/BUILD-src.timestamp
	@(source $(CURRENT_DIR)/devel/setup.bash \
	   && (rosrun rvmaster live_sub &) \
	   && (rosrun rvmaster live_sub2 &))
	@$(status)


LAUNCH-rvmaster : $(TS)/BUILD-src.timestamp $(TS)/LAUNCH-roscore.timestamp $(TS)/CREATE-custom-ros-env.timestamp
	@(source $(CURRENT_DIR)/devel/setup.bash \
	   && source $(CURRENT_DIR)/devel/custom-env.bash \
	   && rosrun rvmaster rvmaster_rosmaster)

$(TS)/LAUNCH-roscore.timestamp :
	@roscore -p 12345 &
	@sleep 3
	@touch $@
	@$(status)

BUILD-src : $(TS)/BUILD-src.timestamp

$(TS)/BUILD-src.timestamp : $(TS)/GEN-monitors.timestamp $(TEST_DIR)/live*.cpp
	@catkin_make -DCMAKE_BUILD_TYPE=Debug


$(TS)/GEN-monitors.timestamp : $(TEST_DIR)/rvmonitor.rv $(TS)/BUILD-rosmop.timestamp
	@(export LOGICPLUGINPATH=${RVMONITOR}/target/release/lib/plugins \
	   && java -jar $(ROSMOP_TARGET_DIR)/rosmop-1.0.0-SNAPSHOT.jar $<)
	@mv $(TEST_DIR)/rvmonitor.cpp $(CURRENT_DIR)/src/RVMaster/src/
	@mv $(TEST_DIR)/rvmonitor.h $(CURRENT_DIR)/src/RVMaster/include/
	@touch $<
	@$(status)


$(TS)/BUILD-rosmop.timestamp : $(TS)/CREATE-BUILD_DIR.timestamp $(shell find $(ROSMOP_DIR) -name *.java)
	@(cd $(ROSMOP_DIR) && mvn package)
	@touch $@
	@$(status)

CREATE-custom-ros-env : $(TS)/CREATE-custom-ros-env.timestamp

$(TS)/CREATE-custom-ros-env.timestamp : $(TS)/CREATE-BUILD_DIR.timestamp
	@mkdir -p $(CURRENT_DIR)/devel/
	$(file >> $(CURRENT_DIR)/devel/custom-env.bash, export ROS_MASTER_URI="http://localhost:11311/")
	$(file >> $(CURRENT_DIR)/devel/custom-env.bash, export REAL_MASTER_URI="http://localhost:12345/")
	$(file >> $(CURRENT_DIR)/devel/custom-env.bash, export ACCESS_POLICY_PATH=$(CURRENT_DIR)/config/access-policy.cfg)
	@touch $@
	@$(status)


$(TS)/CREATE-BUILD_DIR.timestamp :
	@mkdir -p $(@D)
	@touch $@

CLEAN-processes :
	@(pkill "roscore" ; rm -f $(TS)/LAUNCH-roscore.timestamp \
	   ; pkill "rvmaster_rosmas" ; rm -f $(TS)/LAUNCH-rvmaster.timestamp \
	   ; pkill "live_pub" ; rm -f $(TS)/LAUNCH-test.timestamp \
	   ; pkill "live_sub") || true
	@$(status)


clean : CLEAN-processes
	@rm -rf $(TS)
	@rm -f $(TEST_DIR)/rvmonitor.cpp $(CURRENT_DIR)/src/RVMaster/src/rvmonitor.cpp
	@rm -f $(TEST_DIR)/rvmonitor.h $(CURRENT_DIR)/src/RVMaster/include/rvmonitor.h

