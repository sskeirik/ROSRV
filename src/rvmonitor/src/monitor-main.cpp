int main(int argc, char** argv)
{
  ros::init(argc, argv, "rvmonitor");
  rv::monitor::Monitor monitor;
  rosmop_generated::testSpecOne testSpecOne(monitor);
  monitor.enable_rvmaster_shims();
  ros::spin();
  return 0;
}
