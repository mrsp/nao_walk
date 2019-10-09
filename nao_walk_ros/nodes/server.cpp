 #include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <nao_walk/GaitControlConfig.h>

void callback(nao_walk::GaitControlConfig &config, uint32_t level) {
      ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f %f %f", 
            config.COP_Noise,config.CoM_Noise,config.Observer_COP,config.Observer_CoM,
            config.Kp_Pitch, config.Kd_Pitch,config.Kp_Roll,config.Kd_Roll,config.amX,config.amY);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "nao_walk_dynamic_configure");

  dynamic_reconfigure::Server<nao_walk::GaitControlConfig> server;
  dynamic_reconfigure::Server<nao_walk::GaitControlConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  ros::spin();
  return 0;
}