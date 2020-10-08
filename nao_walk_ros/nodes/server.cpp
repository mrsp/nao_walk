 #include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <nao_walk/GaitControlConfig.h>

void callback(nao_walk::GaitControlConfig &config, uint32_t level) {
      ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
             config.Observer_COPX, config.Observer_COPY, config.Observer_CoMX,config.Observer_CoMY, config.COP_NoiseX, config.COP_NoiseY, config.CoM_NoiseX, config.CoM_NoiseY,
            config.Kp_PitchT, config.Kd_PitchT, config.Kp_RollT, config.Kd_RollT,  config.kfx,  config.kfy, config.StepHeight);

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