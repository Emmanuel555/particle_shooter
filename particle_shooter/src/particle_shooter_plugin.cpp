#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>

#include <unistd.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>


#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo
{
class ParticleShooterPlugin : public WorldPlugin
{
public:
  ParticleShooterPlugin() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    
    
    this->world = _world;
    GZ_ASSERT(this->world != NULL, "Got NULL world pointer!");
    this->sdf = _sdf;
    GZ_ASSERT(this->sdf != NULL, "Got NULL SDF element pointer!");
    
    // Check if Config Elements exist, otherwise they will have default value
    if (_sdf->HasElement("number_of_particles"))
      this->number_of_particles = _sdf->Get<double>("number_of_particles");

    if (_sdf->HasElement("x_axis_force"))
      this->x_axis_force = _sdf->Get<double>("x_axis_force");
    if (_sdf->HasElement("y_axis_force"))
      this->y_axis_force = _sdf->Get<double>("y_axis_force");
    if (_sdf->HasElement("z_axis_force"))
      this->z_axis_force = _sdf->Get<double>("z_axis_force");
      
    if (_sdf->HasElement("x_origin"))
      this->x_origin = _sdf->Get<double>("x_origin");
    if (_sdf->HasElement("y_origin"))
      this->y_origin = _sdf->Get<double>("y_origin");
    if (_sdf->HasElement("z_origin"))
      this->z_origin = _sdf->Get<double>("z_origin");
      
    if (_sdf->HasElement("random_range"))
      this->random_range = _sdf->Get<double>("random_range");  
    
    
    
    // We wait for all system to be ready an amount of seconds
    float seconds_to_wait = 5.0;
    this->WaitForseconds(seconds_to_wait);

    // Update Time Init
    this->old_secs =this->world->SimTime().Float();
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ParticleShooterPlugin::OnUpdate, this));


    ROS_DEBUG("Particle Shooter Ready....");
  }


  void Reset()
  {
    this->reseting_plugin = true;
    ROS_ERROR("Reseted the simulation world, we Restart the time clock");
    // Update Time Init
    this->old_secs = 0.0;
    double new_secs = 0.0;
    double delta = -1.0;

    while (delta < 0.0)
    {
      // We change Direction
      ROS_ERROR("Waiting until Clock is reseted and delta is not negative > Update delta=%f, new_secs=%f", delta,  new_secs);
      new_secs = this->world->SimTime().Float();
      delta = new_secs - this->old_secs;
      ROS_ERROR("Updated until Clock is reseted > Update delta=%f, new_secs=%f", delta,  new_secs);

    }

    this->reseting_plugin = false;

  }


  // Called by the world update start event
  public: void OnUpdate()
  {


    if (this->reseting_plugin)
    {
        ROS_ERROR("Reseting in Process, please wait...");
    }else
    {
        // TODO: Check what is necessary now here
        double new_secs =this->world->SimTime().Float();
        double delta = new_secs - this->old_secs;

        double max_delta = 0.0;

        if (this->update_frequency != 0.0)
        {
          max_delta = 1.0 / this->update_frequency;
        }

        if (delta > max_delta && delta != 0.0)
        {
          // We update the Old Time variable.
          this->old_secs = new_secs;
          
          // Update the Particles
          UpdateParticles();
        }
    }



  }

  void WaitForseconds(float seconds_to_wait)
  {
    unsigned int microseconds;
    microseconds = seconds_to_wait * 1e6;
    ROS_DEBUG("Waiting for %f seconds",seconds_to_wait);
    usleep(microseconds);
    ROS_DEBUG("Done waiting...");

  }

  void UpdateParticles()
  {
    //this->SetForceParticles();
    this->MoveParticles();
  }
  
  void MoveParticles()
  {
      
    std::string particle_base_name = "particle";

    for (auto model : this->world->Models())
    {
        std::string model_name = model->GetName();

        float x_pos_rand = 0.0;
        float y_pos_rand = 0.0;
        float z_pos_rand = 0.0;
        float roll_rand = 0.0;
        float pitch_rand = 0.0;
        float yaw_rand = 0.0;

        // Id the model name contains the substring particle, we consider it a particle
        if (model_name.find(particle_base_name) == std::string::npos)
        {
            ROS_DEBUG("Moving model=%s",model_name.c_str());

            float x_pos_rand = RandomFloat(this->x_origin - this->random_range, this->x_origin + this->random_range);
            float y_pos_rand = RandomFloat(this->y_origin - this->random_range, this->y_origin + this->random_range);
            float z_pos_rand = RandomFloat(this->z_origin - this->random_range, this->z_origin + this->random_range);
            
            /***
            float roll_rand = RandomFloat(this->roll_min, this->roll_max);
            float pitch_rand = RandomFloat(this->pitch_min, this->pitch_max);
            float yaw_rand = RandomFloat(this->yaw_min, this->yaw_max);
            ***/
            
            ROS_DEBUG("POSE-RANDOM[X,Y,Z,Roll,Pitch,Yaw=[%f,%f,%f,%f,%f,%f], model=%s", x_pos_rand,y_pos_rand,z_pos_rand,roll_rand,pitch_rand,yaw_rand,model_name.c_str());
            //ignition::math::Pose3 initPose(ignition::math::Vector3<float>(x_pos_rand, y_pos_rand, z_pos_rand), ignition::math::Quaternion<float>(roll_rand, pitch_rand, yaw_rand));
            
            model->SetWorldPose(
                                ignition::math::Pose3d(
                                    ignition::math::Vector3d(x_pos_rand, y_pos_rand, z_pos_rand),
                                    ignition::math::Quaterniond(roll_rand, pitch_rand, yaw_rand)
                                                    )
                                );
            
            ROS_DEBUG("Moving model=%s....END",model_name.c_str());

        }

    }

  }


  
  float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
  }
  
  
  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  
  /// \brief World pointer.
  protected: gazebo::physics::WorldPtr world;
  /// \brief SDF pointer.
  protected: sdf::ElementPtr sdf;
  /// \brief Maps model IDs to colors
  private: std::map<int, gazebo::common::Color> lastKnownColors;
  /// \brief Maps model IDs to ModelNames
  private: std::map<int, std::string> modelIDToName;
  
  /// \brief Maps light IDs to LightNames
  private: std::map<int, std::string> lightIDToName;
  
  // Update Loop frequency
  double update_frequency = 2.0;
  // Time Memory
  double old_secs;
  // Number of particles
  int number_of_particles = 10;
  // Force Direction
  double x_axis_force = 0.0;
  double y_axis_force = 0.0;
  double z_axis_force = -1.0;
  double x_origin = 0.0;
  double y_origin = 0.0;
  double z_origin = 1.0;
  
  double random_range = 0.1;

  // Reseting Flag
  bool reseting_plugin = false;
  

};
GZ_REGISTER_WORLD_PLUGIN(ParticleShooterPlugin)
}