/***********************************************************************/
/**                                                                    */
/** WorldHSFMPlugin.h                                                   */
/**                                                                    */
/**                                                                    */
/***********************************************************************/

#ifndef GAZEBO_PLUGINS_WORLDHSFMPLUGIN_HH_
#define GAZEBO_PLUGINS_WORLDHSFMPLUGIN_HH_

// C++
#include <algorithm>
#include <string>
#include <vector>

// Gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "gazebo/sensors/sensors.hh"
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

// Msgs
#include "world_sfm_hsfm_plugins/msg/forces.hpp"
#include "world_sfm_hsfm_plugins/msg/vector2.hpp"
#include "world_sfm_hsfm_plugins/msg/pose2.hpp"

// Social Force Model
#include <world_sfm_hsfm_plugins/hsfm.hpp>

namespace gazebo {
class GZ_PLUGIN_VISIBLE WorldHSFMPlugin : public WorldPlugin {
  
  // METHODS -------------------------------------------------
public:
  /// \brief Constructor
  WorldHSFMPlugin();

  /// \brief Load the actor plugin.
  /// \param[in] _world Pointer to the parent model.
  /// \param[in] _sdf Pointer to the plugin's SDF elements.
  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  // Documentation Inherited.
  virtual void Reset();

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information
  void OnUpdate(const common::UpdateInfo &_info);

  /// \brief Function that is called every update cycle (robot control not considered)
  /// \param[in] _info Timing information
  void OnUpdateOnlyActors(const common::UpdateInfo &_info);

  /// \brief Function that is called every update cycle before loading all plugin components.
  /// \param[in] _info Timing information
  void Loading(const common::UpdateInfo &_info);

private:
  /// \brief Helper function to detect the closest obstacles.
  void HandleObstacles();

  /// \brief Helper function to detect the closest obstacles - when only actors are considered
  void HandleObstaclesOnlyActors();

  /// \brief Loads the agents' parameters from a config file
  void LoadAgentsFromYaml();

  /// \brief Initialize actors pose and params for SFM plugin
  void InitializeActors();

  /// \brief Initialize robot pose and params for SFM plugin
  void InitializeRobot();

  /// \brief Method used to publish forces
  void PublishForces();

  /// \brief Called whenever new laser data is available
  void LaserCallback(ConstLaserScanStampedPtr &msg);

  /// \brief Method used to create a model to attach to actors for laser detection
  void CreateModelForActors();

  // ATTIRBUTES -------------------------------------------------
private:

  /// BOOLS FOR CONFIGURATION SETTING **************************************************************************
  
  /// \brief The integration method, if true RKF45 is used, otherwise, first order Euler is used
  bool rungeKutta45 = false;
  
  /// \brief Bool to decide wether to attach a collision model to actors for laser detection
  bool attachCollisionToActors = false;

  /// \brief Bool to decide wether to control also the robot or not
  bool controlRobot = true;

  /// \brief Bool to decide wether humans should consider the robot as a pedestrian or ignore it
  bool robotIsConsideredByHumans = true;

  /// \brief Bool to check if no agents publish forces
  bool noAgentPubForces = true;

  /// **************************************************************************************************

  /// \brief Pointer to the world, for convenience.
  physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
  sdf::ElementPtr sdf;

  /// \brief List of connections
  std::vector<event::ConnectionPtr> connections;

  /// \brief Time of the last update.
  common::Time lastUpdate;

  /// \brief The sampling time of the plugin
  double samplingTime = 0.001;

  /// \brief Node to get the parameters in the configuration file
  rclcpp::Node::SharedPtr actorParamsNode;

  /// \brief Node to get the parameters in the configuration file
  rclcpp::Node::SharedPtr actorForcesNode;

  /// \brief Stores the forces publisher
  std::vector<rclcpp::Publisher<world_sfm_hsfm_plugins::msg::Forces>::SharedPtr> actorForcesPub;

  /// \brief Client to call for the get_parameters() service
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr actorParamsClient;

  /// \brief Name of node used to get actors parameters
  std::string nodeName = "actor_params_grabber";

  /// \brief Variable to store agent names
  std::vector<std::string> agentNames;

  /// \brief Variable to store agent mass
  std::vector<int> agentMass;

  /// \brief Variable to store agent radius
  std::vector<double> agentRadius;

  /// \brief Variable to store agent relaxation time
  std::vector<double> agentRelaxTime;

  /// \brief Variable to store agent k orthogonal
  std::vector<double> agentKOrthogonal;

  /// \brief Variable to store agent k damping
  std::vector<double> agentKDamping;

  /// \brief Variable to store agent k lambda
  std::vector<double> agentKLambda;

  /// \brief Variable to store agent alpha
  std::vector<double> agentAlpha;

  /// \brief Variable to store agent group distance forward
  std::vector<double> agentGroupDistForw;

  /// \brief Variable to store agent group distance orthogonal
  std::vector<double> agentGroupDistOrth;

  /// \brief Variable to store agent k1g
  std::vector<double> agentK1g;

  /// \brief Variable to store agent k2g
  std::vector<double> agentK2g;

  /// \brief Variable to store agent Ai
  std::vector<double> agentAi;

  /// \brief Variable to store agent Aw
  std::vector<double> agentAw;

  /// \brief Variable to store agent Bi
  std::vector<double> agentBi;

  /// \brief Variable to store agent Bw
  std::vector<double> agentBw;

  /// \brief Variable to store agent k1
  std::vector<double> agentK1;

  /// \brief Variable to store agent k2
  std::vector<double> agentK2;

  /// \brief Variable to store agent desired velocity
  std::vector<double> agentDesVelocity;

  /// \brief Variable to store agent animation factor
  std::vector<double> agentAnimFact;

  /// \brief Variable to store agent animation name
  std::vector<std::string> agentAnimName;

  /// \brief Variable to store agent ignored obstacles
  std::vector<std::vector<std::string>> agentIgnoreObs;

  /// \brief Variable to store agent goals
  std::vector<std::vector<std::tuple<double,double>>> agentGoals;

  /// \brief Variable to store agent intial orientations
  std::vector<double> agentInitYaw;

  /// \brief Variable to store agent intial orientations
  std::vector<std::tuple<double,double>> agentInitPos;

  /// \brief Variable to store agent boolean publish forces
  std::vector<bool> agentPubForces;

  /// \brief Variable to store agent topic to publish forces names
  std::vector<std::string> agentTopicName;

  /// \brief Variable to store agents in the agent group
  std::vector<std::vector<std::string>> agentGroup;

  /// \brief Variable to store robot's name
  std::string robotName;

  /// \brief Vector of pointers to the actors model
  std::vector<physics::ModelPtr> agentModel;

  /// \brief Vector of pointers to the actor
  std::vector<physics::ActorPtr> actors;

  /// \brief Saves actor as a SFM agent
  std::vector<hsfm::Agent> sfmActors;

  /// \brief Vector of actors trajectories
  std::vector<physics::TrajectoryInfoPtr> trajectoryInfo;

  /// \brief Pointer to the robot model
  physics::ModelPtr robotModel;

  /// \brief Bool to check if first update
  bool robotLoaded = false;
  
  /// \brief SFM entity of the robot;
  hsfm::Agent sfmRobot;

  /// \brief Robot radius
  double robotRadius;

  /// \brief Robot mass
  double robotMass;

  /// \brief Robot desired velocity
  double robotVelocity;

  /// \brief Robot relaxation time
  double robotRelaxTime;

  /// \brief Robot k orthogonal
  double robotKOrthogonal;

  /// \brief Robot k damping
  double robotKDamping;

  /// \brief Robot k lambda
  double robotKLambda;

  /// \brief Robot alpha
  double robotAlpha;

  /// \brief Robot Ai
  double robotAi;

  /// \brief Robot Aw
  double robotAw;

  /// \brief Robot Bi
  double robotBi;

  /// \brief Robot Bw
  double robotBw;

  /// \brief Robot k1
  double robotK1;

  /// \brief Robot k2
  double robotK2;

  /// \brief Robot inital yaw
  double robotInitYaw;

  /// \brief Robot inital position
  std::tuple<double,double> robotInitPos;

  /// \brief Robot goals
  std::vector<std::tuple<double,double>> robotGoals;

  /// \brief Robot ignored obstacles
  std::vector<std::string> robotIgnoreObs;

  /// \brief Stores actors and robot SFM agents
  std::vector<hsfm::Agent> sfmEntities;

  /// \brief Store robot's and actor's models
  std::vector<physics::ModelPtr> entitiesModel;

  /// \brief Variable to store entities ignored obstacles
  std::vector<std::vector<std::string>> entitiesIgnoreObs;

  /// \brief Name of the laser sensor
  std::string laserName;

  /// \brief Pointer to the laser sensor model
  sensors::SensorPtr laserSensor;

  /// \brief Node to get the laser data
  gazebo::transport::NodePtr laserNode;

  /// \brief Subscriber to the laser data
  gazebo::transport::SubscriberPtr laserSub;

  /// \brief Vector that stores laser ranges lower than 2m and 
  // their angle (in radians) with respect to the robot expressed in the world frame
  std::vector<std::tuple<double, double>> laserRanges;

  /// \brief Stores points closest than 2m detected by the laser
  std::vector<ignition::math::Vector2d> laserObs;

  /// \brief Bool to check wether actors collision models were loaded
  bool actorCollisionLoaded = false;

  /// \brief Vector to save pointers to each Actor Collision model
  std::vector<physics::ModelPtr> actorCollisionModel;

  /// \brief Vector to store Actor Collision model names
  std::vector<std::string> actorCollisionNames;

  /// \brief Saves the height of the ground in the world
  double groundHeight;
};
}
#endif
