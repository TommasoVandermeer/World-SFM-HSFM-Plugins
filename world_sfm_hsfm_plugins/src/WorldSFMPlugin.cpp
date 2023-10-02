/***********************************************************************/
/**                                                                    */
/** WorldSFMPlugin.cpp                                                 */
/**                                                                    */
/** Author: Tommaso Van Der Meer (tommaso.vander@unisi.it)             */
/**                                                                    */
/***********************************************************************/

#include <functional>
#include <stdio.h>
#include <string>

#include <world_sfm_hsfm_plugins/WorldSFMPlugin.h>

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(WorldSFMPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
WorldSFMPlugin::WorldSFMPlugin() {}

/////////////////////////////////////////////////
void WorldSFMPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) {
  this->sdf = _sdf;
  this->world = _world;
  
  // Get the actors parameters
  this->actorParamsNode = std::make_shared<rclcpp::Node>(this->nodeName);
  this->actorParamsClient = this->actorParamsNode->create_client<rcl_interfaces::srv::GetParameters>("/agent_params_loader/get_parameters");

  // Load agents parameters from Yaml file
  this->LoadAgentsFromYaml();

  // Set the initial pose of each actor and load all params on the SFM plugin
  this->InitializeActors();

  // Node to publish forces
  this->actorForcesNode = std::make_shared<rclcpp::Node>("publish_Forces_Node");

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
    std::bind(&WorldSFMPlugin::Loading, this, std::placeholders::_1)));

  // Reset settings (goals and animations)
  this->Reset();
}

/////////////////////////////////////////////////
void WorldSFMPlugin::Loading(const common::UpdateInfo &_info) {

  /// Both collision models to actors and robot control
  if ((this->attachCollisionToActors) && (this->controlRobot)) {
    if ((!this->robotLoaded) || (!this->actorCollisionLoaded)) {
      // Check for actor collision models spawns
      for (unsigned int i = 0; i < this->actorCollisionNames.size(); ++i) {
        bool check = false;
        for (unsigned int j = 0; j < this->world->Models().size(); ++j) {
          if ((this->world->Models()[j]->GetName() == this->robotName) && (!this->robotLoaded)) {
            // Set the initial pose of the robot and load all params on the SFM plugin
            this->InitializeRobot();
            this->robotLoaded = true;
          }
          if (this->actorCollisionNames[i] == this->world->Models()[j]->GetName()) check = true;
        }
        if (!check) break;
        if ((check) && (i == this->actorCollisionNames.size()-1)) this->actorCollisionLoaded = true;
      } 
    } else {
      // Add collision models to ignore obs and save Collision models
      for (unsigned int i = 0; i < this->actorCollisionNames.size(); i++) {
        this->actorCollisionModel.push_back(this->world->ModelByName(this->actorCollisionNames[i]));
        for (unsigned int j = 0; j < this->entitiesIgnoreObs.size(); j++) {
          this->entitiesIgnoreObs[j].push_back(this->actorCollisionNames[i]);
          if (j < this->entitiesIgnoreObs.size()-1) this->agentIgnoreObs[j].push_back(this->actorCollisionNames[i]);
          else this->robotIgnoreObs.push_back(this->actorCollisionNames[i]);
        }
      }
      this->connections.pop_back();
      this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
        std::bind(&WorldSFMPlugin::OnUpdate, this, std::placeholders::_1)));
    }
  }

  /// No collision models attached to actors but there is robot control
  if ((!this->attachCollisionToActors) && (this->controlRobot)){
    if (!this->robotLoaded) {
      for (unsigned int i = 0; i < this->world->Models().size(); ++i) {
        if (this->world->Models()[i]->GetName() == this->robotName) {
          this->InitializeRobot();
          this->robotLoaded = true;
        }
      }
    } else {
      this->connections.pop_back();
      this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
        std::bind(&WorldSFMPlugin::OnUpdate, this, std::placeholders::_1)));
    }
  }

  /// No robot control but there are collision models attached to actors
  if ((this->attachCollisionToActors) && (!this->controlRobot)){
    if (!this->actorCollisionLoaded) {
      // Check for actor collision models spawns
      for (unsigned int i = 0; i < this->actorCollisionNames.size(); ++i) {
        bool check = false;
        for (unsigned int j = 0; j < this->world->Models().size(); ++j) {
          if (this->actorCollisionNames[i] == this->world->Models()[j]->GetName()) check = true;
        }
        if (!check) break;
        if ((check) && (i == this->actorCollisionNames.size()-1)) this->actorCollisionLoaded = true;
      } 
    } else {
      // Add collision models to ignore obs and save Collision models
      for (unsigned int i = 0; i < this->actorCollisionNames.size(); i++) {
        this->actorCollisionModel.push_back(this->world->ModelByName(this->actorCollisionNames[i]));
        for (unsigned int j = 0; j < this->agentIgnoreObs.size(); j++) {
          this->agentIgnoreObs[j].push_back(this->actorCollisionNames[i]);
        }
      }
      this->connections.pop_back();
      this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
        std::bind(&WorldSFMPlugin::OnUpdateOnlyActors, this, std::placeholders::_1)));
    }
  }
  
  /// No robot control and no collision models attached to actors
  if ((!this->attachCollisionToActors) && (!this->controlRobot)) {
    this->connections.pop_back();
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&WorldSFMPlugin::OnUpdateOnlyActors, this, std::placeholders::_1)));
  }
}

/////////////////////////////////////////////////
void WorldSFMPlugin::Reset() {
  this->lastUpdate = 0;

  for (unsigned int i = 0; i < this->actors.size(); ++i) {
    // Intialize skeleton animation
    auto skelAnims = this->actors[i]->SkeletonAnimations();
    if (skelAnims.find(this->agentAnimName[i]) == skelAnims.end()) {
      gzerr << "Skeleton animation " << this->agentAnimName[i] << " not found.\n";
    } else {
      physics::TrajectoryInfoPtr trajectory;
      trajectory.reset(new physics::TrajectoryInfo());
      trajectory->type = this->agentAnimName[i];
      trajectory->duration = 1.0;
      this->trajectoryInfo.push_back(trajectory);
      this->actors[i]->SetCustomTrajectory(this->trajectoryInfo[i]);
    }
    // Check if no agent publish forces or visualize forces
    if (this->agentPubForces[i]) {
      this->noAgentPubForces = false;
    }
    // Create publisher for each agent
    rclcpp::Publisher<world_sfm_hsfm_plugins::msg::Forces>::SharedPtr pub = this->actorForcesNode->create_publisher<world_sfm_hsfm_plugins::msg::Forces>(this->agentTopicName[i], 10);
    this->actorForcesPub.push_back(pub);
  }
}

/////////////////////////////////////////////////
void WorldSFMPlugin::HandleObstacles() {
  for (unsigned int k = 0; k < this->sfmEntities.size(); ++k) {
    if (this->entitiesModel[k]->GetId() == this->robotModel->GetId()) {
      // If the robot is considered take the obs found by laser
      this->sfmRobot.obstacles1.clear();
      this->sfmEntities[k].obstacles1.clear();
      for (unsigned int i = 0; i < this->laserObs.size(); ++i) {
        utils::Vector2d ob(this->laserObs[i].X(), this->laserObs[i].Y());
        this->sfmEntities[k].obstacles1.push_back(ob);
        this->sfmRobot.obstacles1.push_back(ob);
      }
    } else {
      double minDist;
      ignition::math::Vector2d closest_obs;
      ignition::math::Vector2d closest_obs2;
      this->sfmEntities[k].obstacles1.clear();
      this->sfmActors[k].obstacles1.clear();

      for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
        physics::ModelPtr model = this->world->ModelByIndex(i);
        // If the model lowest Z is greater than actor's highest Z, the model is added to Ignore Obs
        // if (model->CollisionBoundingBox().Min().Z() > this->entitiesModel[k]->CollisionBoundingBox().Max().Z()) {
        //   this->entitiesIgnoreObs[k].push_back(model->GetName());
        //   ++i;
        //   model = this->world->ModelByIndex(i);;
        // }
        // Check if obstacle is in Ignore Obs
        if (std::find(this->entitiesIgnoreObs[k].begin(), this->entitiesIgnoreObs[k].end(), model->GetName()) == this->entitiesIgnoreObs[k].end()) {
          ignition::math::Vector3d pose = this->entitiesModel[k]->WorldPose().Pos();
          ignition::math::Vector3d modelPos = model->WorldPose().Pos();
          
          ignition::math::Vector2d minBB(model->CollisionBoundingBox().Min().X(),model->CollisionBoundingBox().Min().Y());
          ignition::math::Vector2d maxBB(model->CollisionBoundingBox().Max().X(),model->CollisionBoundingBox().Max().Y());
          ignition::math::Vector2d thirdV(minBB.X(),maxBB.Y());
          ignition::math::Vector2d fourthV(maxBB.X(),minBB.Y());

          std::vector<std::vector<ignition::math::Vector2d>> segments = {{minBB,fourthV},{fourthV,maxBB},{thirdV,maxBB},{minBB,thirdV}};

          ignition::math::Vector2d a;
          ignition::math::Vector2d b;
          ignition::math::Vector2d h;
          double dist;

          minDist = 10000;

          for(unsigned int j = 0; j < segments.size(); ++j) {
            a = std::min(segments[j][0], segments[j][1]);
            b = std::max(segments[j][0], segments[j][1]);
            double t = ((pose.X() - a.X()) * (b.X() - a.X()) + (pose.Y() - a.Y()) * (b.Y() - a.Y())) / (std::pow(b.X() - a.X(), 2) + std::pow(b.Y() - a.Y(), 2));
            double t_star = std::min(std::max(0.0,t),1.0);
            h = a + t_star * (b - a);
            dist = std::sqrt(std::pow(h.X() - pose.X(), 2) + std::pow(h.Y() - pose.Y(),2));
            if (dist < minDist) {
              minDist = dist;
              closest_obs = h;
            }
            // At the last segment,the closest point of the obstacle is passed to the lightSFM library if its distance is lower than 2 meters
            if (j == segments.size() - 1 && minDist < 2) {
              utils::Vector2d ob(closest_obs.X(), closest_obs.Y());
              this->sfmEntities[k].obstacles1.push_back(ob);
              this->sfmActors[k].obstacles1.push_back(ob);
            }
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void WorldSFMPlugin::HandleObstaclesOnlyActors() {
  for (unsigned int k = 0; k < this->sfmActors.size(); ++k) {
    double minDist;
    ignition::math::Vector2d closest_obs;
    ignition::math::Vector2d closest_obs2;
    this->sfmActors[k].obstacles1.clear();

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
      physics::ModelPtr model = this->world->ModelByIndex(i);
      if (std::find(this->agentIgnoreObs[k].begin(), this->agentIgnoreObs[k].end(), model->GetName()) == this->agentIgnoreObs[k].end()) {
        ignition::math::Vector3d pose = this->actors[k]->WorldPose().Pos();
        ignition::math::Vector3d modelPos = model->WorldPose().Pos();
        
        ignition::math::Vector2d minBB(model->CollisionBoundingBox().Min().X(),model->CollisionBoundingBox().Min().Y());
        ignition::math::Vector2d maxBB(model->CollisionBoundingBox().Max().X(),model->CollisionBoundingBox().Max().Y());
        ignition::math::Vector2d thirdV(minBB.X(),maxBB.Y());
        ignition::math::Vector2d fourthV(maxBB.X(),minBB.Y());

        std::vector<std::vector<ignition::math::Vector2d>> segments = {{minBB,fourthV},{fourthV,maxBB},{thirdV,maxBB},{minBB,thirdV}};

        ignition::math::Vector2d a;
        ignition::math::Vector2d b;
        ignition::math::Vector2d h;
        double dist;

        minDist = 10000;

        for(unsigned int j = 0; j < segments.size(); ++j) {
          a = std::min(segments[j][0], segments[j][1]);
          b = std::max(segments[j][0], segments[j][1]);
          double t = ((pose.X() - a.X()) * (b.X() - a.X()) + (pose.Y() - a.Y()) * (b.Y() - a.Y())) / (std::pow(b.X() - a.X(), 2) + std::pow(b.Y() - a.Y(), 2));
          double t_star = std::min(std::max(0.0,t),1.0);
          h = a + t_star * (b - a);
          dist = std::sqrt(std::pow(h.X() - pose.X(), 2) + std::pow(h.Y() - pose.Y(),2));
          if (dist < minDist) {
            minDist = dist;
            closest_obs = h;
          }
          // At the last segment,the closest point of the obstacle is passed to the lightSFM library if its distance is lower than 2 meters
          if (j == segments.size() - 1 && minDist < 2) {
            utils::Vector2d ob(closest_obs.X(), closest_obs.Y());
            this->sfmActors[k].obstacles1.push_back(ob);
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void WorldSFMPlugin::OnUpdate(const common::UpdateInfo &_info) {
  double dt = (_info.simTime - this->lastUpdate).Double();
  // If sampling time is passed from last update
  if (dt >= this->samplingTime){
    // Update closest obstacle
    HandleObstacles();
    
    if (this->robotIsConsideredByHumans){
      this->sfmEntities = sfm::SFM.computeForces(this->sfmEntities);
      // We erase the social forces for the robot, in this way, it only relies on laser data
      this->sfmEntities.back().forces.globalForce -= this->sfmEntities.back().forces.socialForce;
      // Update model
      if (!this->rungeKutta45) {
        this->sfmEntities = sfm::SFM.updatePosition(this->sfmEntities, dt);
      } else {
        this->sfmEntities = sfm::SFM.updatePositionRKF45(this->sfmEntities, _info.simTime.Double(), dt);
      }
    } else {
      sfm::SFM.computeForces(this->sfmRobot, this->sfmActors);
      // We erase the social forces for the robot, in this way, it only relies on laser data
      this->sfmRobot.forces.globalForce -= this->sfmRobot.forces.socialForce;
      this->sfmActors = sfm::SFM.computeForces(this->sfmActors);
      if (!this->rungeKutta45) {
        this->sfmActors = sfm::SFM.updatePosition(this->sfmActors, dt);
        sfm::SFM.updatePosition(this->sfmRobot, dt);
      } else {
        this->sfmActors = sfm::SFM.updatePositionRKF45(this->sfmActors, _info.simTime.Double(), dt);
        sfm::SFM.updatePositionRKF45(this->sfmRobot, _info.simTime.Double(), dt);
      }
      // Save new values
      this->sfmEntities = this->sfmActors;
      this->sfmEntities.push_back(this->sfmRobot);
    }

    // Publish forces
    if (!this->noAgentPubForces) PublishForces();

    for (unsigned int i = 0; i < this->sfmEntities.size(); ++i) {
      ignition::math::Pose3d pose = this->entitiesModel[i]->WorldPose();
      if (this->entitiesModel[i]->GetId() != this->robotModel->GetId()) {
        utils::Angle h = this->sfmEntities[i].yaw;
        utils::Angle add = utils::Angle::fromRadian(1.5707);
        h = h + add;
        double yaw = h.toRadian();

        pose.Pos().X(this->sfmEntities[i].position.getX());
        pose.Pos().Y(this->sfmEntities[i].position.getY());
        pose.Pos().Z(0.91 - this->groundHeight);
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, yaw);

        double distanceTraveled = (pose.Pos() - this->entitiesModel[i]->WorldPose().Pos()).Length();

        this->entitiesModel[i]->SetWorldPose(pose, false, false);
        this->actors[i]->SetScriptTime(this->actors[i]->ScriptTime() + (distanceTraveled * this->agentAnimFact[i]));
      
        if (this->attachCollisionToActors) this->actorCollisionModel[i]->SetWorldPose(this->entitiesModel[i]->WorldPose());
      } else {
        utils::Angle h = this->sfmEntities[i].yaw;
        double yaw = h.toRadian();

        pose.Pos().X(this->sfmEntities[i].position.getX());
        pose.Pos().Y(this->sfmEntities[i].position.getY());
        pose.Pos().Z(this->groundHeight); // This must be exactly the position of the ground, otherwise gravity comes into play
        pose.Rot() = ignition::math::Quaterniond(0, 0, yaw);

        this->entitiesModel[i]->SetWorldPose(pose, true, false);
      }
    }

    this->lastUpdate = _info.simTime;
  }
}

/////////////////////////////////////////////////
void WorldSFMPlugin::OnUpdateOnlyActors(const common::UpdateInfo &_info) {
  double dt = (_info.simTime - this->lastUpdate).Double();
  // If sampling time is passed from last update
  if (dt >= this->samplingTime){
    // Update closest obstacle
    HandleObstaclesOnlyActors();

    this->sfmActors = sfm::SFM.computeForces(this->sfmActors);

    // Update model
    if (!this->rungeKutta45) {
      this->sfmActors = sfm::SFM.updatePosition(this->sfmActors, dt);
    } else {
      this->sfmActors = sfm::SFM.updatePositionRKF45(this->sfmActors, _info.simTime.Double(), dt);
    }

    // Publish forces
    if (!this->noAgentPubForces) PublishForces();

    for (unsigned int i = 0; i < this->sfmActors.size(); ++i) {
      ignition::math::Pose3d actorPose = this->actors[i]->WorldPose();

      utils::Angle h = this->sfmActors[i].yaw;
      utils::Angle add = utils::Angle::fromRadian(1.5707);
      h = h + add;
      double yaw = h.toRadian();

      actorPose.Pos().X(this->sfmActors[i].position.getX());
      actorPose.Pos().Y(this->sfmActors[i].position.getY());
      actorPose.Pos().Z(0.91 - this->groundHeight);
      actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, yaw);

      double distanceTraveled = (actorPose.Pos() - this->actors[i]->WorldPose().Pos()).Length();

      this->actors[i]->SetWorldPose(actorPose, false, false);
      this->actors[i]->SetScriptTime(this->actors[i]->ScriptTime() + (distanceTraveled * this->agentAnimFact[i]));

      if (this->attachCollisionToActors) this->actorCollisionModel[i]->SetWorldPose(this->actors[i]->WorldPose());
    }
    this->lastUpdate = _info.simTime;
  }
}

////////////////////////////////////////////////
void WorldSFMPlugin::LoadAgentsFromYaml() {
  // FIRST REQUEST to get the agents names and other global parameters
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"agents","sampling_time","runge_kutta_45","robot_name","robot_initial_orientation", \
  "robot_waypoints","robot_initial_position","robot_radius","robot_mass","robot_goal_weight", \
  "robot_obstacle_weight", "robot_social_weight", "robot_velocity", "robot_ignore_obstacles", \
  "laser_sensor_name", "attach_cylinder_to_humans","ground_height","robot_control","consider_robot"};
  this->actorParamsClient->wait_for_service();
  auto future = this->actorParamsClient->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->actorParamsNode, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto results = future.get()->values;
    // Sampling time
    if (results[1].type == rclcpp::PARAMETER_NOT_SET) {
      this->samplingTime = 0.001;
      std::cout<<"Sampling time not set, setting it to the default value: 0.001"<<std::endl;
    } else {
      this->samplingTime = results[1].double_value;
    }
    // Integration method
    if (results[2].type == rclcpp::PARAMETER_NOT_SET) {
      this->rungeKutta45 = false;
      std::cout<<"Integration method not set, setting it to the default value: Euler"<<std::endl;
    } else {
      this->rungeKutta45 = results[2].bool_value;
    }
    // Robot control
    if (results[17].type == rclcpp::PARAMETER_NOT_SET) {
      this->controlRobot = false;
      std::cout<<"Robot control not set, setting it to the default value: False"<<std::endl;
    } else {
      this->controlRobot = results[17].bool_value;
    }
    // Agent names and Agent model initialization
    for(unsigned int i = 0; i < results[0].string_array_value.size(); ++i){
      this->agentNames.push_back(results[0].string_array_value[i]);
      this->agentModel.push_back(this->world->ModelByName(this->agentNames[i]));
      this->actors.push_back(boost::dynamic_pointer_cast<physics::Actor>(this->agentModel[i]));
    }
    if (this->controlRobot) {
      // Robot name
      if (results[3].type == rclcpp::PARAMETER_NOT_SET) {
        this->robotName = "robot";
        std::cout<<"Robot name not set, setting it to the default value: robot"<<std::endl;
      } else {
        this->robotName = results[3].string_value;
      }
      // Robot Waypoints - THESE MUST BE LOADED
      for (unsigned int j = 0; j <results[5].double_array_value.size(); j+=2){
        this->robotGoals.push_back(std::make_tuple(results[5].double_array_value[j],results[5].double_array_value[j+1]));
      }
      // Robot initial orientation
      if (results[4].type == rclcpp::PARAMETER_NOT_SET) {
        this->robotInitYaw = 0.0;
        std::cout<<"Robot initial orientation not set, setting it to the default value: 0.0°"<<std::endl;
      } else {
        this->robotInitYaw = results[4].double_value * (M_PI_2 / 90.0);
      }
      // Robot initial position
      if (results[6].type == rclcpp::PARAMETER_NOT_SET) {
        this->robotInitPos = this->robotGoals[0];
        std::cout<<"Robot initial position not set, setting it as the first robot goal: "<<std::get<0>(this->robotGoals[0])<<","<<std::get<1>(this->robotGoals[0])<<std::endl;
      } else {
        std::tuple<double,double> pos(results[6].double_array_value[0],results[6].double_array_value[1]);
        this->robotInitPos = pos;
      }
      // Robot radius
      if (results[7].type == rclcpp::PARAMETER_NOT_SET) {
        this->robotRadius = 0.35;
        std::cout<<"Radius for robot not set, setting it to the default value: 0.35"<<std::endl;
      } else {
        this->robotRadius = results[7].double_value;
      }
      // Robot mass
      if (results[8].type == rclcpp::PARAMETER_NOT_SET) {
        this->robotMass = 35;
        std::cout<<"Mass for robot not set, setting it to the default value: 35"<<std::endl;
      } else {
        this->robotMass = results[8].integer_value;
      }
      // Robot goal weight
      if (results[9].type == rclcpp::PARAMETER_NOT_SET) {
        this->robotGoalWeight = 2.0;
        std::cout<<"Goal weight for robot not set, setting it to the default value: 2.0"<<std::endl;
      } else {
        this->robotGoalWeight = results[9].double_value;
      }
      // Robot obstacle weight
      if (results[10].type == rclcpp::PARAMETER_NOT_SET) {
        this->robotObstacleWeight = 10.0;
        std::cout<<"Obstacle weight for robot not set, setting it to the default value: 10.0"<<std::endl;
      } else {
        this->robotObstacleWeight = results[10].double_value;
      }
      // Robot social weight
      if (results[11].type == rclcpp::PARAMETER_NOT_SET) {
        this->robotSocialWeight = 15.0;
        std::cout<<"Social weight for robot not set, setting it to the default value: 15.0"<<std::endl;
      } else {
        this->robotSocialWeight = results[11].double_value;
      }
      // Robot velocity 
      if (results[12].type == rclcpp::PARAMETER_NOT_SET) {
        this->robotVelocity = 1.0;
        std::cout<<"Velocity for robot not set, setting it to the default value: 1.0"<<std::endl;
      } else {
        this->robotVelocity = results[12].double_value;
      }
      // Robot ingore obstacles
      for (unsigned int j = 0; j < this->agentNames.size(); ++j){
        this->robotIgnoreObs.push_back(this->agentNames[j]); // All the actor's models must be ignored
      }
      this->robotIgnoreObs.push_back(this->robotName); // Robot must ignore itself
      if (results[13].type == rclcpp::PARAMETER_NOT_SET) {
        std::cout<<"No obstacles to ignore for robot"<<std::endl;
      } else {
        for (unsigned int j = 0; j <results[13].string_array_value.size(); ++j){
          this->robotIgnoreObs.push_back(results[13].string_array_value[j]);
        }
      }
      // Laser name
      if (results[14].type == rclcpp::PARAMETER_NOT_SET) {
        this->laserName = "laser";
        std::cout<<"Laser name not set, setting it to the default value: laser"<<std::endl;
      } else {
        this->laserName = results[14].string_value;
      }
    }
    // Attach Cylinder to Actors
    if (results[15].type == rclcpp::PARAMETER_NOT_SET) {
      this->attachCollisionToActors = false;
      std::cout<<"Attach_cylyinder_to_actors not set, setting it to the default value: False"<<std::endl;
    } else {
      this->attachCollisionToActors = results[15].bool_value;
    }
    // Ground height
    if (results[16].type == rclcpp::PARAMETER_NOT_SET) {
      this->groundHeight = -0.1;
      std::cout<<"Ground height not set, setting it to the default value: -0.1"<<std::endl;
    } else {
      this->groundHeight = results[16].double_value;
    }
    // Humans consider robot
    if (results[18].type == rclcpp::PARAMETER_NOT_SET) {
      this->robotIsConsideredByHumans = false;
      std::cout<<"Humans consider robot not set, setting it to the default value: False"<<std::endl;
    } else {
      if (this->controlRobot) this->robotIsConsideredByHumans = results[18].bool_value;
      else this->robotIsConsideredByHumans = false;
    }

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_parameters()");
  }

  // SECOND ROUND OF REQUESTS to get the parameters of each agent
  for(unsigned int i = 0; i < this->agentNames.size(); ++i){
    request->names.clear();
    request->names = {this->agentNames[i] + ".mass", this->agentNames[i] + ".radius", this->agentNames[i] + ".goal_weight", \
    this->agentNames[i] + ".obstacle_weight", this->agentNames[i] + ".social_weight", this->agentNames[i] + ".group_gaze_weight", \
    this->agentNames[i] + ".group_coh_weight", this->agentNames[i] + ".group_rep_weight", this->agentNames[i] + ".velocity", \
    this->agentNames[i] + ".animation_factor", this->agentNames[i] + ".animation_name", this->agentNames[i] + ".people_distance", \
    this->agentNames[i] + ".ignore_obstacles", this->agentNames[i] + ".waypoints", this->agentNames[i] + ".publish_forces", \
    this->agentNames[i] + ".group", this->agentNames[i] + ".initial_orientation", this->agentNames[i] + ".initial_position"};

    auto future = this->actorParamsClient->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->actorParamsNode, future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto results = future.get()->values;
      // Mass
      if (results[0].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentMass.push_back(75);
        std::cout<<"Mass for " + this->agentNames[i] + " not set, setting it to the default value: 75"<<std::endl;
      } else {
        this->agentMass.push_back(results[0].integer_value);
      }
      // Radius
      if (results[1].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentRadius.push_back(0.35);
        std::cout<<"Radius for " + this->agentNames[i] + " not set, setting it to the default value: 0.35"<<std::endl;
      } else {
        this->agentRadius.push_back(results[1].double_value);
      }
      // Goal Weight
      if (results[2].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentGoalWeight.push_back(2.0);
        std::cout<<"Goal weight for " + this->agentNames[i] + " not set, setting it to the default value: 2.0"<<std::endl;
      } else {
        this->agentGoalWeight.push_back(results[2].double_value);
      }
      // Obstacle Weight
      if (results[3].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentObstacleWeight.push_back(10.0);
        std::cout<<"Obstacle weight for " + this->agentNames[i] + " not set, setting it to the default value: 10.0"<<std::endl;
      } else {
        this->agentObstacleWeight.push_back(results[3].double_value);
      }
      // Social Weight
      if (results[4].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentSocialWeight.push_back(15.0);
        std::cout<<"Social weight for " + this->agentNames[i] + " not set, setting it to the default value: 15.0"<<std::endl;
      } else {
        this->agentSocialWeight.push_back(results[4].double_value);
      }
      // Group Gaze Weight
      if (results[5].type == rclcpp::PARAMETER_NOT_SET){
        this->agentGroupGaze.push_back(0.0);
        std::cout<<"Group gaze weight for " + this->agentNames[i] + " not set, setting it to the default value: 0.0"<<std::endl;
      } else {
        this->agentGroupGaze.push_back(results[5].double_value);
      }
      // Group Cohesion Weight
      if (results[6].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentGroupCoh.push_back(0.0);
        std::cout<<"Group cohesion weight for " + this->agentNames[i] + " not set, setting it to the default value: 0.0"<<std::endl;
      } else {
        this->agentGroupCoh.push_back(results[6].double_value);
      }
      // Group Repulsion Weight
      if (results[7].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentGroupRep.push_back(0.0);
        std::cout<<"Group repulsion weight for " + this->agentNames[i] + " not set, setting it to the default value: 0.0"<<std::endl;
      } else {
        this->agentGroupRep.push_back(results[7].double_value);
      }
      // Desired Velocity
      if (results[8].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentDesVelocity.push_back(0.9);
        std::cout<<"Desired velocity for " + this->agentNames[i] + " not set, setting it to the default value: 0.9"<<std::endl;
      } else {
        this->agentDesVelocity.push_back(results[8].double_value);
      }
      // Animation Factor
      if (results[9].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentAnimFact.push_back(5.1);
        std::cout<<"Animation factor for " + this->agentNames[i] + " not set, setting it to the default value: 5.1"<<std::endl;
      } else {
        this->agentAnimFact.push_back(results[9].double_value);
      }
      // Animation Name
      if (results[10].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentAnimName.push_back(WALKING_ANIMATION);
        std::cout<<"Animation name for " + this->agentNames[i] + " not set, setting it to the default value: walking"<<std::endl;
      } else {
        this->agentAnimName.push_back(results[10].string_value);
      }
      // People Distance
      if (results[11].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentPeopleDist.push_back(6.0);
        std::cout<<"People distance for " + this->agentNames[i] + " not set, setting it to the default value: 6.0"<<std::endl;
      } else {
        this->agentPeopleDist.push_back(results[11].double_value);
      }
      // Publish Forces Boolean
      if (results[14].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentPubForces.push_back(false);
        std::cout<<"Publish forces for " + this->agentNames[i] + " not set, setting it to the default value: false"<<std::endl;
      } else {
        this->agentPubForces.push_back(results[14].bool_value);
      }
      // Topic and publisher used to publish forces
      this->agentTopicName.push_back("/forces/" + this->agentNames[i]);
      // Obstacles to ignore
      std::vector<std::string> obs;
      for (unsigned int j = 0; j < this->agentNames.size(); ++j){
        obs.push_back(this->agentNames[j]); // All the actor's models must be ignored
      }
      obs.push_back(this->robotName); // Robot must be ignored
      if (results[12].type == rclcpp::PARAMETER_NOT_SET) {
        std::cout<<"No obstacles to ignore for " + this->agentNames[i]<<std::endl;
        this->agentIgnoreObs.push_back(obs);
      } else {
        for (unsigned int j = 0; j <results[12].string_array_value.size(); ++j){
          obs.push_back(results[12].string_array_value[j]);
        }
        this->agentIgnoreObs.push_back(obs);
      }
      obs.clear();
      // Waypoints - THESE MUST BE LOADED
      std::vector<std::tuple<double,double>> goals;
      for (unsigned int j = 0; j <results[13].double_array_value.size(); j+=2){
        goals.push_back(std::make_tuple(results[13].double_array_value[j],results[13].double_array_value[j+1]));
      }
      this->agentGoals.push_back(goals);
      goals.clear();
      // Initial Orientation
      if (results[16].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentInitYaw.push_back(0.0);
        std::cout<<"Initial orientation for " + this->agentNames[i] + " not set, setting it to the default value: 0.0 radians"<<std::endl;
      } else {
        this->agentInitYaw.push_back(results[16].double_value * (M_PI_2 / 90.0));
      }
      // Initial Position
      if (results[17].type == rclcpp::PARAMETER_NOT_SET) {
        this->agentInitPos.push_back(this->agentGoals[i][0]);
        std::cout<<"Initial position for " + this->agentNames[i] + " not set, setting it as the first waypoint: "<<std::get<0>(this->agentGoals[i][0])<<","<<std::get<1>(this->agentGoals[i][0])<<std::endl;
      } else {
        std::tuple<double,double> pos(results[17].double_array_value[0],results[17].double_array_value[1]);
        this->agentInitPos.push_back(pos);
      }
      // Agents in group
      if (results[12].type == rclcpp::PARAMETER_NOT_SET) {
        std::cout<<"No group for " + this->agentNames[i]<<std::endl;
        this->agentGroup.push_back(std::vector<std::string>());
      } else {
        std::vector<std::string> group;
        for (unsigned int j = 0; j <results[15].string_array_value.size(); ++j){
          group.push_back(results[15].string_array_value[j]);
        }
        this->agentGroup.push_back(group);
        group.clear();
      }
      //std::cout<<this->agentNames[i]<<": [mass: "<<this->agentMass[i]<<", radius: "<<this->agentRadius[i]<<", goal_weight: "<<this->agentGoalWeight[i]<<", obstacle_weight: "<<this->agentObstacleWeight[i]<<", social_weight: "<<this->agentSocialWeight[i]<<", group_gaze: "<<this->agentGroupGaze[i]<<", group_coh: "<<this->agentGroupCoh[i]<<", group_rep: "<<this->agentGroupRep[i]<<", des_velocity: "<<this->agentDesVelocity[i]<<", anim_fact: "<<this->agentAnimFact[i]<<", anim_name: "<<this->agentAnimName[i]<<", people_dist: "<<this->agentPeopleDist[i]<<", first_ignored_obs: "<<this->agentIgnoreObs[i][0]<<", first_goal: ("<<std::get<0>(this->agentGoals[i][0])<<", "<<std::get<1>(this->agentGoals[i][0])<<") "<<", publish_forces: "<<this->agentPubForces[i]<<", topic_name: "<<this->agentTopicName[i]<<", first_agent_in_group: "<<this->agentGroup[i][0]<<", initial_orientation: "<<this->agentInitYaw[i]<<", initial_position: "<<std::get<0>(this->agentInitPos[i])<<","<<std::get<1>(this->agentInitPos[i])<<"]"<<std::endl;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_parameters()");
    }
  }
}

////////////////////////////////////////////////
void WorldSFMPlugin::InitializeActors() {
  for (unsigned int i = 0; i < this->actors.size(); ++i) {
    ignition::math::Pose3d actorPose = this->actors[i]->WorldPose();

    // TODO: REVIEW THIS PART - Set the initial pose of each actor
    actorPose.Pos().X(std::get<0>(this->agentInitPos[i]));
    actorPose.Pos().Y(std::get<1>(this->agentInitPos[i]));
    actorPose.Pos().Z(0.91 - this->groundHeight);
    actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, this->agentInitYaw[i] + M_PI_2);
    this->actors[i]->SetWorldPose(actorPose, false, false);

    // Initialize the SFM actors
    ignition::math::Pose3d pose = this->actors[i]->WorldPose();
    ignition::math::Vector3d linvel = this->actors[i]->WorldLinearVel();
    ignition::math::Vector3d angvel = this->actors[i]->WorldAngularVel();
    sfm::Agent agent;
    agent.id = this->actors[i]->GetId();
    agent.position.set(pose.Pos().X(),pose.Pos().Y());
    agent.yaw = pose.Rot().Euler().Z();
    agent.velocity.set(linvel.X(),linvel.Y());
    agent.linearVelocity = linvel.Length();
    agent.angularVelocity = angvel.Z();
    agent.desiredVelocity = this->agentDesVelocity[i];
    agent.params.forceFactorDesired = this->agentGoalWeight[i];
    agent.params.forceFactorObstacle = this->agentObstacleWeight[i];
    agent.params.forceFactorSocial = this->agentSocialWeight[i];
    agent.params.forceFactorGroupGaze = this->agentGroupGaze[i];
    agent.params.forceFactorGroupCoherence = this->agentGroupCoh[i];
    agent.params.forceFactorGroupRepulsion = this->agentGroupRep[i];
    agent.radius = this->agentRadius[i];
    if (this->agentGroup[i].size() > 0) {
      agent.groupId = agent.id;
    } else {
      agent.groupId = -1;
    }
    for (unsigned int j = 0; j < this->agentGoals[i].size(); ++j) {
      sfm::Goal sfmGoal;
      sfmGoal.center.set(std::get<0>(this->agentGoals[i][j]),std::get<1>(this->agentGoals[i][j]));
      sfmGoal.radius = 0.3;
      agent.cyclicGoals = true;
      agent.goals.push_back(sfmGoal);
    }
    this->sfmActors.push_back(agent);
  }
  // Attach a static cylinder for Laser detection (same size as actor radius)
  if (this->attachCollisionToActors) CreateModelForActors();
}

////////////////////////////////////////////////
void WorldSFMPlugin::CreateModelForActors() {
  for (unsigned int i = 0; i < this->agentModel.size(); ++i) {

    std::string modelName = this->agentNames[i] + "_collision_model";
    this->actorCollisionNames.push_back(modelName);

    std::string modelString(
      "<sdf version ='1.6'>\
        <model name='" + modelName + "'>\
        <allow_auto_disable>false</allow_auto_disable>\
          <pose>0 0 -100 0 0 0</pose>\
          <static>true</static>\
          <link name='link'>\
            <collision name='link'>\
              <pose>0 0 0 -1.5708 0 0</pose>\
              <geometry>\
                <cylinder>\
                  <radius>" + std::to_string(this->agentRadius[i]) + "</radius>\
                  <length>1.8</length>\
                </cylinder>\
              </geometry>\
            </collision>\
          </link>\
        </model>\
      </sdf>");

    this->world->InsertModelString(modelString);
  }
}

////////////////////////////////////////////////
void WorldSFMPlugin::InitializeRobot() {
  this->robotModel = this->world->ModelByName(this->robotName);
  this->robotModel->SetWorld(this->world);
  //std::cout<<"Robot name: "<<this->robotModel->GetName()<<std::endl;
  //std::cout<<"robot_first_goal: "<<std::get<0>(this->robotGoals[0])<<","<<std::get<1>(this->robotGoals[0])<<", robot_initial_position: "<<std::get<0>(this->robotInitPos)<<","<<std::get<1>(this->robotInitPos)<<", robot_initial_yaw: "<<this->robotInitYaw<<", robot_mass: "<<this->robotMass<<", robot_radius: "<<this->robotRadius<<", robot_velocity: "<<this->robotVelocity<<", robot_goal_weight: "<<this->robotGoalWeight<<", robot_obstacle_weight: "<<this->robotObstacleWeight<<", robot_social_weight: "<<this->robotSocialWeight<<", first_ignored_obstacle: "<<this->robotIgnoreObs[0]<<std::endl;

  ignition::math::Pose3d robotPose = this->robotModel->WorldPose();

  // TODO: REVIEW THIS PART - Set the initial pose of each actor
  robotPose.Pos().X(std::get<0>(this->robotInitPos));
  robotPose.Pos().Y(std::get<1>(this->robotInitPos));
  robotPose.Pos().Z(this->groundHeight);
  robotPose.Rot() = ignition::math::Quaterniond(1.5707, 0, this->robotInitYaw);
  this->robotModel->SetWorldPose(robotPose, true, false);

  // Initialize the robot as an SFM actor
  ignition::math::Pose3d pose = this->robotModel->WorldPose();
  ignition::math::Vector3d linvel = this->robotModel->WorldLinearVel();
  ignition::math::Vector3d angvel = this->robotModel->WorldAngularVel();
  sfm::Agent agent;
  agent.id = this->robotModel->GetId();
  agent.position.set(pose.Pos().X(),pose.Pos().Y());
  agent.yaw = pose.Rot().Euler().Z();
  agent.velocity.set(linvel.X(),linvel.Y());
  agent.linearVelocity = linvel.Length();
  agent.angularVelocity = angvel.Z();
  agent.desiredVelocity = this->robotVelocity;
  agent.params.forceFactorDesired = this->robotGoalWeight;
  agent.params.forceFactorObstacle = this->robotObstacleWeight;
  agent.params.forceFactorSocial = this->robotSocialWeight;
  agent.params.forceFactorGroupGaze = 0.0;
  agent.params.forceFactorGroupCoherence = 0.0;
  agent.params.forceFactorGroupRepulsion = 0.0;
  agent.radius = this->robotRadius;
  agent.groupId = -1;
  // Initialize Goals 
  for (unsigned int j = 0; j < this->robotGoals.size(); ++j) {
    sfm::Goal sfmGoal;
    sfmGoal.center.set(std::get<0>(this->robotGoals[j]),std::get<1>(this->robotGoals[j]));
    sfmGoal.radius = 0.3;
    agent.cyclicGoals = true;
    agent.goals.push_back(sfmGoal);
  }
  this->sfmRobot = agent;

  // Generate SFM vector to use for algorithm
  this->sfmEntities = this->sfmActors;
  this->sfmEntities.push_back(this->sfmRobot);
  // Generate models vector to use for algorithm
  this->entitiesModel = this->agentModel;
  this->entitiesModel.push_back(this->robotModel);
  // Generate Ignored Obs vector to use for algorithm
  this->entitiesIgnoreObs = this->agentIgnoreObs;
  this->entitiesIgnoreObs.push_back(this->robotIgnoreObs);

  // for (unsigned int i = 0; i < this->sfmEntities.size(); ++i) {
  //   std::cout<<"SFM Entity ID: "<<this->sfmEntities[i].id<<std::endl;
  //   std::cout<<"Model Entity name: "<<this->entitiesModel[i]->GetName()<<std::endl;
  // }

  // Get the robot laser
  this->laserSensor = sensors::SensorManager::Instance()->GetSensor(this->laserName);
  this->laserNode = boost::make_shared<gazebo::transport::Node>();
  this->laserNode->Init(this->world->Name());
  this->laserSub = this->laserNode->Subscribe(this->laserSensor->Topic(), &WorldSFMPlugin::LaserCallback, this);
  //std::cout<<"Laser sensor name: "<<this->laserSensor->Name()<<", Laser sensor type: "<<this->laserSensor->Type()<<", Laser topic: "<<this->laserSensor->Topic()<<std::endl;
}

////////////////////////////////////////////////
void WorldSFMPlugin::LaserCallback(ConstLaserScanStampedPtr &msg) {
  // Erase previous scans
  this->laserRanges.clear();
  this->laserObs.clear();
  // Save laser scans below 2m and their angle
  auto ls = msg->scan();
  double min_angle = ls.angle_min();
  double step = ls.angle_step();
  for (int i = 0; i < ls.ranges().size(); ++i) {
    if (ls.ranges(i) < 2.0) {    
      double range_angle = min_angle + i * step;
      double yaw = this->robotModel->WorldPose().Rot().Euler().Z();

      double angle = yaw + range_angle;
      if (angle > M_PI) {
        angle -= (2 * M_PI);
      }
      if (angle < -M_PI) {
        angle += (2 * M_PI);
      }
      // angle = angle * (180 / M_PI);
      // std::cout<<"Angle of laser"<<i<<": "<<angle<<std::endl;
      // std::cout<<"Distance of laser"<<i<<": "<<ls.ranges(i)<<std::endl;

      std::tuple<double,double> range(ls.ranges(i), angle); // Polar coordinates
      // TODO: Compute obstacle based on the position of the laser with respect to the pose of the robot
      ignition::math::Vector2d obs(ls.ranges(i) * std::cos(angle) + this->robotModel->WorldPose().Pos().X(), ls.ranges(i) * std::sin(angle) + this->robotModel->WorldPose().Pos().Y()); // Cartesian coordinates

      this->laserRanges.push_back(range); 
      this->laserObs.push_back(obs);
    }
  }
  // // Print closest obstacle based on Laser ranges
  // ignition::math::Vector2d min_obs;
  // double min_dist = 2;
  // double angle;
  // for (unsigned int i = 0; i < this->laserObs.size(); ++i) {
  //   if (std::get<0>(this->laserRanges[i]) < min_dist) {
  //     min_obs = this->laserObs[i];
  //     min_dist = std::get<0>(this->laserRanges[i]);
  //     angle = std::get<1>(this->laserRanges[i]);
  //   }
  //   if (i == this->laserObs.size() - 1) {
  //     std::cout<<"Closest obs : "<<min_obs.X()<<","<<min_obs.Y()<<std::endl;
  //     std::cout<<"Min dist: "<<min_dist<<std::endl;
  //     std::cout<<"Angle with respect to robot: "<<angle<<std::endl;
  //   }
  // }
}

////////////////////////////////////////////////
void WorldSFMPlugin::PublishForces() {
  if (!this->controlRobot) {
    for (unsigned int i = 0; i < this->actors.size(); ++i) {
      if (this->agentPubForces[i] == true) {
        world_sfm_hsfm_plugins::msg::Forces msg = world_sfm_hsfm_plugins::msg::Forces();
        // Global force
        msg.global_force.x = this->sfmActors[i].forces.globalForce.getX();
        msg.global_force.y = this->sfmActors[i].forces.globalForce.getY();
        // Desired force
        msg.desired_force.x = this->sfmActors[i].forces.desiredForce.getX();
        msg.desired_force.y = this->sfmActors[i].forces.desiredForce.getY();
        // Obstacle force
        msg.obstacle_force.x = this->sfmActors[i].forces.obstacleForce.getX();
        msg.obstacle_force.y = this->sfmActors[i].forces.obstacleForce.getY();
        // Social force
        msg.social_force.x = this->sfmActors[i].forces.socialForce.getX();
        msg.social_force.y = this->sfmActors[i].forces.socialForce.getY();
        // Group force
        msg.group_force.x = this->sfmActors[i].forces.groupForce.getX();
        msg.group_force.y = this->sfmActors[i].forces.groupForce.getY();
        // // Torque force - not implemented
        // msg.torque_force = 0.0;
        // Linear velocity
        msg.linear_velocity.x = this->sfmActors[i].velocity.getX();
        msg.linear_velocity.y = this->sfmActors[i].velocity.getY();
        // // Angular velocity - not implemented
        // msg.angular_velocity = 0.0;
        // Pose
        msg.pose.x = this->sfmActors[i].initPosition.getX();
        msg.pose.y = this->sfmActors[i].initPosition.getY();
        msg.pose.theta = this->sfmActors[i].initYaw.toDegree();

        this->actorForcesPub[i]->publish(msg);
      }
    }
  } else {
    for (unsigned int i = 0; i < this->actors.size(); ++i) {
      if (this->agentPubForces[i] == true) {
        world_sfm_hsfm_plugins::msg::Forces msg = world_sfm_hsfm_plugins::msg::Forces();
        // Global force
        msg.global_force.x = this->sfmEntities[i].forces.globalForce.getX();
        msg.global_force.y = this->sfmEntities[i].forces.globalForce.getY();
        // Desired force
        msg.desired_force.x = this->sfmEntities[i].forces.desiredForce.getX();
        msg.desired_force.y = this->sfmEntities[i].forces.desiredForce.getY();
        // Obstacle force
        msg.obstacle_force.x = this->sfmEntities[i].forces.obstacleForce.getX();
        msg.obstacle_force.y = this->sfmEntities[i].forces.obstacleForce.getY();
        // Social force
        msg.social_force.x = this->sfmEntities[i].forces.socialForce.getX();
        msg.social_force.y = this->sfmEntities[i].forces.socialForce.getY();
        // Group force
        msg.group_force.x = this->sfmEntities[i].forces.groupForce.getX();
        msg.group_force.y = this->sfmEntities[i].forces.groupForce.getY();
        // // Torque force - not implemented
        // msg.torque_force = 0.0;
        // Linear velocity
        msg.linear_velocity.x = this->sfmEntities[i].velocity.getX();
        msg.linear_velocity.y = this->sfmEntities[i].velocity.getY();
        // // Angular velocity - not implemented
        // msg.angular_velocity = 0.0;
        // Pose
        msg.pose.x = this->sfmEntities[i].initPosition.getX();
        msg.pose.y = this->sfmEntities[i].initPosition.getY();
        msg.pose.theta = this->sfmEntities[i].initYaw.toDegree();

        this->actorForcesPub[i]->publish(msg);
      }
    }
  }
}