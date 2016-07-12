/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <functional>
#include <cstdio>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include "ProximityRayPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(ProximityRayPlugin)

/////////////////////////////////////////////////
ProximityRayPlugin::ProximityRayPlugin()
{
}

/////////////////////////////////////////////////
ProximityRayPlugin::~ProximityRayPlugin()
{
    this->newLaserScansConnection.reset();

    this->parentSensor.reset();
    this->world.reset();
}

//////////////////////////////////////////////////
std::string ProximityRayPlugin::Topic(std::string topicName) const
{
  std::string globalTopicName = "~/";
  globalTopicName += this->parentSensor->Name() + "/" + this->GetHandle() + topicName;
  boost::replace_all(globalTopicName, "::", "/");

  return globalTopicName;
}

/////////////////////////////////////////////////
void ProximityRayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    // Get the name of the parent sensor
    this->parentSensor =
        std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

    if (!this->parentSensor)
        gzthrow("ProximityRayPlugin requires a Ray Sensor as its parent");

    std::string worldName = this->parentSensor->WorldName();
    this->world = physics::get_world(worldName);
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(worldName);

    std::string interruptionTopic;
    if (_sdf->HasElement("output_state_topic"))
    {
        interruptionTopic = _sdf->Get<std::string>("output_state_topic");
    }
    else {
        interruptionTopic = this->Topic("interruption_state");
    }
    this->interruptionPub =
        this->node->Advertise<msgs::Header>(interruptionTopic, 50);

    std::string interruptionChangeTopic;
    if (_sdf->HasElement("output_change_topic"))
    {
        interruptionChangeTopic = _sdf->Get<std::string>("output_change_topic");
    }
    else {
        interruptionChangeTopic = this->Topic("interruption_change");
    }
    this->interruptionChangePub =
        this->node->Advertise<msgs::Header>(interruptionChangeTopic, 50);

    this->newLaserScansConnection =
        this->parentSensor->LaserShape()->ConnectNewLaserScans(
            std::bind(&ProximityRayPlugin::OnNewLaserScans, this));
}

/////////////////////////////////////////////////
void ProximityRayPlugin::OnNewLaserScans()
{
    // Prevent new scans from arriving while we're processing this one
    this->parentSensor->SetActive(false);

    double maxRange = this->parentSensor->RangeMax();
    double minRange = this->parentSensor->RangeMin();
    int rangeCount = this->parentSensor->RangeCount();
    std::vector<double> ranges;
    this->parentSensor->Ranges(ranges);

    bool objectDetected = false;

    for (int i = 0; i<ranges.size(); i++){
        double range = ranges[i];
        if (range < maxRange and range > minRange) {
            objectDetected = true;
        }
    }

    std::lock_guard<std::mutex> lock(this->mutex);
    msgs::Set(this->interruptionMsg.mutable_stamp(), this->world->GetSimTime());
    this->interruptionMsg.set_index(objectDetected);
    if (this->interruptionPub && this->interruptionPub->HasConnections()) {
        this->interruptionPub->Publish(this->interruptionMsg);
    }

    if (objectDetected) {
        if (!this->interrupted) {
            if (this->interruptionChangePub && this->interruptionChangePub->HasConnections()) {
                this->interruptionChangePub->Publish(this->interruptionMsg);
            }
        }
        this->interrupted = true;
    } else {
        if (this->interrupted) {
            if (this->interruptionChangePub && this->interruptionChangePub->HasConnections()) {
                this->interruptionChangePub->Publish(this->interruptionMsg);
            }
        }
        this->interrupted = false;
    }
    this->parentSensor->SetActive(true); // this seems to happen automatically, not sure if a bug
}


