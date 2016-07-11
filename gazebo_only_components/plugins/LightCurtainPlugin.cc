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
#include "LightCurtainPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(LightCurtainPlugin)

/////////////////////////////////////////////////
LightCurtainPlugin::LightCurtainPlugin()
{
}

/////////////////////////////////////////////////
LightCurtainPlugin::~LightCurtainPlugin()
{
    this->newLaserScansConnection.reset();

    this->parentSensor.reset();
    this->world.reset();
}

//////////////////////////////////////////////////
std::string LightCurtainPlugin::Topic() const
{
  std::string topicName = "~/";
  topicName += this->parentSensor->Name() + "/" + this->GetHandle() + "/interruption";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

/////////////////////////////////////////////////
void LightCurtainPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    // Get the name of the parent sensor
    this->parentSensor =
        std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

    if (!this->parentSensor)
        gzthrow("LightCurtainPlugin requires a Ray Sensor as its parent");

    std::string interruptionTopic;
    if (_sdf->HasElement("output_topic"))
    {
        interruptionTopic = _sdf->Get<std::string>("output_topic");
    }
    else {
        interruptionTopic = this->Topic();
    }
    std::cout << interruptionTopic << std::endl;

    std::string worldName = this->parentSensor->WorldName();
    this->world = physics::get_world(worldName);
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(worldName);

    this->newLaserScansConnection =
        this->parentSensor->LaserShape()->ConnectNewLaserScans(
            std::bind(&LightCurtainPlugin::OnNewLaserScans, this));

    this->interruptionPub =
        this->node->Advertise<msgs::Header>(interruptionTopic, 50);
}

/////////////////////////////////////////////////
void LightCurtainPlugin::OnNewLaserScans()
{
    // Prevent new scans from arriving while we're processing this one
    this->parentSensor->SetActive(false);

    double maxRange = this->parentSensor->RangeMax();
    double minRange = this->parentSensor->RangeMin();
    int rangeCount = this->parentSensor->RangeCount();
    std::vector<double> ranges;
    this->parentSensor->Ranges(ranges);

    bool objectDetected = false;

    std::cout << "Laser ranges:" << std::endl;
    for (int i = 0; i<ranges.size(); i++){
        double range = ranges[i];
        std::cout << range << " ";
        if (range < maxRange and range > minRange) {
            objectDetected = true;
        }
    }
    std::cout << std::endl;

    std::lock_guard<std::mutex> lock(this->mutex);
    msgs::Set(this->interruptionMsg.mutable_stamp(), this->world->GetSimTime());
    this->interruptionMsg.set_index(objectDetected);

    if (objectDetected) {
        std::cout << "Laser beam interrupted" << std::endl;
        if (!this->interrupted) {
            if (this->interruptionPub && this->interruptionPub->HasConnections()) {
                this->interruptionPub->Publish(this->interruptionMsg);
            }
        }
        this->interrupted = true;
    } else {
        std::cout << "nothing" << std::endl;
        if (this->interrupted) {
            if (this->interruptionPub && this->interruptionPub->HasConnections()) {
                this->interruptionPub->Publish(this->interruptionMsg);
            }
        }
        this->interrupted = false;
    }
    this->parentSensor->SetActive(true); // this seems to happen automatically, not sure if a bug
}


