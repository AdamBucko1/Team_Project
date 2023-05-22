#include "include/tp_local_odom/pose_republisher.hpp"
PoseRepublisher::PoseRepublisher(): Node("local_odom_node")
{
     this->Init();
}
void PoseRepublisher::Init()
{
    this->DefaultValues();
    this->SetupSubscribers();
    this->SetupPublishers();
    this->SetupServices();
}
void PoseRepublisher::DefaultValues()
{

}
void PoseRepublisher::SetupSubscribers()
{

}
void PoseRepublisher::SetupPublishers()
{

}
void PoseRepublisher::SetupServices()
{

}


