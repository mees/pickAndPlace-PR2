#include <planning_scene_manager/planning_scene_manager.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <eigen_conversions/eigen_msg.h>

// http://docs.ros.org/indigo/api/moveit_msgs/html/msg/PlanningSceneComponents.html
// http://docs.ros.org/api/moveit_msgs/html/msg/PlanningScene.html

namespace planning_scene_manager
{

PlanningSceneManager::PlanningSceneManager()
{
  client_get_scene_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  client_apply_scene_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
}

void PlanningSceneManager::allowCollision(const std::string& id)
{
  moveit_msgs::AllowedCollisionMatrix acm;
  getAllowedCollisionMatrix(acm);

  if (find(acm.entry_names, id) >= acm.entry_names.size())
  {
    expandAllowedCollisionMatrix(acm, id);

    acm.default_entry_names.push_back(id);
    acm.default_entry_values.push_back(true);
  }
  else
  {
    size_t index = find(acm.default_entry_names, id);

    if (index >= acm.default_entry_names.size())
    {
      acm.default_entry_names.push_back(id);
      acm.default_entry_values.push_back(true);
    }
    else
      acm.default_entry_values.at(index) = true;
  }

  moveit_msgs::PlanningScene scene;
  scene.allowed_collision_matrix = acm;

  applyPlanningScene(scene);
}

bool PlanningSceneManager::getCollisionObject(const std::string& object_id,
  moveit_msgs::CollisionObject& object)
{
  std::vector<moveit_msgs::CollisionObject> objects;
  getCollisionObjects(objects);

  for (size_t i = 0; i < objects.size(); i++)
  {
    if (objects[i].id == object_id)
    {
      object = objects[i];
      return true;
    }
  }

  ROS_WARN("[%s::%s]: Could not find '%s' in planning scene.", ns().c_str(), name().c_str(), object_id.c_str());
  return false;
}

void PlanningSceneManager::addBoxCollisionObject(const std::string& frame_id,
  const std::string& object_id, const Eigen::Affine3d& pose, const Eigen::Vector3d& dimensions)
{
  moveit_msgs::CollisionObject object;
  initHeader(ros::Time::now(), frame_id, object.header);
  object.id = object_id;

  object.primitives.resize(1);
  std::vector<double> dims(dimensions.data(), dimensions.data() + dimensions.size());
  initSolidPrimitive(shape_msgs::SolidPrimitive::BOX, dims, object.primitives[0]);

  object.primitive_poses.resize(1);
  tf::poseEigenToMsg(pose, object.primitive_poses[0]);

  addCollisionObject(object);
}

void PlanningSceneManager::addSphereCollisionObject(const std::string& frame_id,
  const std::string& object_id, const Eigen::Affine3d& pose, double radius)
{
  moveit_msgs::CollisionObject object;
  initHeader(ros::Time::now(), frame_id, object.header);
  object.id = object_id;

  object.primitives.resize(1);
  std::vector<double> dims;
  dims.push_back(radius);
  initSolidPrimitive(shape_msgs::SolidPrimitive::SPHERE, dims, object.primitives[0]);

  object.primitive_poses.resize(1);
  tf::poseEigenToMsg(pose, object.primitive_poses[0]);

  addCollisionObject(object);
}

void PlanningSceneManager::removeCollisionObject(const std::string& frame_id,
  const std::string& object_id)
{
  moveit_msgs::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = object_id;
  object.operation = moveit_msgs::CollisionObject::REMOVE;

  addCollisionObject(object);
}

bool PlanningSceneManager::getAllowedCollisionMatrix(moveit_msgs::AllowedCollisionMatrix& acm)
{
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  bool result = client_get_scene_.call(srv);
  acm = srv.response.scene.allowed_collision_matrix;

  return result;
}

bool PlanningSceneManager::getCollisionObjects(std::vector<moveit_msgs::CollisionObject>& objects)
{
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
  bool result = client_get_scene_.call(srv);
  objects = srv.response.scene.world.collision_objects;

  return result;
}

void PlanningSceneManager::applyPlanningScene(const moveit_msgs::PlanningScene& scene)
{
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = scene;
  srv.request.scene.is_diff = true;

  client_apply_scene_.call(srv);
}

void PlanningSceneManager::expandAllowedCollisionMatrix(moveit_msgs::AllowedCollisionMatrix& acm,
  const std::string& name, bool value)
{
  for (size_t i = 0; i < acm.entry_names.size(); i++)
    acm.entry_values[i].enabled.push_back(value);

  acm.entry_names.push_back(name);

  moveit_msgs::AllowedCollisionEntry entry;
  entry.enabled.assign(acm.entry_names.size(), value);
  acm.entry_values.push_back(entry);
}

void PlanningSceneManager::addCollisionObject(const moveit_msgs::CollisionObject& object)
{
  moveit_msgs::PlanningScene scene;
  scene.world.collision_objects.push_back(object);

  applyPlanningScene(scene);
}

void PlanningSceneManager::initHeader(const ros::Time& stamp, const std::string& frame_id,
  std_msgs::Header& header)
{
  header.stamp = stamp;
  header.frame_id = frame_id;
}

void PlanningSceneManager::initSolidPrimitive(const int& type,
  const std::vector<double>& dimensions, shape_msgs::SolidPrimitive& primitive)
{
  primitive.type = type;
  primitive.dimensions = dimensions;
}

size_t PlanningSceneManager::find(const std::vector<std::string>& vector, const std::string& string)
{
  size_t index = std::find(vector.begin(), vector.end(), string) - vector.begin();

  return index;
}

}
