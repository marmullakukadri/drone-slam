#include "drone_bringup/drone_model.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace drone_bringup
{

namespace
{
using visualization_msgs::msg::Marker;

constexpr double kStaticPeriod = 1.0;
constexpr double kPropPeriod = 1.0 / 30.0;
constexpr double kGhostPeriod = 1.0 / 15.0;
constexpr double kMaxShownSpin = 15.0;
constexpr double kLabelHeight = 0.6;

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

const auto kGhostColor = rgba(0.2f, 0.9f, 1.0f, 0.5f);
const auto kGhostLabelColor = rgba(0.2f, 0.9f, 1.0f, 1.0f);
const auto kSlamColor = rgba(1.0f, 0.55f, 0.1f, 1.0f);

std::vector<double> numbers(const char * text)
{
  std::vector<double> v;
  if (!text) {return v;}
  std::istringstream in(text);
  for (double d; in >> d; ) {v.push_back(d);}
  return v;
}

std::array<double, 6> pose6(const char * text)
{
  std::array<double, 6> p{};
  const auto v = numbers(text);
  std::copy_n(v.begin(), std::min<size_t>(6, v.size()), p.begin());
  return p;
}

const char * childText(const tinyxml2::XMLElement * e, std::initializer_list<const char *> path)
{
  for (const char * name : path) {
    if (!e) {return nullptr;}
    e = e->FirstChildElement(name);
  }
  return e ? e->GetText() : nullptr;
}

tf2::Quaternion rpy(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return q;
}

std::optional<Marker> visualMarker(const tinyxml2::XMLElement * vis)
{
  const auto * geo = vis->FirstChildElement("geometry");
  if (!geo) {return std::nullopt;}
  Marker m;
  if (geo->FirstChildElement("box")) {
    const auto s = numbers(childText(geo, {"box", "size"}));
    if (s.size() < 3) {return std::nullopt;}
    m.type = Marker::CUBE;
    m.scale.x = s[0];
    m.scale.y = s[1];
    m.scale.z = s[2];
  } else if (geo->FirstChildElement("cylinder")) {
    const double r = std::stod(childText(geo, {"cylinder", "radius"}));
    m.type = Marker::CYLINDER;
    m.scale.x = m.scale.y = 2.0 * r;
    m.scale.z = std::stod(childText(geo, {"cylinder", "length"}));
  } else if (geo->FirstChildElement("sphere")) {
    m.type = Marker::SPHERE;
    m.scale.x = m.scale.y = m.scale.z = 2.0 * std::stod(childText(geo, {"sphere", "radius"}));
  } else {
    return std::nullopt;
  }
  const char * rgb = childText(vis, {"material", "diffuse"});
  if (!rgb) {rgb = childText(vis, {"material", "ambient"});}
  auto c = numbers(rgb ? rgb : "0.5 0.5 0.5");
  c.resize(3, 0.5);
  const char * tr = childText(vis, {"transparency"});
  m.color = rgba(c[0], c[1], c[2], 1.0f - (tr ? std::stof(tr) : 0.0f));
  return m;
}
}

DroneModel::DroneModel()
: Node("drone_model")
{
  const auto ws = declare_parameter("workspace", std::string("/workspace"));
  const auto sdf = declare_parameter("model_file", ws + "/models/slam_drone/model.sdf");
  frame_ = declare_parameter("frame_id", std::string("base_link"));
  load(sdf);
  RCLCPP_INFO(
    get_logger(), "Drone model from %s: %zu parts + %zu propeller parts", sdf.c_str(),
    static_.size(), props_.size());

  const auto qos = rclcpp::QoS(1).transient_local();
  pub_ = create_publisher<MarkerArray>("drone_model", qos);
  gt_pub_ = create_publisher<MarkerArray>("drone_model_gt", qos);
  speed_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/drone/prop_speed", 10, [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
      spin_speed_ = std::clamp(msg->data, -kMaxShownSpin, kMaxShownSpin);
    });
  gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom_gt", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {gt_ = msg;});
  static_timer_ = create_timer(
    std::chrono::duration<double>(kStaticPeriod), [this]() {publishStatic();});
  prop_timer_ = create_timer(
    std::chrono::duration<double>(kPropPeriod), [this]() {publishProps();});
  ghost_timer_ = create_timer(
    std::chrono::duration<double>(kGhostPeriod), [this]() {publishGhost();});
  label_.type = Marker::TEXT_VIEW_FACING;
  label_.scale.z = kLabelHeight;
  publishStatic();
}

void DroneModel::load(const std::string & path)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Cannot read " + path);
  }
  const auto * root = doc.FirstChildElement("sdf");
  const auto * model = root ? root->FirstChildElement("model") : nullptr;
  if (!model) {throw std::runtime_error("No <model> in " + path);}
  std::map<std::string, const tinyxml2::XMLElement *> joints;
  for (auto * j = model->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) {
    if (const char * child = childText(j, {"child"})) {joints[child] = j;}
  }
  for (auto * link = model->FirstChildElement("link"); link;
    link = link->NextSiblingElement("link"))
  {
    const std::string name = link->Attribute("name") ? link->Attribute("name") : "";
    for (auto * vis = link->FirstChildElement("visual"); vis;
      vis = vis->NextSiblingElement("visual"))
    {
      auto m = visualMarker(vis);
      if (!m) {continue;}
      const auto vpose = pose6(childText(vis, {"pose"}));
      const auto j = joints.find(name);
      if (j != joints.end()) {
        const auto axis = numbers(childText(j->second, {"axis", "xyz"}));
        const double sign = (axis.size() < 3 || axis[2] >= 0.0) ? 1.0 : -1.0;
        props_.push_back({*m, pose6(childText(link, {"pose"})), vpose, sign});
      } else {
        static_.push_back({*m, vpose});
      }
    }
  }
}

std::vector<DroneModel::Part> DroneModel::bodyParts() const
{
  std::vector<Part> parts;
  for (size_t i = 0; i < static_.size(); ++i) {
    const auto & p = static_[i].pose;
    parts.push_back(
      {&static_[i].m, "body", static_cast<int>(i), tf2::Vector3(p[0], p[1], p[2]),
        rpy(p[3], p[4], p[5])});
  }
  return parts;
}

std::vector<DroneModel::Part> DroneModel::propParts() const
{
  std::vector<Part> parts;
  for (size_t i = 0; i < props_.size(); ++i) {
    const auto & pr = props_[i];
    const double yaw = pr.link[5] + pr.sign * angle_;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const tf2::Vector3 xyz(
      pr.link[0] + c * pr.vis[0] - s * pr.vis[1],
      pr.link[1] + s * pr.vis[0] + c * pr.vis[1],
      pr.link[2] + pr.vis[2]);
    parts.push_back(
      {&pr.m, "propellers", static_cast<int>(i), xyz, rpy(pr.vis[3], pr.vis[4], yaw + pr.vis[5])});
  }
  return parts;
}

DroneModel::Marker DroneModel::marker(
  const Marker & tmpl, const std::string & ns, int id, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp, const tf2::Vector3 & xyz,
  const tf2::Quaternion & q, const std_msgs::msg::ColorRGBA * color) const
{
  Marker m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.action = Marker::ADD;
  m.type = tmpl.type;
  m.scale = tmpl.scale;
  m.text = tmpl.text;
  m.color = color ? *color : tmpl.color;
  m.frame_locked = true;
  m.pose.position.x = xyz.x();
  m.pose.position.y = xyz.y();
  m.pose.position.z = xyz.z();
  m.pose.orientation.x = q.x();
  m.pose.orientation.y = q.y();
  m.pose.orientation.z = q.z();
  m.pose.orientation.w = q.w();
  return m;
}

void DroneModel::publishStatic()
{
  const auto stamp = now();
  MarkerArray out;
  for (const auto & p : bodyParts()) {
    out.markers.push_back(marker(*p.tmpl, p.ns, p.id, frame_, stamp, p.xyz, p.q));
  }
  label_.text = "SLAM";
  out.markers.push_back(
    marker(
      label_, "label", 0, frame_, stamp, tf2::Vector3(0, 0, 0.8),
      tf2::Quaternion::getIdentity(), &kSlamColor));
  pub_->publish(out);
}

void DroneModel::publishProps()
{
  angle_ = std::fmod(angle_ + spin_speed_ * kPropPeriod, 2.0 * M_PI);
  if (angle_ < 0.0) {angle_ += 2.0 * M_PI;}
  const auto stamp = now();
  MarkerArray out;
  for (const auto & p : propParts()) {
    out.markers.push_back(marker(*p.tmpl, p.ns, p.id, frame_, stamp, p.xyz, p.q));
  }
  pub_->publish(out);
}

void DroneModel::publishGhost()
{
  if (!gt_) {return;}
  const auto & p = gt_->pose.pose.position;
  const auto & o = gt_->pose.pose.orientation;
  const tf2::Quaternion qd(o.x, o.y, o.z, o.w);
  const tf2::Vector3 pos(p.x, p.y, p.z);
  const auto & frame = gt_->header.frame_id;
  const auto & stamp = gt_->header.stamp;
  auto parts = bodyParts();
  const auto props = propParts();
  parts.insert(parts.end(), props.begin(), props.end());
  MarkerArray out;
  for (const auto & part : parts) {
    out.markers.push_back(
      marker(
        *part.tmpl, "ghost_" + part.ns, part.id, frame, stamp,
        pos + tf2::quatRotate(qd, part.xyz), qd * part.q, &kGhostColor));
  }
  label_.text = "Drone (Gazebo)";
  out.markers.push_back(
    marker(
      label_, "ghost_label", 0, frame, stamp, tf2::Vector3(p.x, p.y, 4.6),
      tf2::Quaternion::getIdentity(), &kGhostLabelColor));
  gt_pub_->publish(out);
}

}
