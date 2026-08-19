#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/wrench.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <wind_msgs/msg/wind_cmd.hpp>   // ← notre message custom

#include <random>
#include <mutex>

namespace wind_plugin {

class WindPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:

  void Configure(
      const gz::sim::Entity & _entity,
      const std::shared_ptr<const sdf::Element> & _sdf,
      gz::sim::EntityComponentManager & _ecm,
      gz::sim::EventManager &) override
  {
    model_    = gz::sim::Model(_entity);
    linkName_ = _sdf->Get<std::string>("link_name", "base_link").first;

    // Paramètres initiaux depuis SDF
    forceMean_       = _sdf->Get<gz::math::Vector3d>("wind_mean",
                           gz::math::Vector3d(0,0,0)).first;
    forceVariance_   = _sdf->Get<double>("wind_variance",    0.0).first;
    torqueMean_      = _sdf->Get<gz::math::Vector3d>("torque_mean",
                           gz::math::Vector3d(0,0,0)).first;
    torqueVariance_  = _sdf->Get<double>("torque_variance",  0.0).first;
    gustForceMag_    = _sdf->Get<double>("gust_magnitude",   0.0).first;
    gustTorqueMag_   = _sdf->Get<double>("gust_torque_magnitude", 0.0).first;
    gustFrequency_   = _sdf->Get<double>("gust_frequency",   0.1).first;

    // Paramètres Dryden
    double altitude = _sdf->Get<double>("altitude", 100.0).first;  // m

    // Longueur de turbulence selon altitude (MIL-SPEC)
    double L_horiz = altitude < 1000.0 ? altitude / 2.0 : 500.0;
    double L_vert  = altitude < 1000.0 ? altitude / 2.0 : 500.0;

    double sigma_w = _sdf->Get<double>("turbulence_intensity", 1.0).first; // m/s
    double sigma_u = sigma_w / (0.177 + 0.000823 * altitude);
    double sigma_v = sigma_u;

    drydenU_.L = L_horiz; drydenU_.sigma = sigma_u;
    drydenV_.L = L_horiz; drydenV_.sigma = sigma_v;
    drydenW_.L = L_vert;  drydenW_.sigma = sigma_w;

    rng_.seed(std::random_device{}());

    // Init ROS 2
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::InitOptions opts;
      opts.shutdown_on_signal = false;   // ← important
      rclcpp::init(argc, nullptr, opts);
    }

    rosNode_ = std::make_shared<rclcpp::Node>("wind_plugin_node");
    
    // Topic unique pour tout contrôler
    windSub_ = rosNode_->create_subscription<wind_msgs::msg::WindCmd>(
        "/wind/cmd", 10,
        [this](const wind_msgs::msg::WindCmd::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(windMutex_);
          forceMean_      = gz::math::Vector3d(
                              msg->force_mean.x,
                              msg->force_mean.y,
                              msg->force_mean.z);
          forceVariance_  = msg->force_variance;
          torqueMean_     = gz::math::Vector3d(
                              msg->torque_mean.x,
                              msg->torque_mean.y,
                              msg->torque_mean.z);
          torqueVariance_ = msg->torque_variance;
          gustForceMag_   = msg->gust_force_magnitude;
          gustTorqueMag_  = msg->gust_torque_magnitude;
          gustFrequency_  = msg->gust_frequency;
        });

    // Publisher pour monitorer
    windPub_ = rosNode_->create_publisher<wind_msgs::msg::WindCmd>(
        "/wind/current", 10);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(rosNode_);
    rosThread_ = std::thread([this]() { executor_->spin(); });

    gzmsg << "[WindPlugin] Chargé — link: " << linkName_ << "\n";
  }

  void PreUpdate(
      const gz::sim::UpdateInfo & _info,
      gz::sim::EntityComponentManager & _ecm) override
  {
    if (linkEntity_ == gz::sim::kNullEntity) {
      linkEntity_ = model_.LinkByName(_ecm, linkName_);
      if (linkEntity_ == gz::sim::kNullEntity) return;
      gz::sim::enableComponent<gz::sim::components::ExternalWorldWrenchCmd>(
          _ecm, linkEntity_);
    }

    if (_info.paused) return;

    double t = std::chrono::duration<double>(_info.simTime).count();

    gz::math::Vector3d force, torque;

    {
      std::lock_guard<std::mutex> lock(windMutex_);

      std::normal_distribution<double> fDist(0.0, forceVariance_);
      std::normal_distribution<double> tDist(0.0, torqueVariance_);

      double dt = std::chrono::duration<double>(_info.dt).count();

      // Bruit blanc source
      std::normal_distribution<double> whiteDist(0.0, 1.0);
      double noiseU = whiteDist(rng_);
      double noiseV = whiteDist(rng_);
      double noiseW = whiteDist(rng_);

      // Turbulence de Dryden (corrélée dans le temps)
      double turbU = drydenU_.updateOrder1(noiseU, dt);
      double turbV = drydenV_.updateOrder2(noiseV, dt);
      double turbW = drydenW_.updateOrder2(noiseW, dt);

      // Appliquer sur le vent moyen
      force = forceMean_;
      force.X() += turbU + gustForceMag_ * gust;
      force.Y() += turbV;
      force.Z() += turbW * 0.3;

      // Torque = mean + turbulence + rafale sur les 3 axes (roll, pitch, yaw)
      torque = torqueMean_;
      torque.X() += tDist(rng_) + gustTorqueMag_ * gust;  // roll
      torque.Y() += tDist(rng_) + gustTorqueMag_ * gust;  // pitch
      torque.Z() += tDist(rng_);                           // yaw (rafale sans sinus)
    }

    // Application wrench complet
    gz::msgs::Wrench wrenchMsg;
    wrenchMsg.mutable_force()->set_x(force.X());
    wrenchMsg.mutable_force()->set_y(force.Y());
    wrenchMsg.mutable_force()->set_z(force.Z());
    wrenchMsg.mutable_torque()->set_x(torque.X());
    wrenchMsg.mutable_torque()->set_y(torque.Y());
    wrenchMsg.mutable_torque()->set_z(torque.Z());

    _ecm.SetComponentData<gz::sim::components::ExternalWorldWrenchCmd>(
        linkEntity_, wrenchMsg);

    // Publier l'état courant
    wind_msgs::msg::WindCmd current;
    current.force_mean.x     = force.X();
    current.force_mean.y     = force.Y();
    current.force_mean.z     = force.Z();
    current.torque_mean.x    = torque.X();
    current.torque_mean.y    = torque.Y();
    current.torque_mean.z    = torque.Z();
    windPub_->publish(current);
  }

  ~WindPlugin() {
    if (executor_) executor_->cancel();
    if (rosThread_.joinable()) rosThread_.join();
  }

private:
  gz::sim::Model  model_;
  gz::sim::Entity linkEntity_ = gz::sim::kNullEntity;
  std::string     linkName_;

  gz::math::Vector3d forceMean_, torqueMean_;
  double forceVariance_  = 0.0;
  double torqueVariance_ = 0.0;
  double gustForceMag_   = 0.0;
  double gustTorqueMag_  = 0.0;
  double gustFrequency_  = 0.1;

  std::mt19937 rng_;
  std::mutex   windMutex_;

  rclcpp::Node::SharedPtr rosNode_;
  rclcpp::Subscription<wind_msgs::msg::WindCmd>::SharedPtr windSub_;
  rclcpp::Publisher<wind_msgs::msg::WindCmd>::SharedPtr    windPub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread rosThread_;

  // Filtre de Dryden — état interne par axe (u, v, w)
  struct DrydenFilter {
    double x1 = 0.0;   // état du filtre d'ordre 1
    double x2 = 0.0;   // état du filtre d'ordre 2 (pour v et w)

    // Paramètres
    double L  = 200.0; // longueur de turbulence (m) — dépend de l'altitude
    double sigma = 1.0; // intensité (m/s)
    double Va  = 10.0; // vitesse de l'air (m/s)

    // Filtre d'ordre 1 (axe longitudinal u)
    double updateOrder1(double noise, double dt) {
      double tau = L / Va;
      double a   = dt / (tau + dt);
      x1 = (1.0 - a) * x1 + a * sigma * std::sqrt(2.0 * L / (M_PI * Va)) * noise;
      return x1;
    }

    // Filtre d'ordre 2 (axes latéraux v, w)
    double updateOrder2(double noise, double dt) {
      double tau  = L / Va;
      double b    = sigma * std::sqrt(L / (M_PI * Va));
      double a1   = 2.0 * dt / tau;
      double a2   = (dt / tau) * (dt / tau);
      double xNew = (2.0 - a1) / (1.0 + a1 + a2) * x1
                  - (1.0 / (1.0 + a1 + a2)) * x2
                  + (b * dt / (1.0 + a1 + a2)) * noise;
      x2 = x1;
      x1 = xNew;
      return x1;
    }
  };

  DrydenFilter drydenU_;  // axe longitudinal (X)
  DrydenFilter drydenV_;  // axe latéral (Y)
  DrydenFilter drydenW_;  // axe vertical (Z)
};

} // namespace wind_plugin

GZ_ADD_PLUGIN(wind_plugin::WindPlugin,
              gz::sim::System,
              wind_plugin::WindPlugin::ISystemConfigure,
              wind_plugin::WindPlugin::ISystemPreUpdate)