#include "Socket.hpp"
#include "Utils.hpp"

namespace Robot
{

  void Socket::setup()
  {

    poseSubscription_ = this->create_subscription<Pose>(
        "/pose", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Socket::poseCallback, this, std::placeholders::_1));
    goalSubscription_ = this->create_subscription<Pose>(
        "/goal", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Socket::goalCallback, this, std::placeholders::_1));
    lidarSubscription_ = this->create_subscription<Lidar>(
        "/Robot/LDS_01", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Socket::lidarCallback, this, std::placeholders::_1));

    // try three times
    io_service_ = std::make_shared<boost::asio::io_service>();
    socket_ = std::make_shared<tcp::socket>(*io_service_);
    acceptor_ = std::make_shared<tcp::acceptor>(*io_service_, tcp::endpoint(tcp::v4(), 12345));

    // Start accepting connections in a separate thread
    accept_thread_ = std::thread([this]()
                                 { acceptConnections(); });

    LOG(this, "Socket initialized");
  }

  void Socket::acceptConnections()
  {
    acceptor_->accept(*socket_);
    RCLCPP_INFO(get_logger(), "Accepted connection from: %s", socket_->remote_endpoint().address().to_string().c_str());

    // Start a thread to handle this connection
    std::thread([this]()
                { handleConnection(); })
        .detach();
  }

  void Socket::handleConnection()
  {

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->shared_from_this());

    while (rclcpp::ok())
    {

      executor.spin_some();

      // json of jsons
      nlohmann::json json = {
          {"lidar", {
                        {"angle_min", lidar_.angle_min},
                        {"angle_max", lidar_.angle_max},
                        {"angle_increment", lidar_.angle_increment},
                        {"ranges", lidar_.ranges},
                        {"range_min", lidar_.range_min},
                        {"range_max", lidar_.range_max},
                    }},
          {"pose", {
                       {"x", pose_.position.x},
                       {"y", pose_.position.y},
                       {"z", pose_.position.z},
                       {"w", pose_.orientation.w},
                       {"x", pose_.orientation.x},
                       {"y", pose_.orientation.y},
                       {"z", pose_.orientation.z},
                   }},
          {"goal", {
                       {"x", goal_.position.x},
                       {"y", goal_.position.y},
                       {"z", goal_.position.z},
                       {"w", goal_.orientation.w},
                       {"x", goal_.orientation.x},
                       {"y", goal_.orientation.y},
                       {"z", goal_.orientation.z},
                   }}};

      auto string = json.dump();

      boost::asio::write(*socket_, boost::asio::buffer(string));

      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  }

} // namespace Robot
