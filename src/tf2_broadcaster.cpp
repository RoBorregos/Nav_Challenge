#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

using namespace std; 

class FramePublisher : public rclcpp::Node
{
public:
  // Constructor
  FramePublisher()
  : Node("tf2_publisher") // Nombre del nodo
  {
    // Declarar y obtener el parámetro turtlename
    turtlename_ = this->declare_parameter<string>("turtlename", "turtle");

    // Inicializar el transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Suscribirse al tópico pose de la tortuga y llamar a la función de callback handle_turtle_pose en cada mensaje
    ostringstream stream;
    stream << "/" << turtlename_.c_str() << "/pose";
    string topic_name = stream.str();

    // Crear la suscripción
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      topic_name, 10,
      bind(&FramePublisher::handle_turtle_pose, this, placeholders::_1));
  }

private:
  // Callback para manejar la posición de la tortuga
  void handle_turtle_pose(const shared_ptr<turtlesim::msg::Pose> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Leer el contenido del mensaje y asignarlo a las variables correspondientes de tf
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtlename_.c_str(); // frame hijo

    // La tortuga solo existe en 2D, por lo tanto obtenemos x y y del mensaje y dejamos z en 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // Como la tortuga solo puede rotar alrededor de un eje, establecemos la rotación en x y y en 0, obteniendo la rotación en z del mensaje
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Enviar la transformación
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_; // Suscripción al tópico de posición
  unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // Broadcaster de la transformación
  string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}