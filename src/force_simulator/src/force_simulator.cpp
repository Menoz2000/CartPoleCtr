#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>     //per applicare forze

using namespace std::chrono_literals;       //per usare suffissi temporali come 50ms per 50 millisecondi

class ForceSimulator : public rclcpp::Node{
  public:
    ForceSimulator() : Node("force_simulator_node"){
      
      this->loadParameters();

      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->timer_period),
        std::bind(&ForceSimulator::timerCb, this)
      );

      force_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>(this->force_topic, 10);


    }
  private:

    void loadParameters() {
        this->declare_parameter("force_bound.upper_bound", 50.0);
        this->get_parameter("force_bound.upper_bound", this->upper_bound);

        this->declare_parameter("force_bound.lower_bound", 0.0);
        this->get_parameter("force_bound.lower_bound", this->lower_bound);

        this->declare_parameter("ros_timer.period", 1000);
        this->get_parameter("ros_timer.period", this->timer_period);
        
        this->declare_parameter("topic.force_topic", "");
        this->get_parameter("topic.force_topic", this->force_topic);
        
    }

    void timerCb() {
      geometry_msgs::msg::Wrench msg;
      if (count % 2 == 0)
        msg.force.y = random_force();
      else msg.force.y = -1 * random_force();

      this->force_publisher_->publish(msg);
      count ++;

      RCLCPP_INFO(this->get_logger(),"Send y-force: %lf", msg.force.y);

    }

    double random_force() {
      const long max_rand = 1000000L;

      srandom((unsigned) time(NULL));
      double random_double = lower_bound
                            + (upper_bound - lower_bound)
                                  * (random() % max_rand)
                                  / max_rand;

      // RCLCPP_INFO(this->get_logger(),"Force generated: %lf", random_double);
      

      return random_double;
    }

  double upper_bound, lower_bound; //random bound
  int timer_period; //timer period
  long unsigned int count = 0; //count for positive/negative

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_publisher_;
  std::string force_topic;

};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);                               // Inizializza l'ambiente ROS
    rclcpp::spin(std::make_shared<ForceSimulator>());       // Esegue il nodo CartPoleController, mantenendolo attivo e ascoltando per messaggi
    rclcpp::shutdown();                                     // Chiude ROS2 al termina
    return 0;
}