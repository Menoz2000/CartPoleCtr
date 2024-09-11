#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>  //per lo stato del pendolo
#include <geometry_msgs/msg/wrench.hpp>     //per applicare forze
#include <std_srvs/srv/empty.hpp>

using namespace std::chrono_literals;       //per usare suffissi temporali come 50ms per 50 millisecondi

class CartPoleController : public rclcpp::Node {
public:
    //inizializzo i parametri PID e le var per il controllo. nomino il nodo "cart_pole_controller"
    CartPoleController() : Node("cart_pole_controller"), kp_(75.0), ki_(0.075), kd_(7.5), setpoint_(0.0), integral_(0.0), previous_error_(0.0) {
        
        // Imposta il nodo per usare il tempo simulato, sincronizzando il nodo con il tempo gazebo
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // Creo un publisher che pubblica messaggi di tipo Wrench sul topic "/cart/force" con  una coda di 10 messaggi
        force_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/cart/force", 10);
        
        // Creo un subscriber che si iscrive al topic "/cart/pole_state" per ricevere messaggi tipo JointState, rappresentanti lo stato del pendolo. il callback associato è poleStateCallback
        pole_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cart/pole_state", 10, std::bind(&CartPoleController::poleStateCallback, this, std::placeholders::_1));
        
        // crea un client che chiama il servizio /reset_simulation per resettare la simulazione
        reset_client_ = this->create_client<std_srvs::srv::Empty>("/reset_simulation");

        // Crea un timer che chiama la funzione controlLoop ogni 50ms eseguendo il ciclo di controllo PID
        control_timer_ = this->create_wall_timer(50ms, std::bind(&CartPoleController::controlLoop, this));
        
        // Inizializza il tempo corrente utilizzando il clock del nodo
        last_time_ = this->get_clock()->now();
        
        // Al termine della costruzione del nodo chiamo la funzione che resetta la simulazione
        resetSimulation();
    }

private:
    /*
    la funzione poleStateCallback viene chiamata quando si riceve un messaggio sul topic "/cart/pole_state". estrae la posizione del 
    pendolo (l'angolo) e lo memorizza nella variabile pole_angle_. se i dati sono vuoti, viene registrato un avviso
    */
    void poleStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->position.empty()) {
            pole_angle_ = msg->position[0];
            RCLCPP_INFO(this->get_logger(), "Angolo del pendolo ricevuto: %f", pole_angle_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Dati di posizione del pendolo vuoti.");
        }
    }

    /*
    richiede il reset della simulazione inviando una richiesta vuota al servizio /reset_simulation.
    durante l'attesa che il servizio sia disponibile, stampa avvisi.
    dopo il reset, pubblica una forza zero per fermare il carrello
    */
    void resetSimulation() {
        // Richiede il reset del simulatore
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        while (!reset_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "In attesa che il servizio /reset_simulation sia disponibile...");
        }
        reset_client_->async_send_request(request);
        
        // Applica una forza di 0.0 per fermare il carrello
        auto stop_msg = geometry_msgs::msg::Wrench();
        stop_msg.force.y = 0.0;
        force_publisher_->publish(stop_msg);
        RCLCPP_INFO(this->get_logger(), "Reset completato, carrello fermato.");
    }

    void controlLoop() {
        // Calcolo del tempo trascorso (dt) dall'ultima esecuzione
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        if (dt <= 0 || dt > max_dt_threshold) {
            // Riassestare il tempo di riferimento in caso di anomalie
            last_time_ = current_time;
            return;
        }
        last_time_ = current_time;

        // Calcolo del PID
        double error = setpoint_ - pole_angle_;         //errore tra angolo desiderato del pendolo e angolo attuale
        integral_ += error * dt;                        //aggiorna la parte integrale
        double derivative = (error - previous_error_) / dt; //calcola la derivata dell'errore
        double control_signal = kp_ * error + ki_ * integral_ + kd_ * derivative;   //calcola il segnale PID
        previous_error_ = error;

        // Pubblica la forza calcolata
        auto force_msg = geometry_msgs::msg::Wrench();
        force_msg.force.y = control_signal;  // Applica la forza lungo l'asse Y
        force_publisher_->publish(force_msg);

        RCLCPP_INFO(this->get_logger(), "Position err: %f, Forza applicata: %f", error, control_signal);
    }


    // Parametri del controllore PID
    double kp_, ki_, kd_;       // Coefficienti del controllo PID
    double setpoint_;           //Valore target (l'angolo desiderato del pendolo)
    double integral_, previous_error_;  // Memorizzanpo la parte integrale e l'errore precedente per il calcolo PID

    // Tempo e stato
    rclcpp::Time last_time_;        // Tiene traccia dell'ultimo tempo di esecuzione
    double pole_angle_;             // Memorizza l'angolo attuale del pendolo
    double max_dt_threshold = 0.1;  // Limite massimo per il tempo trascorso tra due cicli di controllo

    // ROS2 Publisher, Subscriber, Client, e Timer
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_publisher_;              // Publisher per inviare il segnale di controllo
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pole_state_subscriber_;   // Subscriber per ricevere lo stato del pendolo
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;                          // Client per richiedere il reset della simulazione
    rclcpp::TimerBase::SharedPtr control_timer_;                                            // Timer per eseguire il ciclo di controllo PID
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);                               // Inizializza l'ambiente ROS
    rclcpp::spin(std::make_shared<CartPoleController>());   // Esegue il nodo CartPoleController, mantenendolo attivo e ascoltando per messaggi
    rclcpp::shutdown();                                     // Chiude ROS2 al termina
    return 0;
}
