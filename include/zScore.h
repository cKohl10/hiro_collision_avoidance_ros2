 #pragma once

 #include <string> 
 #include <vector>
 #include <numeric>
 #include <tuple>
 #include <cmath>
 #include <algorithm>
 #include "std_msgs/msg/float64.hpp"
 #include <rclcpp/rclcpp.hpp>


 
 
  class zScore{
    public:
      int lag = 500; // How many previous values are we talking into account for data smoothing
      float threshold = 0.75; // Number of std deviations needed to show a signal
      float influence = 0.1; // How much weight do we give to signaled values
      bool publish_values = true; // Do you want these values continuously published in ros?
      
      // float max_threshold = 6.7;
      float max_threshold = 6.7;
      // float max_threshold = 8;
      std::tuple<bool, float> getSignal(double new_value);
      double updateThreshold(double velocity);

      // Constructors
      zScore();
      zScore(std::shared_ptr<rclcpp::Node> node_handle, std::string topic_prefix);
      double current_stdDev;
      double current_mean;
      double current_signal;
      double current_raw_value;


    private:

      std::string topic_prefix = "not_set";
      double getStdDev(std::vector<double> data);
      double getMean(std::vector<double> data);
      void publishValues();
      std_msgs::msg::Float64 toROSType(double value);
      std::vector<double> lag_values;

      // Create all publishers
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_mean;
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_positive_threshold;
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_negative_threshold;
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_signal;
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_raw_value;
  };