#include <ros_tools/helpers.h>

#include <ros_tools/types.h>
#include <ros_tools/ros_visuals.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace RosTools
{

    /*
      Draw nn samples from a size-dimensional normal distribution
      with a specified mean and covariance
    */
    /** Todo: Integrate with data allocation */
    void SampleMultivariateGaussian(int size, int S, const Eigen::VectorXd &mean, const Eigen::MatrixXd &cov)
    {
        Eigen::internal::scalar_normal_dist_op<double> randN;        // Gaussian functor
        Eigen::internal::scalar_normal_dist_op<double>::rng.seed(1); // Seed the rng

        // Define mean and covariance of the distribution
        // Eigen::VectorXd mean(size);
        // Eigen::MatrixXd covar(size, size);

        // mean << 0, 0;
        // covar << 1, .5,
        // 	.5, 1;

        Eigen::MatrixXd normTransform(size, size);

        Eigen::LLT<Eigen::MatrixXd> cholSolver(cov);

        // We can only use the cholesky decomposition if
        // the covariance matrix is symmetric, pos-definite.
        // But a covariance matrix might be pos-semi-definite.
        // In that case, we'll go to an EigenSolver
        if (cholSolver.info() == Eigen::Success)
        {
            // Use cholesky solver
            normTransform = cholSolver.matrixL();
        }
        else
        {
            // Use eigen solver
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(cov);
            normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
        }

        Eigen::MatrixXd samples = (normTransform * Eigen::MatrixXd::NullaryExpr(size, S, randN)).colwise() + mean;

        std::cout << "Mean\n"
                  << mean << std::endl;
        std::cout << "Covariance\n"
                  << cov << std::endl;
        std::cout << "Samples\n"
                  << samples << std::endl;
    }

    void uniformToGaussian2D(Eigen::Vector2d &uniform_variables)
    {
        // Temporarily safe the first variable
        double temp_u1 = uniform_variables(0);

        // Convert the uniform variables to gaussian via Box-Muller
        uniform_variables(0) = std::sqrt(-2 * std::log(temp_u1)) * std::cos(2 * M_PI * uniform_variables(1));
        uniform_variables(1) = std::sqrt(-2 * std::log(temp_u1)) * std::sin(2 * M_PI * uniform_variables(1));
    }

    Eigen::Matrix2d rotationMatrixFromHeading(double heading)
    {
        Eigen::Matrix2d result;
        result << std::cos(heading), std::sin(heading), -std::sin(heading), std::cos(heading);

        return result;
    }

    RandomGenerator::RandomGenerator(int seed)
    {
        if (seed == -1)
        {
            rng_double_ = std::mt19937(std::random_device{}());   // Standard mersenne_twister_engine seeded with rd()
            rng_int_ = std::mt19937(std::random_device{}());      // Standard mersenne_twister_engine seeded with rd()
            rng_gaussian_ = std::mt19937(std::random_device{}()); // Standard mersenne_twister_engine seeded with rd()
        }
        else
        {
            rng_double_ = std::mt19937(seed);   // Standard mersenne_twister_engine seeded with rd()
            rng_int_ = std::mt19937(seed);      // Standard mersenne_twister_engine seeded with rd()
            rng_gaussian_ = std::mt19937(seed); // Standard mersenne_twister_engine seeded with rd()
        }
        runif_ = std::uniform_real_distribution<>(0.0, 1.0);
        epsilon_ = std::numeric_limits<double>::epsilon();
    }

    double RandomGenerator::Double()
    {
        return (double)runif_(rng_double_); //(double)distribution_(random_engine_) /
                                            //(double)std::numeric_limits<uint32_t>::max();
    }

    int RandomGenerator::Int(int max)
    {
        std::uniform_int_distribution<std::mt19937::result_type> new_dist(0, max);
        return new_dist(rng_int_);
    }

    double RandomGenerator::Gaussian(double mean, double stddev)
    {
        return BivariateGaussian(Eigen::Vector2d(mean, mean), stddev, stddev, 0.)(0);
    }

    Eigen::Vector2d RandomGenerator::BivariateGaussian(const Eigen::Vector2d &mean, const double major_axis, const double minor_axis, double angle)
    {
        Eigen::Matrix<double, 2, 2> A, Sigma, R, SVD;

        // // Get the angle of the path
        R = rotationMatrixFromHeading(angle);

        // Generate uniform random numbers in 2D
        double u1, u2;
        do
        {
            u1 = runif_(rng_gaussian_);
        } while (u1 <= epsilon_);
        u2 = runif_(rng_gaussian_);
        Eigen::Vector2d uniform_samples(u1, u2);

        // Convert them to a Gaussian
        uniformToGaussian2D(uniform_samples);

        // Convert the semi axes back to gaussians
        SVD << std::pow(major_axis, 2), 0.0, 0.0, std::pow(minor_axis, 2);

        // Compute Sigma and cholesky decomposition
        Sigma = R * SVD * R.transpose();
        A = Sigma.llt().matrixL(); // ~sqrt

        return Eigen::Vector2d(A(0, 0) * uniform_samples(0) + A(0, 1) * uniform_samples(1) + mean(0),
                               A(1, 0) * uniform_samples(0) + A(1, 1) * uniform_samples(1) + mean(1));
    }

    double Bisection(double low, double high, std::function<double(double)> func, double tol)
    {
        if (low > high)
            throw std::runtime_error("Safety Certifier: Bisection low value was higher than the high value!");

        double value_low = func(low);

        double mid;
        double value_mid;

        for (int iterations = 0; iterations < 1000; iterations++)
        {
            mid = (low + high) / 2.0;
            value_mid = func(mid);

            if (std::abs(value_mid) < tol || (high - low) / 2.0 < tol)
                return mid;

            // Extra check because of integers
            if (high - low == 1)
                return high;

            if (RosTools::sgn(value_mid) == RosTools::sgn(value_low))
            {
                low = mid;
                value_low = value_mid;
            }
            else
            {
                high = mid;
            }
            // std::cout << "low: " << low << ", mid: " << mid << ", high: " << high << std::endl;
        }

        throw std::runtime_error("Safety Certifier: Bisection failed!");
    }

    geometry_msgs::msg::Quaternion angleToQuaternion(double angle)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);

        geometry_msgs::msg::Quaternion result;
        result.x = q.getX();
        result.y = q.getY();
        result.z = q.getZ();
        result.w = q.getW();

        return result;
    }

    double quaternionToAngle(const geometry_msgs::msg::Pose &pose)
    {
        double ysqr = pose.orientation.y * pose.orientation.y;
        double t3 = +2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
        double t4 = +1.0 - 2.0 * (ysqr + pose.orientation.z * pose.orientation.z);

        return atan2(t3, t4);
    }

    double quaternionToAngle(geometry_msgs::msg::Quaternion q)
    {
        double ysqr, t3, t4;

        // Convert from quaternion to RPY
        ysqr = q.y * q.y;
        t3 = +2.0 * (q.w * q.z + q.x * q.y);
        t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
        return std::atan2(t3, t4);
    }

    /** @todo: Translate to ROS2
    
    bool transformPose(tf2_ros::TransformListener &tf_listener_, const std::string &from, const std::string &to, geometry_msgs::msg::Pose &pose)
    {
        bool transform = false;

        // ROS_DEBUG_STREAM("Transforming from :" << from << " to: " << to);
        geometry_msgs::msg::PoseStamped stampedPose_in, stampedPose_out;
        stampedPose_in.pose = pose;

        if (std::sqrt(std::pow(pose.orientation.x, 2) + std::pow(pose.orientation.y, 2) + std::pow(pose.orientation.z, 2) +
                      std::pow(pose.orientation.w, 2)) < 1.0 - 1e-9)
        {
            stampedPose_in.pose.orientation.x = 0;
            stampedPose_in.pose.orientation.y = 0;
            stampedPose_in.pose.orientation.z = 0;
            stampedPose_in.pose.orientation.w = 1;
            std::cout << "LMPCC: Quaternion was not normalised properly!" << std::endl;
        }
        //    stampedPose_in.header.stamp = ros::Time::now();
        stampedPose_in.header.frame_id = from;

        // make sure source and target frame exist
        if (tf_listener_.frameExists(to) && tf_listener_.frameExists(from))
        {
            try
            {
                tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.02));
                tf_listener_.transformPose(to, stampedPose_in, stampedPose_out);

                transform = true;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("MPCC::getTransform: %s", ex.what());
            }
        }
        else
        {
            RCLCPP_WARN(HELPERS_LOGGER, ("MPCC::getTransform: '%s' or '%s' frame doesn't exist, pass existing frame", from.c_str(), to.c_str());
            if (!tf_listener_.frameExists(to))
            {
                RCLCPP_WARN(HELPERS_LOGGER, ("%s doesn't exist", to.c_str());
            }
            if (!tf_listener_.frameExists(from))
            {
                RCLCPP_WARN(HELPERS_LOGGER, ("%s doesn't exist", from.c_str());
            }
        }
        pose = stampedPose_out.pose;
        stampedPose_in.pose = stampedPose_out.pose;
        stampedPose_in.header.frame_id = to;

        return transform;
    }*/

    void DrawPoint(ROSMarkerPublisher &ros_markers, const Eigen::Vector2d &point)
    {
        // Get a line drawer and set properties
        ROSPointMarker &point_marker = ros_markers.getNewPointMarker("CUBE");
        point_marker.setColor(0., 0., 1.);
        point_marker.setScale(0.2, 0.2, 0.2);

        point_marker.addPointMarker(Eigen::Vector3d(point(0), point(1), 0.2));
    }

    void DrawLine(ROSLine &line, double a1, double a2, double b, double line_length)
    {
        // Loop through the columns of the constraints

        // Constraint in z
        if (std::abs(a1) < 1e-3 && std::abs(a2) < 1e-3)
        {
            RCLCPP_WARN_ONCE(HELPERS_LOGGER, "Invalid constraint ignored during visualisation! (this warning appears once!)");
            return;
        }

        geometry_msgs::msg::Point p1, p2;
        double z = 0.;

        // If we cant draw in one direction, draw in the other
        if (std::abs(a2) >= 1e-3)
        {
            p1.x = -line_length;
            p1.y = (b - a1 * p1.x) / a2; // (b - a1*x) / a2
            p1.z = z;

            p2.x = line_length;
            p2.y = (b - a1 * p2.x) / a2;
            p2.z = z;
        }
        else
        {
            // Draw the constraint as a line
            p1.y = -line_length;
            p1.x = (b - a2 * p1.y) / a1;
            p1.z = z;

            p2.y = line_length;
            p2.x = (b - a2 * p2.y) / a1;
            p2.z = z;
        }

        line.addLine(p1, p2);
    };

    void DrawHalfspaces(ROSMarkerPublisher &ros_markers, const std::vector<Halfspace> &halfspaces, int idx)
    {

        for (auto &halfspace : halfspaces)
        {
            ROSLine &line = ros_markers.getNewLine();
            line.setColorInt(idx);
            DrawLine(line, halfspace.A_[0], halfspace.A_[1], halfspace.b_);
        }
    };

    SignalPublisher::SignalPublisher(rclcpp::Node::SharedPtr node, const std::string &signal_name)
    {
        pub_ = node->create_publisher<std_msgs::msg::Float32>("/lmpcc/" + signal_name, 1);
    }

    void SignalPublisher::Publish(double signal_value)
    {
        msg.data = signal_value;
        pub_->publish(msg);
    }

    /*StatusPublisher::StatusPublisher(ros::NodeHandle &nh, const std::string &&topic)
    {
        pub_ = nh.advertise<jsk_rviz_plugins::OverlayText>(topic, 1);
    }

    void StatusPublisher::Publish(const std::string &&status)
    {
        msg_.text = status;
        Publish();
    }
    void StatusPublisher::Publish()
    {
        // Show a message on the screen
        msg_.action = 0;
        msg_.width = 100;
        msg_.height = 20;
        // msg_.left = 10;
        // msg_.top = 80;
        msg_.bg_color.a = 0.;
        msg_.line_width = 2;
        msg_.text_size = 12.0;
        msg_.font = "DejaVu Sans Mono";
        msg_.fg_color.r = 0.098;
        msg_.fg_color.g = 0.94;
        msg_.fg_color.b = 0.94;
        msg_.fg_color.a = 1.0;

        pub_.publish(msg_);
    }*/

    Benchmarker::Benchmarker(const std::string &name, bool record_duration, int ignore_first)
    {
        name_ = name;
        record_duration_ = record_duration;
        running_ = false;
        ignore_first_ = ignore_first;
    }

    void Benchmarker::initialize(const std::string &name, bool record_duration, int ignore_first)
    {
        name_ = name;
        record_duration_ = record_duration;
        ignore_first_ = ignore_first;
    }

    // Print results on destruct
    Benchmarker::~Benchmarker()
    {
        double average_run_time = total_duration_ / ((double)total_runs_) * 1000.0;

        std::cout << "Timing Results for [" << name_ << "]\n";
        std::cout << "Average: " << average_run_time << " ms\n";
        std::cout << "Min: " << min_duration_ * 1000.0 << " ms\n";
        std::cout << "Max: " << max_duration_ * 1000.0 << " ms\n";
    }

    void Benchmarker::start()
    {
        running_ = true;
        start_time_ = std::chrono::system_clock::now();
    }

    double Benchmarker::stop()
    {
        if (!running_)
            return 0.0;

        // Don't time the first 10, there may be some startup behavior
        if (total_runs_ < ignore_first_)
        {
            total_runs_++;
            return 0.0;
        }

        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> current_duration = end_time - start_time_;

        if (record_duration_)
            duration_list_.push_back(current_duration.count() * 1000.0); // in ms

        if (current_duration.count() < min_duration_)
            min_duration_ = current_duration.count();

        if (current_duration.count() > max_duration_)
            max_duration_ = current_duration.count();

        total_duration_ += current_duration.count();
        total_runs_++;
        running_ = false;

        last_ = current_duration.count();
        return last_;
    }

    void Benchmarker::dataToMessage(std_msgs::msg::Float64MultiArray &msg)
    {
        msg.data.resize(duration_list_.size());

        for (size_t i = 0; i < duration_list_.size(); i++)
            msg.data[i] = duration_list_[i];
    }

    void Benchmarker::reset()
    {
        total_runs_ = 0;
        total_duration_ = 0.0;
        max_duration_ = -1.0;
        min_duration_ = 99999.0;
    }

    Eigen::Vector2d DouglasRachford::Project(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const double r, const Eigen::Vector2d &start_pose)
    {
        if (std::sqrt((p - delta).transpose() * (p - delta)) < r)
            return delta - (delta - start_pose) / (std::sqrt((start_pose - delta).transpose() * (start_pose - delta))) * r;
        else
            return p;
    }

    Eigen::Vector2d DouglasRachford::Reflect(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const double r, const Eigen::Vector2d &start_pose)
    {
        return 2.0 * Project(p, delta, r, start_pose) - p;
    }

    Eigen::Vector2d DouglasRachford::DouglasRachfordProjection(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const Eigen::Vector2d &anchor, const double r,
                                                               const Eigen::Vector2d &start_pose)
    {
        return (p + Reflect(Reflect(p, anchor, r, p), delta, r, start_pose)) / 2.0;
    }

    TriggeredTimer::TriggeredTimer(const double &duration) { duration_ = duration; }

    void TriggeredTimer::start() { start_time = std::chrono::system_clock::now(); }

    double TriggeredTimer::currentDuration()
    {
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> current_duration = end_time - start_time;

        return current_duration.count();
    }

    bool TriggeredTimer::hasFinished()
    {
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> current_duration = end_time - start_time;

        return current_duration.count() >= duration_;
    }

    SimulationTool::SimulationTool(rclcpp::Node::SharedPtr node, const std::string &topic, double min_time_between, int max_experiments) : max_experiments_(max_experiments)
    {
        counter_ = 0;
        finished_ = false;
        node_ = node;
        reset_sub_ = node->create_subscription<std_msgs::msg::Empty>(topic.c_str(), 1, std::bind(&SimulationTool::ResetCallback, this, std::placeholders::_1));
        timer_.reset(new TriggeredTimer(min_time_between));
        timer_->start();
    }

    void SimulationTool::ResetCallback(const std_msgs::msg::Empty &msg)
    {
        // Was this the last simulation (noting that the system is reset initially)
        if (counter_ >= max_experiments_)
        {
            RCLCPP_ERROR_STREAM(HELPERS_LOGGER, "Simulation Tool: Done with " << max_experiments_ << " experiments!");
            finished_ = true;
        }

        // Otherwise count
        if (timer_->hasFinished())
        {
            counter_++;
            RCLCPP_WARN_STREAM(HELPERS_LOGGER, "\033[34;47mSimulation Tool: === Experiment " << counter_ << " / " << max_experiments_ << " ===\033[m");
        }
    }

};