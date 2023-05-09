
#ifndef ROSTOOLS_HELPERS_H
#define ROSTOOLS_HELPERS_H

#include <Eigen/Eigen>
#include <chrono>
#include <fstream>
#include <mutex>
#include <random>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <thread>

#include <boost/math/distributions/laplace.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

#include <assert.h>

#include <mutex>
#include <thread>

#include <fstream>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>

#include "ros_tools/data_saver.h"
#include "ros_tools/ros_visuals.h"
#include "ros_tools/types.h"

/** Logging Pragmas */
#define ROSTOOLS_HOOK std::cout << __FILE__ << " Line " << __LINE__ << std::endl;
#define ROSTOOLS_ASSERT(Expr, Msg) __ROSTOOLS_ASSERT(#Expr, Expr, __FILE__, __LINE__, Msg)
#define ROSTOOLS_HOOK_MSG(MSG) std::cout << __FILE__ << " Line " << __LINE__ << " [Message: " << MSG << "]" << std::endl;

// From https://stackoverflow.com/questions/3692954/add-custom-messages-in-assert
// #ifndef NDEBUG
inline void __ROSTOOLS_ASSERT(const char *expr_str, bool expr, const char *file, int line, const char *msg)
{
  if (!expr)
  {
    std::cerr << "Assert failed:\t" << msg << "\n"
              << "Expected:\t" << expr_str << "\n"
              << "Source:\t\t" << file << ", line " << line << "\n";
    abort();
  }
}

/*
  We need a functor that can pretend it's const,
  but to be a good random number generator
  it needs mutable state.
*/
namespace Eigen
{
  namespace internal
  {
    template <typename Scalar>
    struct scalar_normal_dist_op
    {
      static boost::mt19937 rng;                       // The uniform pseudo-random algorithm
      mutable boost::normal_distribution<Scalar> norm; // The gaussian combinator

      EIGEN_EMPTY_STRUCT_CTOR(scalar_normal_dist_op)

      template <typename Index>
      inline const Scalar operator()(Index, Index = 0) const { return norm(rng); }
    };

    template <typename Scalar>
    boost::mt19937 scalar_normal_dist_op<Scalar>::rng;

    template <typename Scalar>
    struct functor_traits<scalar_normal_dist_op<Scalar>>
    {
      enum
      {
        Cost = 50 * NumTraits<Scalar>::MulCost,
        PacketAccess = false,
        IsRepeatable = false
      };
    };
  } // end namespace internal
} // end namespace Eigen

namespace RosTools
{
  enum class ObstacleType
  {
    STATIC,
    DYNAMIC,
    RANGE
  };
  enum class ConstraintSide
  {
    BOTTOM,
    TOP,
    UNDEFINED
  };

  struct Scenario
  {
    int idx_;
    int obstacle_idx_;
  };

  struct ScenarioConstraint
  {
    // LinearConstraint2D constraint_; // Improve later

    Scenario *scenario_;

    ObstacleType type_;
    ConstraintSide side_;

    ScenarioConstraint(){};

    ScenarioConstraint(Scenario *scenario, const ObstacleType &type, const ConstraintSide &side)
    {
      scenario_ = scenario;
      type_ = type;
      side_ = side;
    }

    bool isActive(const Eigen::Vector2d &x)
    {
      // double val = constraint_.A_(0, 0) * x(0) + constraint_.A_(0, 1) * x(1) - constraint_.b_(0);

      // return std::fabs(val) < 1e-4;
      return false;
    }

    double Value(const Eigen::Vector2d &x)
    {
      return 0;
      // double val = constraint_.A_(0, 0) * x(0) + constraint_.A_(0, 1) * x(1) - constraint_.b_(0);
      // return -val; // needs to be less than 0, so 0 - val
    }

    int GetHalfspaceIndex(int sample_size)
    {
      return type_ == ObstacleType::DYNAMIC ? sample_size * scenario_->obstacle_idx_ + scenario_->idx_ : scenario_->idx_;
    }
  };

  /**
   * @brief Structure for tracking the scenarios of support
   *
   * @param support_indices_ saves indices for easier search
   * @param scenarios_ scenario and obstacle indices
   */

  struct SupportSubsample
  {
    std::vector<int> support_indices_;
    std::vector<Scenario> scenarios_;

    int support_subsample_size_;

    SupportSubsample(int max_size = 150)
    {
      support_subsample_size_ = 0;
      support_indices_.reserve(max_size);
      scenarios_.reserve(max_size);
    }

    void Add(const Scenario &scenario)
    {
      // No duplicates
      if (ContainsScenario(scenario))
        return;

      // Note: will allocate if above size!
      support_indices_.push_back(scenario.idx_);
      scenarios_.push_back(scenario);
      support_subsample_size_++;
    }

    void Reset() { support_subsample_size_ = 0; }

    bool ContainsScenario(const Scenario &scenario) { return ContainsScenario(scenario.obstacle_idx_); }

    bool ContainsScenario(const int scenario_idx)
    {
      return (std::find(support_indices_.begin(), support_indices_.begin() + support_subsample_size_, scenario_idx) !=
              support_indices_.begin() + support_subsample_size_);
    }

    // Aggregate vector 2 into vector 1
    void MergeWith(const SupportSubsample &other)
    {
      for (int i = 0; i < other.support_subsample_size_; i++)
      {
        if (!ContainsScenario(other.scenarios_[i]))
        {
          Add(other.scenarios_[i]);
        }
      }
    }

    void Print()
    {
      std::cout << "Support Subsample:\n---------------\n";
      for (int i = 0; i < support_subsample_size_; i++)
      {
        std::cout << "Scenario " << scenarios_[i].idx_ << ", Obstacle: " << scenarios_[i].obstacle_idx_ << std::endl;
      }
      std::cout << "---------------\n";
    }

    void PrintUpdate(int bound, const SupportSubsample &removed, int removed_bound, int iterations)
    {
      ROS_INFO_STREAM("SQP (" << iterations << "): Support = " << support_subsample_size_ << "/" << bound
                              << " - Removed: " << removed.support_subsample_size_ << "/" << removed_bound);
    }
  };
  /*
    Draw nn samples from a size-dimensional normal distribution
    with a specified mean and covariance
  */
  /** Todo: Integrate with data allocation */
  inline void SampleMultivariateGaussian(int size, int S, const Eigen::VectorXd &mean, const Eigen::MatrixXd &cov)
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
  typedef std::vector<std::vector<Eigen::VectorXd>> trajectory_sample; // location per obstacle and time step

  inline void uniformToGaussian2D(Eigen::Vector2d &uniform_variables)
  {
    // Temporarily safe the first variable
    double temp_u1 = uniform_variables(0);

    // Convert the uniform variables to gaussian via Box-Muller
    uniform_variables(0) = std::sqrt(-2 * std::log(temp_u1)) * std::cos(2 * M_PI * uniform_variables(1));
    uniform_variables(1) = std::sqrt(-2 * std::log(temp_u1)) * std::sin(2 * M_PI * uniform_variables(1));
  }

  inline Eigen::Matrix2d rotationMatrixFromHeading(double heading)
  {
    Eigen::Matrix2d result;
    result << std::cos(heading), std::sin(heading), -std::sin(heading), std::cos(heading);

    return result;
  }

  // Class for generating random ints/doubles
  class RandomGenerator
  {
  private:
    std::mt19937 rng_double_;
    std::mt19937 rng_int_;
    std::mt19937 rng_gaussian_;
    std::uniform_real_distribution<> runif_;
    double epsilon_;

  public:
    RandomGenerator(int seed = -1)
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

    double Double()
    {
      return (double)runif_(rng_double_); //(double)distribution_(random_engine_) /
                                          //(double)std::numeric_limits<uint32_t>::max();
    }

    int Int(int max)
    {
      std::uniform_int_distribution<std::mt19937::result_type> new_dist(0, max);
      return new_dist(rng_int_);
    }
    // static std::mt19937 random_engine_;
    // static std::uniform_int_distribution<std::mt19937::result_type> distribution_;

    Eigen::Vector2d BivariateGaussian(const Eigen::Vector2d &mean, const double major_axis, const double minor_axis, double angle)
    {
      Eigen::Matrix<double, 2, 2> A, Sigma, R, SVD;

      // // Get the angle of the path
      // psi = RosTools::quaternionToAngle(path.poses[k].pose);
      R = rotationMatrixFromHeading(angle);

      // Generate uniform random numbers in 2D
      // Eigen::Vector2d uniform_samples = Eigen::Vector2d(Double(), Double());
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
  };

  inline double dist(const Eigen::Vector2d &one, const Eigen::Vector2d &two) { return (two - one).norm(); }

  /**
   * @brief Interpolate linearly on a line
   *
   * @param start lower value
   * @param end upper value
   * @param value the current value
   * @return double
   */
  // inline double InterpolateLinearly(double start, double end, double value)
  // {
  // 	double lambda = (value - start) / (end - start);
  // 	return lambda * end + (1. - lambda) * start;
  // }

  /**
   * @brief Interpolate a variable linearly between lower and upper by a representative variable "value" that moves
   * between start and end
   *
   * @tparam T The type of variable to interpolate between
   * @param start Lower value of the domain
   * @param end Upper value of the domain
   * @param value Current value in the domain
   * @param lower Lower value of the mapped domain
   * @param upper Upper value of the mapped domain
   */
  template <class T>
  inline T InterpolateLinearly(double start, double end, double value, const T &lower, const T &upper)
  {
    double lambda = (value - start) / (end - start);
    return lambda * upper + (1. - lambda) * lower;
  }

  inline double LogisticFunction(double L, double k, double x) { return L / (1. + std::exp(-k * (x - (6. / k)))); }

  inline void ProjectOntoDisc(Eigen::Vector2d &point, const Eigen::Vector2d &disc_origin, const double radius)
  {
    point = disc_origin - (disc_origin - point) / (disc_origin - point).norm() * radius;
  }

  inline double evaluate1DCDF(double value) { return 0.5 * erfc(-value * M_SQRT1_2); }

  inline Eigen::Vector3d AsVector3d(const Eigen::Vector2d &vec2, double z_value = 0.) { return Eigen::Vector3d(vec2(0), vec2(1), z_value); }

  // Finds the exponential CDF value at probability p (for a rate of lambda)
  inline double ExponentialQuantile(double lambda, double p) { return -std::log(1 - p) / lambda; }

  template <typename T>
  int sgn(T val) { return (T(0) < val) - (val < T(0)); }

  // inline bool isTopConstraint(const LinearConstraint2D &constraint, const Eigen::Vector2d &pose)
  // {
  //   // Is y at pose.x > pose.y?
  //   double y = (constraint.b_(0) - constraint.A_(0, 0) * pose(0)) / constraint.A_(0, 1);
  //   return (y > pose(1));
  // }

  inline double quaternionToAngle(const geometry_msgs::Pose &pose)
  {
    double ysqr = pose.orientation.y * pose.orientation.y;
    double t3 = +2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
    double t4 = +1.0 - 2.0 * (ysqr + pose.orientation.z * pose.orientation.z);

    return atan2(t3, t4);
  }

  inline double Bisection(double low, double high, std::function<double(double)> func, double tol = 1e-3)
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

  inline geometry_msgs::Quaternion angleToQuaternion(double angle)
  {
    tf::Quaternion q = tf::createQuaternionFromRPY(0., 0., angle);
    geometry_msgs::Quaternion result;
    result.x = q.getX();
    result.y = q.getY();
    result.z = q.getZ();
    result.w = q.getW();

    return result;
  }

  inline double quaternionToAngle(geometry_msgs::Quaternion q)
  {
    double ysqr, t3, t4;

    // Convert from quaternion to RPY
    ysqr = q.y * q.y;
    t3 = +2.0 * (q.w * q.z + q.x * q.y);
    t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
    return std::atan2(t3, t4);
  }

  inline bool transformPose(tf::TransformListener &tf_listener_, const std::string &from, const std::string &to, geometry_msgs::Pose &pose)
  {
    bool transform = false;
    tf::StampedTransform stamped_tf;

    // ROS_DEBUG_STREAM("Transforming from :" << from << " to: " << to);
    geometry_msgs::PoseStamped stampedPose_in, stampedPose_out;
    // std::cout << "from " << from << " to " << to << ", x = " << pose.position.x << ", y = " << pose.position.y <<
    // std::endl;
    stampedPose_in.pose = pose;
    // std::cout << " value: " << std::sqrt(std::pow(pose.orientation.x, 2.0) + std::pow(pose.orientation.y, 2.0) +
    // std::pow(pose.orientation.z, 2.0) + std::pow(pose.orientation.w, 2.0)) << std::endl;
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
        // std::cout << "in transform try " << std::endl;
        // find transforamtion between souce and target frame
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
      ROS_WARN("MPCC::getTransform: '%s' or '%s' frame doesn't exist, pass existing frame", from.c_str(), to.c_str());
      if (!tf_listener_.frameExists(to))
      {
        ROS_WARN("%s doesn't exist", to.c_str());
      }
      if (!tf_listener_.frameExists(from))
      {
        ROS_WARN("%s doesn't exist", from.c_str());
      }
    }
    pose = stampedPose_out.pose;
    stampedPose_in.pose = stampedPose_out.pose;
    stampedPose_in.header.frame_id = to;

    return transform;
  }

  inline void drawPoint(ROSMarkerPublisher &ros_markers, const Eigen::Vector2d &point)
  {
    // Get a line drawer and set properties
    ROSPointMarker &point_marker = ros_markers.getNewPointMarker("CUBE");
    point_marker.setColor(0., 0., 1.);
    point_marker.setScale(0.2, 0.2, 0.2);

    point_marker.addPointMarker(Eigen::Vector3d(point(0), point(1), 0.2));
  }

  inline void drawLine(ROSLine &line, double a1, double a2, double b, double line_length = 100.)
  {
    // Loop through the columns of the constraints

    // Constraint in z
    if (std::abs(a1) < 1e-3 && std::abs(a2) < 1e-3)
    {
      ROS_WARN("Invalid constraint ignored during visualisation!");
      return;
    }

    geometry_msgs::Point p1, p2;
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

  inline std::string GetLMPCCDataPath() { return ros::package::getPath("lmpcc") + "/matlab_exports/data"; }

  class SignalPublisher
  {
  public:
    SignalPublisher(ros::NodeHandle &nh, const std::string &signal_name)
    {
      pub_ = nh.advertise<std_msgs::Float32>("/lmpcc/" + signal_name, 1); /* Publish the sample size when it is incorrect */
    }

    void Publish(double signal_value)
    {
      msg.data = signal_value;
      pub_.publish(msg);
    }

  private:
    ros::Publisher pub_;
    std_msgs::Float32 msg;
  };

  // Use as static to print average run time
  class Benchmarker
  {
  public:
    Benchmarker(const std::string &name, bool record_duration = false, int ignore_first = 10)
    {
      name_ = name;
      record_duration_ = record_duration;
      running_ = false;
      ignore_first_ = ignore_first;
    }

    // Simpler
    Benchmarker() {}

    void initialize(const std::string &name, bool record_duration = false, int ignore_first = 10)
    {
      name_ = name;
      record_duration_ = record_duration;
      ignore_first_ = ignore_first;
    }

    // Print results on destruct
    ~Benchmarker()
    {
      double average_run_time = total_duration_ / ((double)total_runs_) * 1000.0;

      std::cout << "Timing Results for [" << name_ << "]\n";
      std::cout << "Average: " << average_run_time << " ms\n";
      std::cout << "Min: " << min_duration_ * 1000.0 << " ms\n";
      std::cout << "Max: " << max_duration_ * 1000.0 << " ms\n";
    }

    void start()
    {
      running_ = true;
      start_time_ = std::chrono::system_clock::now();
    }

    double stop()
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

    void dataToMessage(std_msgs::Float64MultiArray &msg)
    {
      msg.data.resize(duration_list_.size());

      for (size_t i = 0; i < duration_list_.size(); i++)
        msg.data[i] = duration_list_[i];
    }

    void reset()
    {
      total_runs_ = 0;
      total_duration_ = 0.0;
      max_duration_ = -1.0;
      min_duration_ = 99999.0;
    }

    bool isRunning() { return running_; };

    int getTotalRuns() { return total_runs_; };
    double getLast() { return last_; };

  private:
    std::chrono::system_clock::time_point start_time_;

    double total_duration_ = 0.0;
    double max_duration_ = -1.0;
    double min_duration_ = 99999.0;

    double last_ = -1.0;

    int total_runs_ = 0;

    std::string name_;
    bool record_duration_;
    std::vector<double> duration_list_;
    bool running_ = false;

    int ignore_first_;
  };

  class DouglasRachford
  {
  public:
  private:
    Eigen::Vector2d Project(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const double r, const Eigen::Vector2d &start_pose)
    {
      if (std::sqrt((p - delta).transpose() * (p - delta)) < r)
        return delta - (delta - start_pose) / (std::sqrt((start_pose - delta).transpose() * (start_pose - delta))) * r;
      else
        return p;
    }

    Eigen::Vector2d Reflect(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const double r, const Eigen::Vector2d &start_pose)
    {
      return 2.0 * Project(p, delta, r, start_pose) - p;
    }

  public:
    Eigen::Vector2d DouglasRachfordProjection(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const Eigen::Vector2d &anchor, const double r,
                                              const Eigen::Vector2d &start_pose)
    {
      return (p + Reflect(Reflect(p, anchor, r, p), delta, r, start_pose)) / 2.0;
    }
  };

  class TriggeredTimer
  {
  public:
    // Duration in s
    TriggeredTimer(const double &duration) { duration_ = duration; }

    void start() { start_time = std::chrono::system_clock::now(); }

    double currentDuration()
    {
      auto end_time = std::chrono::system_clock::now();
      std::chrono::duration<double> current_duration = end_time - start_time;

      return current_duration.count();
    }

    bool hasFinished()
    {
      auto end_time = std::chrono::system_clock::now();
      std::chrono::duration<double> current_duration = end_time - start_time;

      return current_duration.count() >= duration_;
    }

  private:
    std::chrono::system_clock::time_point start_time;
    double duration_;
  };

  // Simple class to count the number of simulations
  class SimulationTool
  {
  public:
    SimulationTool(const std::string &topic, double min_time_between, int max_experiments) : max_experiments_(max_experiments)
    {
      counter_ = 0;
      finished_ = false;
      reset_sub_ = nh_.subscribe(topic.c_str(), 1, &SimulationTool::ResetCallback, this);
      timer_.reset(new TriggeredTimer(min_time_between));
      timer_->start();
    }

  public:
    void ResetCallback(const std_msgs::Empty &msg)
    {
      // Was this the last simulation (noting that the system is reset initially)
      if (counter_ >= max_experiments_)
      {
        ROS_ERROR_STREAM("Simulation Tool: Done with " << max_experiments_ << " experiments!");
        finished_ = true;
      }

      // Otherwise count
      if (timer_->hasFinished())
      {
        counter_++;
        ROS_WARN_STREAM("\033[34;47mSimulation Tool: === Experiment " << counter_ << " / " << max_experiments_ << " ===\033[m");
      }
    }

    bool Finished() const { return finished_; };

  private:
    ros::NodeHandle nh_;
    ros::Subscriber reset_sub_;

    int counter_;
    int max_experiments_;

    bool finished_;

    std::unique_ptr<TriggeredTimer> timer_;
  };

  /* T needs to be summable */
  template <class T>
  class StatisticAnalysis
  {
  public:
    StatisticAnalysis()
    {
      data_.reserve(100);
      empty_ = true;
    };

    ~StatisticAnalysis() { std::cout << "MAX: " << Max() << std::endl; }

  public:
    bool Empty() { return empty_; };

    void AddData(const T &new_data)
    {
      if (empty_)
        empty_ = false;

      data_.push_back(new_data);
    }

    void Clear() { data_.clear(); }

    double Mean()
    {
      if (empty_)
        return 0;

      double sum = 0.;
      for (T &d : data_)
        sum += d;

      return sum / ((double)data_.size());
    }

    double Max()
    {
      if (empty_)
        return 0;

      double max = 0.;
      for (T &d : data_)
      {
        if (d > max)
          max = d;
      }
      return max;
    }

    double cVaR(double alpha)
    {
      if (empty_)
        return 0;

      // Sort all data (can be slow)
      std::sort(data_.begin(), data_.end());

      // Get the alpha*N highest value
      double threshold = data_[std::floor((1. - alpha) * (double)data_.size())];

      double sum = 0.;
      int n = 0;
      for (T &d : data_)
      {
        // Add if higher than the threshold
        if (d >= threshold)
        {
          sum += d;
          n++;
        }
      }

      return sum / ((double)n);
    }

    /**
     * @brief How many probability mass is higher than value
     *
     * @param value Threshold
     * @return double (0 - 1)
     */
    double PAbove(const T &value)
    {
      if (Empty())
        return 0.0;

      int count = 0;
      for (T &d : data_)
      {
        if (d > value)
          count++;
      }

      return ((double)count) / data_.size();
    }

    void Load(const std::string &&file_name)
    {
      DataSaver data_saver;

      std::map<std::string, std::vector<int>> read_data;
      bool success = data_saver.LoadData(file_name, read_data);

      if (!success)
        return;

      data_ = read_data["support"];
      empty_ = false;
    }

    void Save(const std::string &&file_name)
    {
      DataSaver data_saver;

      for (T &data_point : data_)
        data_saver.AddData("support", data_point);

      data_saver.SaveData(file_name);
    }

  private:
    std::vector<T> data_;
    bool empty_;
  };

  class CallCounter
  {
  public:
    // Singleton (Use SafetyCertifier::Get(). to access this class)
    static CallCounter &Get()
    {
      static CallCounter instance_;

      return instance_;
    }

    CallCounter(const CallCounter &) = delete;

    void Count() { count_++; };
    void Loop()
    {
      if (count_ == 0)
        return;
      std::cout << "CALL COUNTER COUNT: " << count_ << std::endl;
      count_ = 0;
    }

  private:
    CallCounter(){};

  private:
    int count_ = 0;
  };

/********** Some Fancy timing classes for profiling (from TheCherno) ***********/
#define PROFILER 1
#if PROFILER
#define PROFILE_SCOPE(name) RosTools::InstrumentationTimer timer##__LINE__(name)
#define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)
#define PROFILE_AND_LOG(debug_enable, name) \
  if (debug_enable)                         \
  {                                         \
    ROS_INFO_STREAM(name);                  \
  }                                         \
  RosTools::InstrumentationTimer timer##__LINE__(name)
#else
#define PROFILE_SCOPE(name)
#define PROFILE_FUNCTION()
#endif

  struct ProfileResult
  {
    std::string Name;
    long long Start, End;
    uint32_t ThreadID;
  };

  struct InstrumentationSession
  {
    std::string Name;
  };

  class Instrumentor
  {
  private:
    InstrumentationSession *m_CurrentSession;
    std::ofstream m_OutputStream;
    int m_ProfileCount;
    std::mutex m_lock;

  public:
    Instrumentor() : m_CurrentSession(nullptr), m_ProfileCount(0) {}

    void BeginSession(const std::string &name, const std::string &filepath = "profiler.json")
    {
      std::string full_filepath = ros::package::getPath("guidance_planner") + "/" + filepath;
      m_OutputStream.open(full_filepath);
      WriteHeader();
      m_CurrentSession = new InstrumentationSession{name};
    }

    void EndSession()
    {
      WriteFooter();
      m_OutputStream.close();
      delete m_CurrentSession;
      m_CurrentSession = nullptr;
      m_ProfileCount = 0;
    }

    void WriteProfile(const ProfileResult &result)
    {
      std::lock_guard<std::mutex> lock(m_lock);

      if (m_ProfileCount++ > 0)
        m_OutputStream << ",";

      std::string name = result.Name;
      std::replace(name.begin(), name.end(), '"', '\'');

      m_OutputStream << "{";
      m_OutputStream << "\"cat\":\"function\",";
      m_OutputStream << "\"dur\":" << (result.End - result.Start) << ',';
      m_OutputStream << "\"name\":\"" << name << "\",";
      m_OutputStream << "\"ph\":\"X\",";
      m_OutputStream << "\"pid\":0,";
      m_OutputStream << "\"tid\":" << result.ThreadID << ",";
      m_OutputStream << "\"ts\":" << result.Start;
      m_OutputStream << "}";

      m_OutputStream.flush();
    }

    void WriteHeader()
    {
      m_OutputStream << "{\"otherData\": {},\"traceEvents\":[";
      m_OutputStream.flush();
    }

    void WriteFooter()
    {
      m_OutputStream << "]}";
      m_OutputStream.flush();
    }

    static Instrumentor &Get()
    {
      static Instrumentor *instance = new Instrumentor();
      return *instance;
    }
  };

  class InstrumentationTimer
  {
  public:
    InstrumentationTimer(const char *name) : m_Name(name), m_Stopped(false) { m_StartTimepoint = std::chrono::system_clock::now(); }

    ~InstrumentationTimer()
    {
      if (!m_Stopped)
        Stop();
    }

    void Stop()
    {
      auto endTimepoint = std::chrono::system_clock::now();

      long long start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimepoint).time_since_epoch().count();
      long long end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint).time_since_epoch().count();

      uint32_t threadID = std::hash<std::thread::id>{}(std::this_thread::get_id());
      Instrumentor::Get().WriteProfile({m_Name, start, end, threadID});

      m_Stopped = true;
    }

  private:
    const char *m_Name;
    std::chrono::system_clock::time_point m_StartTimepoint;
    bool m_Stopped;
  };
};
#endif