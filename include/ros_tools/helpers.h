
#ifndef ROSTOOLS_HELPERS_H
#define ROSTOOLS_HELPERS_H

#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

#include <Eigen/Eigen>

#include <assert.h>
#include <string>
#include <fstream>
#include <random>
#include <chrono>

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
  typedef std::vector<std::vector<Eigen::VectorXd>> trajectory_sample; // location per obstacle and time step

  // Forward declarations
  class ROSMarkerPublisher;
  class ROSPointMarker;
  class ROSLine;
  class Halfspace;

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
  void SampleMultivariateGaussian(int size, int S, const Eigen::VectorXd &mean, const Eigen::MatrixXd &cov);

  void uniformToGaussian2D(Eigen::Vector2d &uniform_variables);

  Eigen::Matrix2d rotationMatrixFromHeading(double heading);

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
    RandomGenerator(int seed = -1);

    double Double();
    int Int(int max);

    double Gaussian(double mean, double stddev);
    Eigen::Vector2d BivariateGaussian(const Eigen::Vector2d &mean,
                                      const double major_axis, const double minor_axis,
                                      double angle);
  };

  inline double dist(const Eigen::Vector2d &one, const Eigen::Vector2d &two) { return (two - one).norm(); }

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
  inline double evaluate1DCDF(double value) { return 0.5 * erfc(-value * M_SQRT1_2); }
  inline double ExponentialQuantile(double lambda, double p) { return -std::log(1 - p) / lambda; } // Finds the exponential CDF value at probability p (for a rate of lambda)

  inline Eigen::Vector3d AsVector3d(const Eigen::Vector2d &vec2, double z_value = 0.) { return Eigen::Vector3d(vec2(0), vec2(1), z_value); }

  template <typename T>
  int sgn(T val) { return (T(0) < val) - (val < T(0)); }

  double Bisection(double low, double high, std::function<double(double)> func, double tol = 1e-3);

  geometry_msgs::Quaternion angleToQuaternion(double angle);

  double quaternionToAngle(const geometry_msgs::Pose &pose);
  double quaternionToAngle(geometry_msgs::Quaternion q);

  bool transformPose(tf::TransformListener &tf_listener_, const std::string &from, const std::string &to, geometry_msgs::Pose &pose);

  void DrawPoint(ROSMarkerPublisher &ros_markers, const Eigen::Vector2d &point);
  void DrawLine(ROSLine &line, double a1, double a2, double b, double line_length = 100.);
  void DrawHalfspaces(ROSMarkerPublisher &ros_markers, const std::vector<Halfspace> &halfspaces, double r = 0, double g = 1, double b = 0);

  class SignalPublisher
  {
  public:
    SignalPublisher(ros::NodeHandle &nh, const std::string &signal_name);
    void Publish(double signal_value);

  private:
    ros::Publisher pub_;
    std_msgs::Float32 msg;
  };

  // Use as static to print average run time
  class Benchmarker
  {
  public:
    Benchmarker(const std::string &name, bool record_duration = false, int ignore_first = 10);

    // Simpler
    Benchmarker() {}
    void initialize(const std::string &name, bool record_duration = false, int ignore_first = 10);

    // Print results on destruct
    ~Benchmarker();

    void start();
    double stop();
    void reset();

    void dataToMessage(std_msgs::Float64MultiArray &msg);

    bool isRunning() const { return running_; };

    int getTotalRuns() const { return total_runs_; };
    double getLast() const { return last_; };

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
    Eigen::Vector2d DouglasRachfordProjection(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const Eigen::Vector2d &anchor, const double r,
                                              const Eigen::Vector2d &start_pose);

  private:
    Eigen::Vector2d Project(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const double r, const Eigen::Vector2d &start_pose);

    Eigen::Vector2d Reflect(const Eigen::Vector2d &p, const Eigen::Vector2d &delta, const double r, const Eigen::Vector2d &start_pose);
  };

  class TriggeredTimer
  {
  public:
    // Duration in s
    TriggeredTimer(const double &duration);

    void start();

    double currentDuration();
    bool hasFinished();

  private:
    std::chrono::system_clock::time_point start_time;
    double duration_;
  };

  // Simple class to count the number of simulations
  class SimulationTool
  {
  public:
    SimulationTool(const std::string &topic, double min_time_between, int max_experiments);

  public:
    void ResetCallback(const std_msgs::Empty &msg);

    bool Finished() const { return finished_; };

  private:
    ros::NodeHandle nh_;
    ros::Subscriber reset_sub_;

    int counter_;
    int max_experiments_;

    bool finished_;

    std::unique_ptr<TriggeredTimer> timer_;
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
};
#endif