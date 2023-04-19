/**
 * @file risk_computation.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Class for quickly computing the risk of a plan under Gaussian or Mixture of Gaussian distributions
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __RISK_COMPUTATION_H__
#define __RISK_COMPUTATION_H__

#include <ros/package.h>

#include <boost/filesystem.hpp>

#include <ros_tools/data_saver.h>
#include <ros_tools/helpers.h>
#include <ros_tools/ros_visuals.h>

int horizon_max = 20;
using namespace RosTools;
class RiskComputation
{
public:
  RiskComputation()
  {
    int max_obs = 8;  // 1
    // if (is_binomial)
    //     max_obs = 2;

    double static_dist = 0.625;  // 0.625; // 0.625; // 0.325
    double noise = 0.3;          // Was 0.3
    bool compute_region = false;

    ros_markers_.reset(new ROSMarkerPublisher(nh_, "/risk/samples", "map", 1800));

    ros::NodeHandle nh("~");
    nh.getParam("sample_size", S);
    nh.getParam("folder", folder);
    nh.getParam("binomial", is_binomial);
    nh.getParam("ellipsoid", is_ellipsoid);

    if (is_ellipsoid)
      ROS_WARN("Ellipsoid is true");

    std::string path = ros::package::getPath("lmpcc") + "/matlab_exports/data/" + folder;

    // Get all files in the folder
    std::vector<std::string> files;
    for (const auto& entry : boost::filesystem::directory_iterator(path))
    {
      if (boost::filesystem::is_regular_file(entry))
        files.emplace_back(entry.path().string());
    }
    // Get the newest file
    std::sort(files.begin(), files.end());

    std::string file = files.back();
    auto pos = file.find_last_of('/');
    if (pos != std::string::npos)
    {
      file = file.substr(pos);  // Get the file name only
      file.erase(0, 1);         // Remove the "/""
      file.pop_back();          // Remove the extension
      file.pop_back();
      file.pop_back();
      file.pop_back();
    }

    // bool ellipsoid = file.find("ellipsoid") != std::string::npos;

    // Retrieve data from the selected file
    DataSaver data_saver;
    std::map<std::string, std::vector<double>> scalar_data;
    std::map<std::string, std::vector<Eigen::Vector2d>> vector_data;
    data_saver.LoadAllData(path, file, scalar_data, vector_data);

    int data_size = scalar_data["status"].size();  // scalar_data["reset"].size();

    // Split simulations
    std::vector<int> reset_times;
    std::vector<double>& reset_data = scalar_data["reset"];
    for (size_t i = 0; i < reset_data.size(); i++)
    {
      reset_times.push_back((int)reset_data[i]);
    }

    std::cout << "Generating samples...\n";
    std::vector<Eigen::VectorXd> sample_x, sample_y;  // k
    GenerateSamples(sample_x, sample_y, noise);

    std::cout << "Retrieving vehicle data...\n";

    // Vehicle data
    std::vector<std::vector<Eigen::Vector2d>> x_veh;  // k, t, (x, y)
    x_veh.resize(horizon_max);
    for (int k = 0; k < horizon_max; k++)
      x_veh[k] = vector_data["vehicle_plan_" + std::to_string(k)];

    // std::cout << "x " << 0 << ": " << x_veh[0][0] << std::endl;

    std::cout << "Retrieving obstacle data...\n";

    // Obstacle data
    // Check the number of modes
    std::vector<std::vector<int>> num_modes;
    num_modes.resize(max_obs);
    int mode_nr = -1;

    if (is_binomial)
    {
      for (int v = 0; v < max_obs; v++)  // Number of modes for each obstacle
      {
        std::vector<double>& data = scalar_data["prediction_num_modes_" + std::to_string(v)];
        for (size_t t = 0; t < data.size(); t++)  // At each time step
          num_modes[v].push_back((int)data[t]);
      }

      for (int v = 0; v < max_obs; v++)
      {
        if (num_modes[v][0] > mode_nr)
          mode_nr = num_modes[v][0];  // Just use the max
      }
      mode_nr = horizon_max + 1;
      // std::cout << mode_nr << std::endl;
    }
    else
    {
      for (int v = 0; v < max_obs; v++)  // Number of modes for each obstacle
      {
        for (size_t t = 0; t < x_veh[0].size(); t++)  // At each time step
          num_modes[v].push_back(1);
      }
      mode_nr = 1;
      std::cout << "Only one mode detected!\n";
    }

    std::vector<std::vector<std::vector<std::vector<Eigen::Vector2d>>>> x_obs;  // v, mode, k, t, (x, y)
    std::vector<std::vector<std::vector<double>>> p_obs;                        // v, mode, t (Assume constant over k)
    for (int v = 0; v < max_obs; v++)
    {
      x_obs.emplace_back();
      p_obs.emplace_back();
      for (int mode = 0; mode < mode_nr; mode++)
      {
        x_obs[v].emplace_back();
        x_obs[v][mode].resize(horizon_max);
        for (int k = 0; k < horizon_max; k++)
        {
          if (is_binomial)
            x_obs[v][mode][k] = vector_data["prediction_mode_" + std::to_string(mode) + "_mean_" + std::to_string(v) +
                                            "_" + std::to_string(k)];
          else
            x_obs[v][0][k] = vector_data["prediction_mean_" + std::to_string(v) + "_" + std::to_string(k)];
        }
        p_obs[v].emplace_back();
        if (is_binomial)
          p_obs[v][mode] = scalar_data["prediction_mode_" + std::to_string(mode) + "_p_" + std::to_string(v)];
        else
          p_obs[v][mode] = std::vector<double>(x_obs[v][0][0].size(), 1.0);
      }
    }

    std::cout << "Computing risk:\n";

    // Sampler::Get().Init(nh, );
    std::vector<int> sample_counts_per_mode;

    // We assume that there is only one type of mode distribution otherwise it is too slow
    ConstructModeSamples(0, 0, mode_nr, p_obs, sample_counts_per_mode);

    Eigen::ArrayXd diffs_x, diffs_y, distances;
    diffs_x.resize(S);
    diffs_y.resize(S);
    distances.resize(S);

    std::vector<double> risk;                     // t
    std::vector<std::vector<double>> stage_risk;  // t, k
    for (int t = 0; t < data_size; t++)           // For all experiments
    {
      stage_risk.emplace_back();
      if (scalar_data["status"][t] != 2)  // Ignore infeasible steps
      {
        risk.push_back(0.0);
        stage_risk[t].resize(horizon_max);
        for (int k = 0; k < horizon_max; k++)
          stage_risk[t][k] = 0.;

        continue;
      }
      std::cout << "[" << t << "] \n";

      // Evaluate samples per experiment
      std::vector<int> sample_in_collision(S, 0);

      for (int k = 0; k < horizon_max; k++)  // For all timesteps
      {
        std::vector<int> sample_in_collision_stage(S, 0);  // Also track in which stages the samples are violated

        for (int v = 0; v < max_obs; v++)  // For all obstacles
        {
          // Computation for k, v (for all S)

          // The modes complicate this process a bit (we need to compute sections of the diffs with the number of picked
          // samples)
          if (num_modes[v][t] > 1)
          {
            int cur_s = 0;
            for (int mode = 0; mode < num_modes[v][t]; mode++)
            {
              // Compute for all samples the distances
              diffs_x.block(cur_s, 0, sample_counts_per_mode[mode], 1) =
                  x_veh[k][t](0) - x_obs[v][mode][k][t](0) -
                  sample_x[k].block(cur_s, 0, sample_counts_per_mode[mode], 1).array();
              diffs_y.block(cur_s, 0, sample_counts_per_mode[mode], 1) =
                  x_veh[k][t](1) - x_obs[v][mode][k][t](1) -
                  sample_y[k].block(cur_s, 0, sample_counts_per_mode[mode], 1).array();
              distances.block(cur_s, 0, sample_counts_per_mode[mode], 1) =
                  (diffs_x.block(cur_s, 0, sample_counts_per_mode[mode], 1) *
                       diffs_x.block(cur_s, 0, sample_counts_per_mode[mode], 1) +
                   diffs_y.block(cur_s, 0, sample_counts_per_mode[mode], 1) *
                       diffs_y.block(cur_s, 0, sample_counts_per_mode[mode], 1))
                      .sqrt();

              cur_s += sample_counts_per_mode[mode];
            }
          }
          else
          {
            diffs_x = x_veh[k][t](0) - x_obs[v][0][k][t](0) - sample_x[k].array();  // Just use mode 0
            diffs_y = x_veh[k][t](1) - x_obs[v][0][k][t](1) - sample_y[k].array();
            distances = (diffs_x * diffs_x + diffs_y * diffs_y).sqrt();
          }

          for (int s = 0; s < S; s++)
          {
            if (distances(s) < static_dist)  // 0.625)
            {
              // std::cout << "\tcollision at k = " << k << ", v = " << v << ", s = " << s << " (d = " << distances(s)
              // << ")" << std::endl;
              sample_in_collision[s] = 1;
              sample_in_collision_stage[s] = 1;
            }
          }
        }
        // How many samples are in collision?
        double sum2 = 0.;
        for (int s = 0; s < S; s++)
          sum2 += (double)(sample_in_collision_stage[s]);

        stage_risk[t].push_back(sum2 / ((double)S));  // Compute the empirical risk PER STAGE
        // std::cout << "stage risk of " << sum2 / ((double)S) << " added.\n";
      }

      // How many samples are in collision?
      double sum = 0.;
      for (int s = 0; s < S; s++)
        sum += (double)(sample_in_collision[s]);

      risk.push_back(sum / ((double)S));  // Compute the empirical risk
                                          // std::cout << risk.back();
    }
    std::cout << "\n";

    DataSaver risk_saver;

    // Just for one case! (only works for one mode)
    if (compute_region)
    {
      int test_k = 0;

      ROS_WARN("Computing collision region as well (Only works for Gaussian and a single obstacle!)");
      ComputeCollisionRegion(sample_x[test_k], sample_y[test_k], x_obs[0][0][test_k][5], risk_saver);

      // Add the first stage in the plan to compare against
      for (auto& plan : x_veh[test_k])
        risk_saver.AddData("plan_0", plan);
    }
    else
    {
      ROS_WARN("Not computing the region (it is disabled)");
    }

    for (double r : risk)
      risk_saver.AddData("risk", r);

    for (int t = 0; t < data_size; t++)  // For all experiments
    {
      for (int k = 0; k < horizon_max; k++)
        risk_saver.AddData("risk_" + std::to_string(k), stage_risk[t][k]);
    }

    for (int reset_time : reset_times)
      risk_saver.AddData("reset_times", reset_time);

    for (auto& status : scalar_data["status"])
    {
      if (status != 2)
        risk_saver.AddData("infeasible", 1);
      else
        risk_saver.AddData("infeasible", 0);
    }

    // Load the robot state
    for (size_t t = 0; t < vector_data["vehicle_pose"].size(); t++)
      risk_saver.AddData("vehicle_pose", vector_data["vehicle_pose"][t]);

    // Load the robot velocity
    for (size_t t = 0; t < scalar_data["longitudinal_velocity"].size(); t++)
      risk_saver.AddData("velocity", scalar_data["longitudinal_velocity"][t]);

    // // Simple copy from the input data
    if (!is_ellipsoid)
    {
      for (int data_point : scalar_data["support_estimate"])
        risk_saver.AddData("support_estimate", data_point);

      risk_saver.AddData("max_support", scalar_data["max_support"][0]);
      risk_saver.AddData("sample_size", scalar_data["sample_size"][0]);
      risk_saver.AddData("removal_count", scalar_data["removal_count"][0]);
      risk_saver.AddData("max_iterations", scalar_data["max_iterations"][0]);
      risk_saver.AddData("risk_level", scalar_data["risk"][0]);
      risk_saver.AddData("confidence", scalar_data["confidence"][0]);

      // Load the cost evolution
      for (size_t j = 0; j < scalar_data["max_iterations"][0]; j++)
      {
        for (size_t t = 0; t < scalar_data["cost_" + std::to_string(j)].size(); t++)
          risk_saver.AddData("cost_" + std::to_string(j), scalar_data["cost_" + std::to_string(j)][t]);
      }

      // Load the final cost
      for (size_t t = 0; t < scalar_data["cost"].size(); t++)
        risk_saver.AddData("cost", scalar_data["cost"][t]);

      for (int data_point : scalar_data["solver_iterations"])
        risk_saver.AddData("solver_iterations", data_point);
    }
    else
    {
      ROS_WARN("Ellipsoid data. Not saving any support data.");
    }

    std::cout << S << std::endl;
    for (int k = 0; k < horizon_max; k++)
    {
      for (int s = 0; s < S; s++)
      {
        risk_saver.AddData("sample_x_" + std::to_string(k), Eigen::Vector2d(sample_x[k][s], sample_y[k][s]));
      }
    }

    risk_saver.SaveData(folder + "_computed_risk");

    // Visualize(sample_x, sample_y);
    // ros::spin();
  }

public:
  void Visualize(const std::vector<Eigen::VectorXd>& x, const std::vector<Eigen::VectorXd>& y)
  {
    // Plot all x and y with delays in between
    ROSPointMarker& sample = ros_markers_->getNewPointMarker("CUBE");
    sample.setScale(10.0, 10.0, 10.0);
    sample.setColor(1., 0., 0., 1.);

    geometry_msgs::Point p;
    for (int k = 0; k < horizon_max; k++)
    {
      for (int s = 0; s < 5; s++)
      {
        sample.setColor(1., 0., 0., 1.);

        p.x = x[k][s];
        p.y = y[k][s];
        p.z = 0.;
        sample.addPointMarker(Eigen::Vector3d(p.x, p.y, p.z));
      }
    }
    ros_markers_->publish();
  }

  std::vector<int> FindResets(const std::vector<double>& reset_data)
  {
    std::vector<int> reset_times;
    int last_t = -100;
    for (size_t t = 0; t < reset_data.size(); t++)
    {
      if (reset_data[t] == 1 && t - last_t > 50)
      {
        reset_times.push_back(t);
        last_t = t;
      }
    }

    return reset_times;
  }

  void GenerateSamples(std::vector<Eigen::VectorXd>& x, std::vector<Eigen::VectorXd>& y,
                       double noise)  // Samples for all k, s (same for v)
  {
    srand(time(NULL));

    RosTools::RandomGenerator rand_;
    double dt = 0.2;

    for (int k = 0; k < horizon_max; k++)
    {
      x.push_back(Eigen::VectorXd::Zero(S));
      y.push_back(Eigen::VectorXd::Zero(S));

      Eigen::Vector2d mean(0., 0.), sample;
      double major = noise, minor = noise, angle = 0;
      for (int s = 0; s < S; s++)
      {
        sample = rand_.BivariateGaussian(mean, major, minor, angle);  // dt here or below doesn't matter
        // std::cout << sample << std::endl;
        if (k == 0)
        {
          x[k](s) = sample(0) * dt;
          y[k](s) = sample(1) * dt;
        }
        else
        {
          x[k](s) = x[k - 1](s) + sample(0) * dt;
          y[k](s) = y[k - 1](s) + sample(1) * dt;
        }
      }
    }
  }

  void ConstructModeSamples(int v, int t, const int& num_modes,
                            const std::vector<std::vector<std::vector<double>>>& p_obs,
                            std::vector<int>& sample_counts_per_mode)
  {
    // SINCE IT CHANGES PER T AND V, THIS NEEDS TO HAPPEN DYNAMICALLY!
    // Generate a selection vector for the modes
    sample_counts_per_mode.clear();
    sample_counts_per_mode = std::vector<int>(num_modes, 0);

    // First, construct a list of summed probabilities
    double p_summed = 0.;
    static std::vector<double> sum_of_probabilities_;
    sum_of_probabilities_.clear();
    sum_of_probabilities_.resize(num_modes);
    for (int mode = 0; mode < num_modes; mode++)
    {
      p_summed += p_obs[v][mode][t];
      sum_of_probabilities_[mode] = p_summed;
    }

    // Then generate random numbers sequentially
    double mode_random;
    for (int s = 0; s < S; s++)
    {
      RosTools::RandomGenerator random_generator;
      mode_random = random_generator.Double();

      for (int mode = 0; mode < num_modes; mode++)
      {
        if (mode_random <= sum_of_probabilities_[mode])
        {
          sample_counts_per_mode[mode]++;  // We then sample one more for this mode
          break;
        }
      }
    }
  }

  /**
   * @brief Compute the collision space in 2D in which the risk is exceeded (for one stage!)
   *
   */
  void ComputeCollisionRegion(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::Vector2d& mean,
                              DataSaver& data_saver)
  {
    // Given the prediction with (mean, variance=static)
    // Validate the given samples for a grid
    std::vector<std::vector<double>> result;

    // Define the grid
    int grid_size = 250;
    double grid_spacing = 0.01;               // grid_size * grid_spacing = range
    double range = grid_size * grid_spacing;  // -1 because intervals require a value on either side
    result.resize(grid_size);

    Eigen::ArrayXd diffs_x, diffs_y, distances;
    diffs_x.resize(S);
    diffs_y.resize(S);
    distances.resize(S);

    for (int x_i = 0; x_i < grid_size; x_i++)
    {
      result[x_i].resize(grid_size);

      double x_grid = -range / 2.0 + range * ((double)(x_i) / ((double)grid_size));

      for (int y_i = 0; y_i < grid_size; y_i++)
      {
        double y_grid = -range / 2.0 + range * ((double)(y_i) / ((double)grid_size));

        // For the current x and y, evaluate all the samples, is it safe?
        diffs_x = x_grid - x.array();  // - mean applies the translation (not necessary if we translate it later)
        diffs_y = y_grid - y.array();
        distances = (diffs_x * diffs_x + diffs_y * diffs_y).sqrt();

        int collisions = 0;
        for (int s = 0; s < S; s++)
        {
          if (distances(s) < 0.325)
            collisions++;
        }

        result[x_i][y_i] = ((double)collisions) / ((double)S);
        if (collisions > 0)
          std::cout << "[" << x_i << ", " << y_i << "]: " << result[x_i][y_i] << std::endl;
      }
    }

    for (size_t i = 0; i < result.size(); i++)
    {
      for (size_t j = 0; j < result[i].size(); j++)
      {
        std::cout << "i: " << i << ", j: " << j << std::endl;
        data_saver.AddData("grid_" + std::to_string(i), result[i][j]);
      }
    }
    data_saver.AddData("grid_size", grid_size);
    data_saver.AddData("grid_spacing", grid_spacing);
    data_saver.AddData("grid_mean", mean);
  }

private:
  int S;
  std::string folder = "jackal_S-MPCC_05";
  bool is_binomial = false;
  bool is_ellipsoid = false;

  // Visuals
  ros::NodeHandle nh_;
  std::unique_ptr<ROSMarkerPublisher> ros_markers_;
};

#endif  // __RISK_COMPUTATION_H__