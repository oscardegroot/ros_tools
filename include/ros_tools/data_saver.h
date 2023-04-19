//
// Class for exporting data from ROS to a .txt document, to be read by another program
// Useful for plotting etc.

// Usage: Make a DataSaver class
// Then use AddData(data_name, data) to add new data occurences
// When you want to save, use SaveData()

// You can use Clear() to clear the currently saved data, i.e., when you want to save in each iteration

// Example:
/*
DataSaver data_saver_test;
data_saver_test.AddData("num", 1);
data_saver_test.AddData("num", 2);
data_saver_test.AddData("num2", 3);
data_saver_test.SaveData("export_1");
*/
// The associated matlab file LoadROSData.m converts the generated files to a matlab structure

#ifndef DATA_SAVER_H
#define DATA_SAVER_H

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <ros/ros.h>
#include <vector>
namespace RosTools{

class DataSet
{
public:
  DataSet(const std::string &&name)
  {
    name_ = name;
    num_entries_ = 0;
  };
  DataSet(const std::string &name)
  {
    name_ = name;
    num_entries_ = 0;
  };

protected:
  std::string name_;
  int num_entries_;

public:
  virtual void AddData(const double &value){};
  virtual void AddData(const Eigen::Vector2d &value){};

  virtual void SaveData(std::ofstream &file) = 0;

  virtual void Clear() = 0;
};

class DoubleDataSet : public DataSet
{
public:
  DoubleDataSet(const std::string &&name) : DataSet(name) { data_.reserve(100); };

  DoubleDataSet(const std::string &name) : DataSet(name) { data_.reserve(100); };

protected:
  // Data needs to be convertible to double, there is no easy work around
  std::vector<double> data_;

public:
  void AddData(const double &value)
  {
    num_entries_++;
    data_.push_back(value);
  }

  void SaveData(std::ofstream &file)
  {
    // Print the number of entries
    file << name_.c_str() << ": " << 1 << " " << std::to_string(num_entries_) << "\n";

    for (size_t i = 0; i < data_.size(); i++)
    {
      file << std::fixed << std::setprecision(12) << data_[i] << "\n";
    }
  }

  void Clear()
  {
    num_entries_ = 0;
    data_.clear();
  }
};

class PointDataSet : public DataSet
{
public:
  PointDataSet(const std::string &&name) : DataSet(name) { data_.reserve(100); };
  PointDataSet(const std::string &name) : DataSet(name) { data_.reserve(100); };

protected:
  // Data needs to be convertible to double, there is no easy work around
  std::vector<Eigen::Vector2d> data_;

public:
  void AddData(const Eigen::Vector2d &value)
  {
    num_entries_++;
    data_.push_back(value);
  }

  void SaveData(std::ofstream &file)
  {
    // Print the number of entries
    file << name_.c_str() << ": " << 2 << " " << std::to_string(num_entries_) << "\n";

    for (size_t i = 0; i < data_.size(); i++)
    {
      file << std::fixed << std::setprecision(12) << data_[i](0) << " " << data_[i](1) << "\n";
    }
  }
  void Clear()
  {
    num_entries_ = 0;
    data_.clear();
  }
};

class DataSaver
{
public:
  DataSaver(double size = 20, bool add_timestamp = false)
  {
    datasets_.reserve(size);
    add_timestamp_ = add_timestamp;
  }

private:
  std::vector<std::unique_ptr<DataSet>> datasets_;

  std::map<std::string, int> data_lookup_;

  bool add_timestamp_;

  bool timestamp_set = false;
  std::chrono::system_clock::time_point start_time_;

  template <typename T> void ReadSingleDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<T>> &result)
  {
    double x;
    std::string data_name;
    int data_size, num_entries;

    while (import_file >> data_name >> data_size >> num_entries)
    {
      data_name = data_name.substr(0, data_name.find(":")); // Remove the colon

      result[data_name].reserve(num_entries);

      for (int i = 0; i < num_entries; i++)
      {
        import_file >> x;
        result[data_name].push_back(x);
      }
    }
  }

  void ReadDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<double>> &result) { ReadSingleDataFromFile(import_file, result); }

  void ReadDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<int>> &result) { ReadSingleDataFromFile(import_file, result); }

  void ReadDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<Eigen::Vector2d>> &result)
  {
    double x, y;
    std::string data_name;
    int data_size, num_entries;

    while (import_file >> data_name >> data_size >> num_entries)
    {
      data_name = data_name.substr(0, data_name.find(":")); // Remove the colon

      result[data_name].reserve(num_entries);
      for (int i = 0; i < num_entries; i++)
      {
        import_file >> x;
        import_file >> y;
        result[data_name].push_back(Eigen::Vector2d(x, y));
      }
    }
  }

public:
  template <typename T> void AddData(const std::string &&data_name, const T &data_value)
  {
    auto idx_it = data_lookup_.find(data_name);
    int idx;
    if (idx_it == data_lookup_.end())
    {
      if (datasets_.size() > 1e5)
        std::cout << "Warning: Data saver is not saving anymore data, too much different datasets were added already "
                     "(safety to prevent allocation errors!)"
                  << std::endl;

      // Create a new data set, if none exists with this name
      datasets_.emplace_back();

      // Depending on the datatype, add the data
      if (typeid(data_value) == typeid(const Eigen::Vector2d))
      {
        datasets_[datasets_.size() - 1].reset(new PointDataSet(data_name));
      }
      else
      {
        datasets_[datasets_.size() - 1].reset(new DoubleDataSet(data_name));
      }

      // Add its index to the map
      data_lookup_[data_name] = datasets_.size() - 1;

      // Add the data
      idx = datasets_.size() - 1;
    }
    else
    {
      idx = idx_it->second;
    }

    datasets_[idx]->AddData(data_value);
  }

  /* Save data
   * file_name: name of the file without folder or extension
   */
  void SaveData(const std::string &file_name)
  {
    std::string path = ros::package::getPath("lmpcc_tools") + "/scripts/data";
    SaveData(path, file_name);
  }

  // Use the given path
  void SaveData(const std::string &file_path, const std::string &file_name)
  {
    // Create directories if they do not exist
    std::string complete_file_path = file_path + "/" + file_name;
    std::string folder_path = complete_file_path.substr(0, complete_file_path.rfind("/"));

    if (boost::filesystem::create_directories(folder_path))
      ROS_INFO_STREAM("Data Saver: Creating Directory Path: " << folder_path);

    std::string full_file_path;
    if (add_timestamp_)
    {
      if (!timestamp_set)
      {
        start_time_ = std::chrono::system_clock::now();
        timestamp_set = true;
      }

      auto tt = std::chrono::system_clock::to_time_t(start_time_);
      auto local_time = *localtime(&tt);

      std::string datestamp = ParseYear(local_time.tm_year) + "_";
      datestamp += ValueWithZero(local_time.tm_mon + 1) + "_";
      datestamp += ValueWithZero(local_time.tm_mday);

      std::string timestamp = "";
      timestamp += ValueWithZero(local_time.tm_hour);
      timestamp += ValueWithZero(local_time.tm_min);

      full_file_path = complete_file_path + "/" + datestamp + "-" + timestamp + ".txt";
    }
    else
    {
      full_file_path = complete_file_path + ".txt";
    }

    // Setup a file stream
    std::ofstream export_file;

    ROS_INFO_STREAM("Data Saver: Saving collected data in " << full_file_path);

    export_file.open(full_file_path);

    for (auto &dataset : datasets_)
      dataset->SaveData(export_file);

    export_file << "-1\n";

    // Close the file
    export_file.close();
  }

  std::string ParseYear(int value) { return "20" + std::to_string(value).erase(0, 1); }

  std::string ValueWithZero(int value)
  {
    if (value < 10)
      return "0" + std::to_string(value);
    else
      return std::to_string(value);
  }

  template <class T> bool LoadData(const std::string &file_name, std::map<std::string, std::vector<T>> &result)
  {
    const std::string path = ros::package::getPath("lmpcc") + "/matlab_exports/data";
    return LoadData(path, file_name, result);
  }

  template <class T> bool LoadData(const std::string &file_path, const std::string &file_name, std::map<std::string, std::vector<T>> &result)
  {
    std::string full_file_path = file_path + "/" + file_name + ".txt";

    // Setup a file stream
    std::ifstream import_file(full_file_path);

    ROS_INFO_STREAM("Data Saver: Loading data from " << full_file_path);

    if (!import_file.good())
    {
      ROS_WARN("Data Saver: No file with this name was found.");
      return false;
    }

    ReadDataFromFile(import_file, result);

    return true;
  }

  // This is hacky
  bool LoadAllData(const std::string &file_path, const std::string &file_name, std::map<std::string, std::vector<double>> &result_scalar,
                   std::map<std::string, std::vector<Eigen::Vector2d>> &result_vector)
  {
    // Setup a file stream
    std::string full_file_path = file_path + "/" + file_name + ".txt";

    std::ifstream import_file(full_file_path);

    ROS_INFO_STREAM("Data Saver: Loading data from " << full_file_path);

    if (!import_file.good())
    {
      ROS_WARN("Data Saver: No file with this name was found.");
      return false;
    }

    double x, y;
    std::string data_name;
    int data_size, num_entries;

    while (import_file >> data_name >> data_size >> num_entries)
    {
      data_name = data_name.substr(0, data_name.find(":")); // Remove the colon
      if (data_size == 1)
      {
        result_scalar[data_name].reserve(num_entries);
        for (int i = 0; i < num_entries; i++)
        {
          import_file >> x;
          result_scalar[data_name].push_back((double)x);
        }
      }
      else
      {
        result_vector[data_name].reserve(num_entries);
        for (int i = 0; i < num_entries; i++)
        {
          import_file >> x;
          import_file >> y;
          result_vector[data_name].push_back(Eigen::Vector2d(x, y));
        }
      }
    }
    return true;
  }

  void Clear()
  {
    for (auto &dataset : datasets_)
      dataset->Clear();
  }

  void SetAddTimestamp(bool add_timestamp) { add_timestamp_ = add_timestamp; };
};
};

#endif