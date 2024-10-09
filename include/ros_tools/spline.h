#ifndef ros_tools_SPLINE_H
#define ros_tools_SPLINE_H

#include <third_party/tkspline.h>

#include <Eigen/Dense>

namespace RosTools
{
    class Spline2D
    {

    public:
        Spline2D(const std::vector<double> &x, const std::vector<double> &y);
        Spline2D(const tk::spline &x, const tk::spline &y, const std::vector<double> &t_vector);

        Spline2D(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &t_vector);

        Eigen::Vector2d getPoint(double t) const;
        double getX(double t) const;
        double getY(double t) const;

        Eigen::Vector2d getVelocity(double t) const;
        Eigen::Vector2d getAcceleration(double t) const;
        Eigen::Vector2d getJerk(double t) const;
        Eigen::Vector2d getOrthogonal(double t) const;
        double getYaw(double s) const;
        double getCurvature(double s) const;
        double getCurvatureDeriv(double s) const;

        void samplePoints(std::vector<Eigen::Vector2d> &points, double ds) const;
        void samplePoints(std::vector<Eigen::Vector2d> &points, std::vector<double> &angles, double ds) const;

        /** @brief Check the entire spline for the closest point */
        void initializeClosestPoint(const Eigen::Vector2d &point, int &segment_out, double &t_out);
        void findClosestPoint(const Eigen::Vector2d &point, int &segment_out, double &t_out, int range = 2);

        void getParameters(int segment_index,
                           double &ax, double &bx, double &cx, double &dx,
                           double &ay, double &by, double &cy, double &dy) const;

        int numSegments() const { return _x_spline.m_x_.size() - 1; }
        double getSegmentStart(int index) const;
        // double getSegmentEnd(int index) const { return _s_vector[index]; };
        double length() const { return _s_vector.back(); }
        double parameterLength() const { return _t_vector.back(); }



        tk::spline &getXSpline() { return _x_spline; }
        tk::spline &getYSpline() { return _y_spline; }
        std::vector<double> &getTVector() { return _t_vector; }
        std::vector<double> &getDistanceVector() { return _s_vector; }

    private:
        tk::spline _x_spline, _y_spline;

        std::vector<double> _t_vector; // Both splines are defined over another parameter t
        std::vector<double> _s_vector; // Holds distances at which each spline begins

        // Finding the closest point
        int _closest_segment{-1};
        Eigen::Vector2d _prev_query_point;

        void computeDistanceVector(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &out);

        double findClosestSRecursively(const Eigen::Vector2d &point, double low, double high, int num_recursions) const;
    };

    class Spline4D
    {

    public:
        Spline4D(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z, const std::vector<double> &w, const std::vector<double> &t_vector);

        Eigen::Vector4d getPoint(double t) const;

        void getParameters(int segment_index,
                           double &ax, double &bx, double &cx, double &dx,
                           double &ay, double &by, double &cy, double &dy,
                           double &az, double &bz, double &cz, double &dz,
                           double &aw, double &bw, double &cw, double &dw) const;

        int numSegments() const { return _x_spline.m_x_.size() - 1; }
        double getSegmentStart(int index) const { return _t_vector[index]; };
        // double getSegmentEnd(int index) const { return _s_vector[index]; };
        double length() const { return _s_vector.back(); }
        double parameterLength() const { return _t_vector.back(); }

        tk::spline &getXSpline() { return _x_spline; }
        tk::spline &getYSpline() { return _y_spline; }
        tk::spline &getZSpline() { return _z_spline; }
        tk::spline &getWSpline() { return _w_spline; }

    private:
        tk::spline _x_spline, _y_spline, _z_spline, _w_spline;

        std::vector<double> _t_vector; // Both splines are defined over another parameter t
        std::vector<double> _s_vector; // Holds distances at which each spline begins
    };
}

#endif // MPC_PLANNER_UTIL_SPLINE_H
