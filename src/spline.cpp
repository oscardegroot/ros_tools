#include "ros_tools/spline.h"

#include <ros_tools/logging.h>
#include <ros_tools/math.h>

namespace RosTools
{

    /** @note a spline parameterized by distance s */
    Spline2D::Spline2D(const std::vector<double> &x, const std::vector<double> &y)
    {
        // Compute the distance vector
        computeDistanceVector(x, y, _s_vector);

        // Initialize two splines for x and y
        _t_vector = _s_vector; // Spline in s

        _x_spline.set_points(_t_vector, x);
        _y_spline.set_points(_t_vector, y);
    }

    /** @note a spline parameterized over another vector t*/
    Spline2D::Spline2D(const tk::spline &x, const tk::spline &y, const std::vector<double> &t_vector)
        : _x_spline(x), _y_spline(y), _t_vector(t_vector)
    {
        computeDistanceVector(_x_spline.m_y_, _y_spline.m_y_, _s_vector); // Compute distances
    }

    Spline2D::Spline2D(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &t_vector)
        : _t_vector(t_vector)
    {

        _x_spline.set_points(t_vector, x);
        _y_spline.set_points(t_vector, y);

        computeDistanceVector(_x_spline.m_y_, _y_spline.m_y_, _s_vector); // Compute distances
    }

    Eigen::Vector2d Spline2D::getPoint(double t) const
    {
        return Eigen::Vector2d(_x_spline(t), _y_spline(t));
    }

    Eigen::Vector2d Spline2D::getVelocity(double t) const
    {
        return Eigen::Vector2d(_x_spline.deriv(1, t), _y_spline.deriv(1, t));
    }

    Eigen::Vector2d Spline2D::getAcceleration(double t) const
    {
        return Eigen::Vector2d(_x_spline.deriv(2, t), _y_spline.deriv(2, t));
    }

    Eigen::Vector2d Spline2D::getOrthogonal(double t) const
    {
        return Eigen::Vector2d(_y_spline.deriv(1, t), -_x_spline.deriv(1, t));
    }

    // Compute distances between points
    void Spline2D::computeDistanceVector(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &out)
    {
        out.clear();
        out.resize(x.size());
        out[0] = 0.;
        for (size_t i = 1; i < x.size(); i++)
        {
            double dist = std::sqrt(std::pow(x[i] - x[i - 1], 2.) + std::pow(y[i] - y[i - 1], 2.));
            out[i] = out[i - 1] + dist;
        }
    }

    void Spline2D::getParameters(int segment_index,
                                 double &ax, double &bx, double &cx, double &dx,
                                 double &ay, double &by, double &cy, double &dy) const
    {
        _x_spline.getParameters(segment_index, ax, bx, cx, dx);
        _y_spline.getParameters(segment_index, ay, by, cy, dy);
    }

    // Find the distance that we travelled on the spline
    void Spline2D::findClosestPoint(const Eigen::Vector2d &point, int &segment_out, double &t_out) const
    {
        t_out = findClosestSRecursively(point, 0., _t_vector.back(), 0);

        for (size_t i = 0; i < _t_vector.size() - 1; i++)
        {
            if (t_out > _t_vector[i] && t_out < _t_vector[i + 1])
            {
                segment_out = i; // Find the index to match the spline variable computed
                return;
            }
        }

        segment_out = _t_vector.size() - 1;
    }

    double Spline2D::findClosestSRecursively(const Eigen::Vector2d &point, double low, double high, int num_recursions) const
    {
        // Stop after x recursions
        if (num_recursions > 20)
        {
            if (std::abs(high - low) > 1e-3)
                LOG_ERROR("FindClosestSRecursively did not find an accurate s (accuracy = "
                          << std::abs(high - low) << " | tolerance = 1e-3)");
            return (low + high) / 2.;
        }

        // Computes the distance between point "s" on the spline and a vehicle position
        auto dist_to_spline = [&](double t, const Eigen::Vector2d &point)
        {
            return RosTools::distance(getPoint(t), point);
        };

        // Compute a middle s value
        double mid = (low + high) / 2.;

        // Compute the distance to the spline for high/low
        double value_low = dist_to_spline(low, point);
        double value_high = dist_to_spline(high, point);

        // Check the next closest value
        if (value_low < value_high)
            return findClosestSRecursively(point, low, mid, num_recursions + 1);
        else
            return findClosestSRecursively(point, mid, high, num_recursions + 1);
    }

    /** @note a spline parameterized by t_vector */
    Spline4D::Spline4D(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z, const std::vector<double> &w, const std::vector<double> &t_vector)
        : _t_vector(t_vector)
    {

        _x_spline.set_points(t_vector, x);
        _y_spline.set_points(t_vector, y);
        _z_spline.set_points(t_vector, z);
        _w_spline.set_points(t_vector, w);
    }

    Eigen::Vector4d Spline4D::getPoint(double t) const
    {
        return Eigen::Vector4d(_x_spline(t), _y_spline(t), _z_spline(t), _w_spline(t));
    }

    void Spline4D::getParameters(int segment_index,
                                 double &ax, double &bx, double &cx, double &dx,
                                 double &ay, double &by, double &cy, double &dy,
                                 double &az, double &bz, double &cz, double &dz,
                                 double &aw, double &bw, double &cw, double &dw) const
    {
        _x_spline.getParameters(segment_index, ax, bx, cx, dx);
        _y_spline.getParameters(segment_index, ay, by, cy, dy);
        _z_spline.getParameters(segment_index, az, bz, cz, dz);
        _w_spline.getParameters(segment_index, aw, bw, cw, dw);
    }
}
