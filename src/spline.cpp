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

    Spline2D::Spline2D(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &t)
        : _t_vector(t)
    {
        // Compute the distance vector
        computeDistanceVector(x, y, _s_vector);

        // Initialize two splines for x and y
        _x_spline.set_points(_t_vector, x);
        _y_spline.set_points(_t_vector, y);
    }

    Eigen::Vector2d Spline2D::getPoint(double t) const
    {
        return Eigen::Vector2d(_x_spline(t), _y_spline(t));
    }

    double Spline2D::getX(double t) const
    {
        return _x_spline(t);
    }

    double Spline2D::getY(double t) const
    {
        return _y_spline(t);
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
        return Eigen::Vector2d(_y_spline.deriv(1, t), -_x_spline.deriv(1, t)).normalized();
    }

    double Spline2D::getPathAngle(double t) const
    {
        return std::atan2(_y_spline.deriv(1, t), _x_spline.deriv(1, t));
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
        // Extrapolate with a constant path based on the last segment
        if (segment_index > numSegments() - 1)
        {
            getParameters(numSegments() - 1,
                          ax, bx, cx, dx,
                          ay, by, cy, dy);

            ax = 0.;
            bx = 0.;
            cx = 0.;

            ay = 0.;
            by = 0.;
            cy = 0.;

            return;
        }

        _x_spline.getParameters(segment_index, ax, bx, cx, dx);
        _y_spline.getParameters(segment_index, ay, by, cy, dy);
    }

    double Spline2D::getSegmentStart(int segment_index) const
    {
        if (segment_index > numSegments() - 1)
            return _t_vector.back();
        else
            return _t_vector[segment_index];
    }

    void Spline2D::initializeClosestPoint(const Eigen::Vector2d &point, int &segment_out, double &t_out)
    {
        // Computes the distance between point "s" on the spline and a vehicle position
        auto dist_to_spline = [&](double t, const Eigen::Vector2d &point)
        {
            return RosTools::distance(getPoint(t), point);
        };

        double min_dist = 1e9;
        int local_segment_out = -1;
        double local_t_out = -1.;
        for (size_t i = 0; i < _t_vector.size() - 1; i++)
        {
            double cur_t = findClosestSRecursively(point, _t_vector[i], _t_vector[i + 1], 10); // Closest in this segment

            double cur_dist = dist_to_spline(cur_t, point);
            if (cur_dist < min_dist)
            {
                min_dist = cur_dist;
                local_t_out = cur_t;
                local_segment_out = i;
            }
        }

        ROSTOOLS_ASSERT(local_segment_out != -1, "Could not find a closest point on the spline");
        segment_out = local_segment_out;
        t_out = local_t_out;
        _closest_segment = segment_out;
    }

    // Find the distance that we travelled on the spline
    void Spline2D::findClosestPoint(const Eigen::Vector2d &point, int &segment_out, double &t_out, int range)
    {
        if (_closest_segment == -1 || RosTools::distance(_prev_query_point, point) > 5.) // Non-initialized
        {
            // LOG_INFO("Initialize Closest Point");
            initializeClosestPoint(point, segment_out, t_out);
            _prev_query_point = point;

            return;
        }
        _prev_query_point = point;

        int first_segment = std::max(0, _closest_segment - range);
        int last_segment = std::min((int)_t_vector.size() - 1, _closest_segment + range);

        // Search locally
        t_out = findClosestSRecursively(point,
                                        _t_vector[first_segment],
                                        _t_vector[last_segment], 0);
        // LOG_INFO("Recursive search between " << _t_vector[first_segment] << " and " << _t_vector[last_segment] << ": " << t_out);

        for (int i = first_segment; i < last_segment; i++)
        {
            if (t_out > _t_vector[i] && t_out < _t_vector[i + 1])
            {
                segment_out = i; // Find the index to match the spline variable computed
                _closest_segment = segment_out;

                return;
            }
        }

        _closest_segment = segment_out;
        segment_out = last_segment;
    }

    double Spline2D::findClosestSRecursively(const Eigen::Vector2d &point, double low, double high, int num_recursions) const
    {
        if (std::abs(high - low) <= 1e-4 || num_recursions > 40)
        {
            if (num_recursions > 40)
                LOG_WARN_THROTTLE(1500, "Recursion count exceeded.");

            // LOG_INFO("Difference between " << high << " and " << low << " < 1e-4. Returning " << (low + high) / 2.);
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

    void Spline2D::samplePoints(std::vector<Eigen::Vector2d> &points, double ds) const
    {
        std::vector<double> angles;
        samplePoints(points, angles, ds);
    }

    void Spline2D::samplePoints(std::vector<Eigen::Vector2d> &points, std::vector<double> &angles, double ds) const
    {
        points.clear();
        angles.clear();
        double length = this->length();

        double spline_sample_dist = std::min(ds, length);
        int n_spline_pts = ceil(length / spline_sample_dist);
        points.resize(n_spline_pts);
        angles.resize(n_spline_pts);

        double s_cur = 0;
        for (int i = 0; i < n_spline_pts; i++)
        {
            Eigen::Vector2d vel = getVelocity(s_cur);

            points[i] = getPoint(s_cur);
            angles[i] = std::atan2(vel(1), vel(0));

            s_cur += spline_sample_dist;
        }

        // Check if we are not at our destination yet
        double error = RosTools::distance(points.back(), getPoint(length));
        if (error > 0.01)
        {
            Eigen::Vector2d vel = getVelocity(length);

            points.emplace_back(getPoint(length));
            angles.emplace_back(std::atan2(vel(1), vel(0)));
        }
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

    template <int dim>
    Spline<dim>::Spline(const std::vector<std::vector<double>> &points)
    {
        // Compute the distance vector
        computeDistanceVector(points, _s_vector);

        // Initialize two splines for x and y
        _t_vector = _s_vector; // Spline in s

        _splines.resize(dim);
        for (int i = 0; i < dim; i++)
            _splines[i].set_points(_t_vector, points[i]);
    }

    template <int dim>
    Spline<dim>::Spline(const std::vector<tk::spline> &splines, const std::vector<double> &t_vector)
    {
        _splines = splines;
        _t_vector = t_vector;
    }

    template <int dim>
    Spline<dim>::Spline(const std::vector<std::vector<double>> &points, const std::vector<double> &t_vector)
        : _t_vector(t_vector)
    {
        // Compute the distance vector
        computeDistanceVector(points, _s_vector);

        // Initialize two splines for x and y
        _splines.resize(dim);
        for (int i = 0; i < dim; i++)
            _splines[i].set_points(_t_vector, points[i]);
    }

    template <int dim>
    Spline<dim>::Spline(const std::vector<std::vector<double>> &points, const std::vector<double> &t_vector,
                        const SplineVector &start_velocity)
        : _t_vector(t_vector)
    {
        // Compute the distance vector
        computeDistanceVector(points, _s_vector);

        // Initialize two splines for x and y
        _splines.resize(dim);
        for (int i = 0; i < dim; i++)
        {
            _splines[i].set_boundary(tk::spline::first_deriv, start_velocity(i), tk::spline::second_deriv, 0.);
            _splines[i].set_points(_t_vector, points[i]);
        }
    }

    template <int dim>
    typename Spline<dim>::SplineVector Spline<dim>::getPoint(double t) const
    {
        SplineVector result;
        for (int i = 0; i < dim; i++)
            result(i) = _splines[i](t);
        return result;
    }

    template <int dim>
    double Spline<dim>::getCoordinate(double t, int coordinate) const
    {
        return _splines[coordinate](t);
    }

    template <int dim>
    typename Spline<dim>::SplineVector Spline<dim>::getVelocity(double t) const
    {
        SplineVector result;
        for (int i = 0; i < dim; i++)
            result(i) = _splines[i].deriv(1, t);
        return result;
    }

    template <int dim>
    typename Spline<dim>::SplineVector Spline<dim>::getAcceleration(double t) const
    {
        SplineVector result;
        for (int i = 0; i < dim; i++)
            result(i) = _splines[i].deriv(2, t);
        return result;
    }

    template <int dim>
    typename Spline<dim>::SplineVector Spline<dim>::getOrthogonal(double t) const
    {
        SplineVector point = getPoint(t);
        Eigen::VectorXd orth = point;
        return orth.unitOrthogonal();
    }

    template <int dim>
    void Spline<dim>::initializeClosestPoint(const SplineVector &point, int &segment_out, double &t_out)
    {
        // Computes the distance between point "s" on the spline and a vehicle position
        auto dist_to_spline = [&](double t, const SplineVector &point)
        {
            return (getPoint(t) - point).norm();
        };

        double min_dist = 1e9;
        int local_segment_out = -1;
        double local_t_out = -1.;
        for (size_t i = 0; i < _t_vector.size() - 1; i++)
        {
            double cur_t = findClosestSRecursively(point, _t_vector[i], _t_vector[i + 1], 10); // Closest in this segment

            double cur_dist = dist_to_spline(cur_t, point);
            if (cur_dist < min_dist)
            {
                min_dist = cur_dist;
                local_t_out = cur_t;
                local_segment_out = i;
            }
        }

        ROSTOOLS_ASSERT(local_segment_out != -1, "Could not find a closest point on the spline");
        segment_out = local_segment_out;
        t_out = local_t_out;
        _closest_segment = segment_out;
    }

    template <int dim>
    void Spline<dim>::findClosestPoint(const SplineVector &point, int &segment_out, double &t_out, int range)
    {
        if (_closest_segment == -1 || (_prev_query_point - point).norm() > 5.) // Non-initialized
        {
            // LOG_INFO("Initialize Closest Point");
            initializeClosestPoint(point, segment_out, t_out);
            _prev_query_point = point;

            return;
        }
        _prev_query_point = point;

        int first_segment = std::max(0, _closest_segment - range);
        int last_segment = std::min((int)_t_vector.size() - 1, _closest_segment + range);

        // Search locally
        t_out = findClosestSRecursively(point,
                                        _t_vector[first_segment],
                                        _t_vector[last_segment], 0);
        // LOG_INFO("Recursive search between " << _t_vector[first_segment] << " and " << _t_vector[last_segment] << ": " << t_out);

        for (int i = first_segment; i < last_segment; i++)
        {
            if (t_out > _t_vector[i] && t_out < _t_vector[i + 1])
            {
                segment_out = i; // Find the index to match the spline variable computed
                _closest_segment = segment_out;

                return;
            }
        }

        _closest_segment = segment_out;
        segment_out = last_segment;
    }

    template <int dim>
    void Spline<dim>::getParameters(int segment_index,
                                    std::vector<double> &a, std::vector<double> &b,
                                    std::vector<double> &c, std::vector<double> &d) const
    {
        // Extrapolate with a constant path based on the last segment
        if (segment_index > numSegments() - 1)
        {
            getParameters(numSegments() - 1, a, b, c, d);

            for (int i = 0; i < dim; i++)
            {
                a[i] = 0.;
                b[i] = 0.;
                c[i] = 0.;
            }

            return;
        }

        a.resize(dim);
        b.resize(dim);
        c.resize(dim);
        d.resize(dim);

        for (int i = 0; i < dim; i++)
            _splines[i].getParameters(segment_index, a[i], b[i], c[i], d[i]);
    }

    template <int dim>
    double Spline<dim>::getSegmentStart(int index) const
    {
        if (index > numSegments() - 1)
            return _t_vector.back();
        else
            return _t_vector[index];
    }

    template <int dim>
    void Spline<dim>::computeDistanceVector(const std::vector<std::vector<double>> &points,
                                            std::vector<double> &out)
    {
        out.clear();
        out.resize(points[0].size());
        out[0] = 0.;
        for (size_t i = 1; i < points[0].size(); i++)
        {
            SplineVector a, b;
            for (int d = 0; d < dim; d++)
            {
                a(d) = points[d][i - 1];
                b(d) = points[d][i];
            }
            double dist = (b - a).norm();
            out[i] = out[i - 1] + dist;
        }
    }

    template <int dim>
    double Spline<dim>::findClosestSRecursively(const SplineVector &point,
                                                double low, double high,
                                                int num_recursions) const
    {
        if (std::abs(high - low) <= 1e-4 || num_recursions > 40)
        {
            if (num_recursions > 40)
                LOG_WARN_THROTTLE(1500, "Recursion count exceeded.");

            // LOG_INFO("Difference between " << high << " and " << low << " < 1e-4. Returning " << (low + high) / 2.);
            return (low + high) / 2.;
        }

        // Computes the distance between point "s" on the spline and a vehicle position
        auto dist_to_spline = [&](double t, const SplineVector &point)
        {
            return (getPoint(t) - point).norm();
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

    // // Explicit template instantiation
    template class Spline<2>;
    // template class Spline<3>;
    // template class Spline<4>;
}
