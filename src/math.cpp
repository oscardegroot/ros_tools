#include "ros_tools/math.h"

namespace RosTools
{
    double distance(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
    {
        return std::sqrt((a - b).transpose() * (a - b));
    }

    // Finds the exponential CDF value at probability p (for a rate of lambda)
    double ExponentialQuantile(double lambda, double p)
    {
        return -std::log(1 - p) / lambda;
    }

    Eigen::Matrix2d rotationMatrixFromHeading(double heading)
    {
        Eigen::Matrix2d result;
        result << std::cos(heading), std::sin(heading), -std::sin(heading), std::cos(heading);

        return result;
    }

    double Bisection(double low, double high, std::function<double(double)> func, double tol)
    {
        if (low > high)
            throw std::runtime_error("Bisection low value was higher than the high value!");

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
        }

        throw std::runtime_error("Bisection failed!");
    }

} // namespace RosTools