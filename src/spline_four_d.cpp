#include "ros_tools/spline.h"

#include <ros_tools/logging.h>
#include <ros_tools/math.h>

namespace RosTools
{

    /** @note a spline parameterized by t_vector */
    Spline4D::Spline4D(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z, const std::vector<double>& w, const std::vector<double> &t_vector)
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
