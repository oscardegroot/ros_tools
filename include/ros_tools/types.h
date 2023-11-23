#ifndef __ROSTOOLS_TYPES_H__
#define __ROSTOOLS_TYPES_H__

#include <Eigen/Dense>

namespace RosTools
{

    /** Virtual class for splines in 2D (x, y)*/
    class Spline2D
    {
    public:
        virtual Eigen::Vector2d GetPoint(double s) const = 0;
        virtual Eigen::Vector2d GetVelocity(double s) const = 0;
        virtual Eigen::Vector2d GetAcceleration(double s) const = 0;
        virtual Eigen::Vector2d GetOrthogonal(double s) const = 0;
        virtual void GetParameters(int index, double &ax, double &bx, double &cx, double &dx, double &ay, double &by,
                                   double &cy, double &dy) = 0;

        virtual double GetSplineStart(int index) = 0;
        virtual double GetSplineEnd(int index) = 0;
        virtual double GetSplineLength() = 0;
        virtual int NumberOfSegments() = 0;
        virtual void Print() = 0;
    };

    /** Combined definition of 2D cubic splines (x, y), can use multiple types of classes as long as the () operator
     * evaluates the spline*/
    template <class T>
    class CubicSpline2D : public Spline2D
    {
    public:
        CubicSpline2D(const T &spline_x, const T &spline_y) : spline_x_(spline_x), spline_y_(spline_y)
        {
        }

        CubicSpline2D(const CubicSpline2D<T> &other) // copy constructor
        {
            spline_x_ = other.spline_x_;
            spline_y_ = other.spline_y_;
        }

        Eigen::Vector2d GetPoint(double s) const override
        {
            return Eigen::Vector2d(spline_x_(s), spline_y_(s));
        }

        Eigen::Vector2d GetVelocity(double s) const override
        {
            return Eigen::Vector2d(spline_x_.deriv(1, s), spline_y_.deriv(1, s));
        }

        Eigen::Vector2d GetAcceleration(double s) const override
        {
            return Eigen::Vector2d(spline_x_.deriv(2, s), spline_y_.deriv(2, s));
        }

        Eigen::Vector2d GetOrthogonal(double s) const override
        {
            return Eigen::Vector2d(-spline_y_.deriv(1, s), spline_x_.deriv(1, s));
        }

        void GetParameters(int index, double &ax, double &bx, double &cx, double &dx, double &ay, double &by, double &cy,
                           double &dy) override
        {
            spline_x_.GetParameters(index, ax, bx, cx, dx);
            spline_y_.GetParameters(index, ay, by, cy, dy);
        }

        double GetSplineStart(int index) override
        {
            return spline_x_.GetSplineStart(index);
        }

        double GetSplineEnd(int index) override
        {
            return spline_x_.GetSplineEnd(index);
        }

        double GetSplineLength() override
        {
            return spline_x_.m_x_.back();
        }

        int NumberOfSegments() override
        {
            return spline_x_.m_a.size() - 1; // Don't use the last segment (it is an extrapolation to the right)
        }

        void Print() override
        {
            spline_x_.Print();
            spline_y_.Print();
        }

    private:
        T spline_x_, spline_y_;
    };

    struct Halfspace
    {
        Eigen::Vector2d A_;
        double b_;

        Halfspace(){};

        Halfspace(const Eigen::Vector2d &A, const double b) : A_(A), b_(b) {}

        static Halfspace &Dummy()
        {
            static Halfspace dummy(Eigen::Vector2d(1, 0), 1000);
            return dummy;
        }
    };
};

#endif