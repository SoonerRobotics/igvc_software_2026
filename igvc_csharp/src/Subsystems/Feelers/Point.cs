
using igvc_csharp.Core.Units;
using igvc_csharp.Utils;

namespace igvc_csharp.Subsystems.Feelers;

public struct Point : IEquatable<Point>, IComparable<Point>
{
    public int X { get; set; } = 0;
    public int Y { get; set; } = 0;

    public Point() : this(0, 0) {}

    public Point(int x, int y)
    {
        X = x;
        Y = y;
    }

    // expects theta to be in radians
    public Point(double rho, double theta)
    {
        X = (int)(rho * Math.Cos(theta));
        Y = (int)(rho * Math.Sin(theta));
    }

    public Point(double rho, Angle theta)
    {
        X = (int)(rho * Math.Cos(theta.To(AngleUnit.Radians)));
        Y = (int)(rho * Math.Sin(theta.To(AngleUnit.Radians)));
    }

    public double Dist(Point other)
    {
        //FIXME double check if these are in the right order ?
        return Math.Sqrt(
            Math.Pow(X - other.X, 2) + Math.Pow(Y - other.Y, 2)
        );
    }

    public double GetRadius()
    {
        return Dist(new Point(0, 0));
    }

    public readonly OpenCvSharp.Point GetOpenCvPoint() => new OpenCvSharp.Point(X, Y);

    public readonly double Angle => Math.Atan2(Y, X);

    public bool Equals(Point other) => X.Equals(other.X) && Y.Equals(other.Y);

    public override bool Equals(object? obj) => obj is Point other && Equals(other);

    public static bool operator ==(Angle a, Angle b) => a.Equals(b);

    public static bool operator !=(Angle a, Angle b) => !a.Equals(b);

    //TODO FIXME
    // public int CompareTo(Point other) => Radians.CompareTo(other.Radians);

    public static bool operator <(Point a, Point b) => a.GetRadius() < b.GetRadius();

    public static bool operator >(Point a, Point b) => a.GetRadius() > b.GetRadius();

    public static bool operator <=(Point a, Point b) => a.GetRadius() <= b.GetRadius();

    public static bool operator >=(Point a, Point b) => a.GetRadius() >= b.GetRadius();

    public static Point operator +(Point a, Point b) => new(a.X + b.X, a.Y + b.Y);

    public static Point operator -(Point a, Point b) => new(a.X - b.X, a.Y - b.Y);

    public static Point operator *(Point a, double scalar) => new(a.X * scalar, a.Y * scalar);

    public static Point operator /(Point a, double scalar) => new(a.X / scalar, a.Y / scalar);
}