using igvc_csharp.Core.Units;
using igvc_csharp.Utils;

namespace igvc_csharp.src.Subsystems.Feelers;

public struct SCR_Point : IEquatable<SCR_Point>, IComparable<SCR_Point>
{
    public int X { get; set; } = 0;
    public int Y { get; set; } = 0;

    public SCR_Point() : this(0, 0) {}

    public SCR_Point(int x, int y)
    {
        X = x;
        Y = y;
    }

    // expects theta to be in radians
    public SCR_Point(double rho, double theta)
    {
        X = (int)(rho * Math.Cos(theta));
        Y = (int)(rho * Math.Sin(theta));
    }

    public SCR_Point(double rho, Angle theta)
    {
        X = (int)(rho * Math.Cos(theta.To(AngleUnit.Radians)));
        Y = (int)(rho * Math.Sin(theta.To(AngleUnit.Radians)));
    }

    public double Dist(SCR_Point other)
    {
        //FIXME double check if these are in the right order ?
        return Math.Sqrt(
            Math.Pow(X - other.X, 2) + Math.Pow(Y - other.Y, 2)
        );
    }

    public double GetRadius()
    {
        return Dist(new SCR_Point(0, 0));
    }

    public readonly OpenCvSharp.Point GetOpenCvPoint() => new OpenCvSharp.Point(X, Y);

    public readonly double Angle => Math.Atan2(Y, X);

    public bool Equals(SCR_Point other) => X.Equals(other.X) && Y.Equals(other.Y);

    public override bool Equals(object? obj) => obj is SCR_Point other && Equals(other);
    public override int CompareTo(SCR_Point other) => (int)(other.Angle - Angle);

    public static bool operator ==(SCR_Point a, SCR_Point b) => a.Equals(b);

    public static bool operator !=(SCR_Point a, SCR_Point b) => !a.Equals(b);

    public static bool operator <(SCR_Point a, SCR_Point b) => a.GetRadius() < b.GetRadius();

    public static bool operator >(SCR_Point a, SCR_Point b) => a.GetRadius() > b.GetRadius();

    public static bool operator <=(SCR_Point a, SCR_Point b) => a.GetRadius() <= b.GetRadius();

    public static bool operator >=(SCR_Point a, SCR_Point b) => a.GetRadius() >= b.GetRadius();

    public static SCR_Point operator +(SCR_Point a, SCR_Point b) => new(a.X + b.X, a.Y + b.Y);

    public static SCR_Point operator -(SCR_Point a, SCR_Point b) => new(a.X - b.X, a.Y - b.Y);

    public static SCR_Point operator *(SCR_Point a, double scalar) => new(a.X * scalar, a.Y * scalar);

    public static SCR_Point operator /(SCR_Point a, double scalar) => new(a.X / scalar, a.Y / scalar);
}