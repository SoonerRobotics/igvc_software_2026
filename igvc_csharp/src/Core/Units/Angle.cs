using System.Text.Json;
using igvc_csharp.Core.Config;
using igvc_csharp.Utils;

namespace igvc_csharp.Core.Units;

public struct Angle : IEquatable<Angle>, IComparable<Angle>, IConfigSerializable
{
    private double Radians { get; set; }

    internal Angle(double angle, bool isRadians = true)
    {
        if (isRadians)
        {
            Radians = angle;
        }
        else
        {
            Radians = MathUtils.ToRadians(angle);
        }
    }

    public double To(AngleUnit unit) => unit.FromRadians(Radians);

    public bool Equals(Angle other) => Radians.Equals(other.Radians);

    public override bool Equals(object? obj) => obj is Angle other && Equals(other);

    public override int GetHashCode() => Radians.GetHashCode();

    public static bool operator ==(Angle a, Angle b) => a.Equals(b);

    public static bool operator !=(Angle a, Angle b) => !a.Equals(b);

    public int CompareTo(Angle other) => Radians.CompareTo(other.Radians);

    public static bool operator <(Angle a, Angle b) => a.Radians < b.Radians;

    public static bool operator >(Angle a, Angle b) => a.Radians > b.Radians;

    public static bool operator <=(Angle a, Angle b) => a.Radians <= b.Radians;

    public static bool operator >=(Angle a, Angle b) => a.Radians >= b.Radians;

    public static Angle operator +(Angle a, Angle b) => new(a.Radians + b.Radians);

    public static Angle operator -(Angle a, Angle b) => new(a.Radians - b.Radians);

    public static Angle operator *(Angle a, double scalar) => new(a.Radians * scalar);

    public static Angle operator /(Angle a, double scalar) => new(a.Radians / scalar);

    public override string ToString() => $"{Radians} rad";

    public object Serialize() => new { radians = Radians };
    public object Deserialize(object value)
    {
        var obj = (JsonElement)value;
        Radians = obj.GetProperty("radians").GetDouble();
        return this;
    }
}