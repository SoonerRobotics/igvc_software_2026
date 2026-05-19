using System.Text.Json;
using igvc_csharp.Core.Config;

namespace igvc_csharp.Core.Units;

using System;
using System.Globalization;

public struct Distance : IEquatable<Distance>, IConfigSerializable
{
    private double Meters { set; get; }
    
    internal Distance(double meters)
    {
        Meters = meters;
    }

    public static Distance From(double x, DistanceUnit unit)
    {
        return new Distance(1 / unit.FromMeters(x));
    }


    public double To(DistanceUnit unit)
        => unit.FromMeters(Meters);

    public string ToString(DistanceUnit unit, string? format = null, IFormatProvider? provider = null)
    {
        provider ??= CultureInfo.InvariantCulture;
        var value = To(unit);
        return value.ToString(format, provider) + " " + unit.Symbol;
    }

    public override string ToString() => ToString(DistanceUnit.Meters);

    public bool Equals(Distance other) => Meters.Equals(other.Meters);

    public override bool Equals(object? obj) => obj is Distance other && Equals(other);

    public override int GetHashCode() => Meters.GetHashCode();

    public static bool operator ==(Distance left, Distance right) => left.Equals(right);

    public static bool operator !=(Distance left, Distance right) => !left.Equals(right);

    public static Distance operator +(Distance a, Distance b) => new(a.Meters + b.Meters);

    public static Distance operator -(Distance a, Distance b) => new(a.Meters - b.Meters);

    public static Distance operator *(Distance d, double scalar) => new(d.Meters * scalar);

    public static Distance operator *(double scalar, Distance d) => d * scalar;

    public static Distance operator /(Distance d, double scalar) => new(d.Meters / scalar);

    public static LinearVelocity operator /(Distance d, Time t) => new LinearVelocity(d.Meters / t.Seconds);
    
    public object Serialize() => new { meters = Meters };
    public object Deserialize(object value)
    {
        var obj = (JsonElement)value;
        Meters = obj.GetProperty("meters").GetDouble();
        return this;
    }
}