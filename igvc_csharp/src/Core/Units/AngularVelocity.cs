using System.Text.Json;
using igvc_csharp.Core.Config;

namespace igvc_csharp.Core.Units;

using System;
using System.Globalization;

public struct AngularVelocity : IEquatable<AngularVelocity>, IComparable<AngularVelocity>, IConfigSerializable
{
    private double RadiansPerSecond { set; get; }

    internal AngularVelocity(double radiansPerSecond)
    {
        RadiansPerSecond = radiansPerSecond;
    }
    
    public double To(AngularVelocityUnit unit) => unit.FromRadiansPerSecond(RadiansPerSecond);

    public string ToString(
        AngularVelocityUnit unit,
        string? format = null,
        IFormatProvider? provider = null)
    {
        provider ??= CultureInfo.InvariantCulture;
        return To(unit).ToString(format, provider) + " " + unit.Symbol;
    }

    public override string ToString()
        => ToString(AngularVelocityUnit.RadiansPerSecond);

    public bool Equals(AngularVelocity other) => RadiansPerSecond.Equals(other.RadiansPerSecond);

    public override bool Equals(object? obj) => obj is AngularVelocity other && Equals(other);

    public override int GetHashCode() => RadiansPerSecond.GetHashCode();

    public static bool operator ==(AngularVelocity a, AngularVelocity b) => a.Equals(b);

    public static bool operator !=(AngularVelocity a, AngularVelocity b) => !a.Equals(b);

    public int CompareTo(AngularVelocity other) => RadiansPerSecond.CompareTo(other.RadiansPerSecond);

    public static bool operator <(AngularVelocity a, AngularVelocity b) => a.RadiansPerSecond < b.RadiansPerSecond;

    public static bool operator >(AngularVelocity a, AngularVelocity b) => a.RadiansPerSecond > b.RadiansPerSecond;

    public static bool operator <=(AngularVelocity a, AngularVelocity b) => a.RadiansPerSecond <= b.RadiansPerSecond;

    public static bool operator >=(AngularVelocity a, AngularVelocity b) => a.RadiansPerSecond >= b.RadiansPerSecond;

    public static AngularVelocity operator +(AngularVelocity a, AngularVelocity b) =>
        new(a.RadiansPerSecond + b.RadiansPerSecond);

    public static AngularVelocity operator -(AngularVelocity a, AngularVelocity b) =>
        new(a.RadiansPerSecond - b.RadiansPerSecond);

    public static AngularVelocity operator *(AngularVelocity v, double scalar) => new(v.RadiansPerSecond * scalar);

    public static AngularVelocity operator /(AngularVelocity v, double scalar) => new(v.RadiansPerSecond / scalar);

    public static Angle operator *(AngularVelocity v, Time t) => new Angle(v.RadiansPerSecond * t.Seconds);

    public object Serialize() => new { rps = RadiansPerSecond };

    public object Deserialize(object value)
    {
        var obj = (JsonElement)value;
        RadiansPerSecond = obj.GetProperty("rps").GetDouble();
        return this;
    }
}