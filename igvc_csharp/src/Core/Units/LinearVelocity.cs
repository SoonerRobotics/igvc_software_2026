using System.Globalization;
using System.Text.Json;
using igvc_csharp.Core.Config;

namespace igvc_csharp.Core.Units;

public struct LinearVelocity : IEquatable<LinearVelocity>, IComparable<LinearVelocity>, IConfigSerializable
{
    private double MetersPerSecond { get; set; }
    
    internal LinearVelocity(double metersPerSecond)
    {
        MetersPerSecond = metersPerSecond;
    }
    
    public readonly double To(LinearVelocityUnit unit) => unit.FromMetersPerSecond(MetersPerSecond);

    public readonly double ToMetersPerSecond() => MetersPerSecond;
    public readonly double ToMilesPerHour() => To(LinearVelocityUnit.MilesPerHour);
    
    public string ToString(LinearVelocityUnit unit, string? format = null, IFormatProvider? provider = null)
    {
        provider ??= CultureInfo.InvariantCulture;
        return To(unit).ToString(format, provider) + " " + unit.Symbol;
    }

    public override string ToString() => ToString(LinearVelocityUnit.MetersPerSecond);

    public bool Equals(LinearVelocity other) => MetersPerSecond.Equals(other.MetersPerSecond);

    public override bool Equals(object? obj) => obj is LinearVelocity other && Equals(other);

    public override int GetHashCode() => MetersPerSecond.GetHashCode();

    public static bool operator ==(LinearVelocity a, LinearVelocity b) => a.Equals(b);

    public static bool operator !=(LinearVelocity a, LinearVelocity b) => !a.Equals(b);

    public int CompareTo(LinearVelocity other) => MetersPerSecond.CompareTo(other.MetersPerSecond);

    public static bool operator <(LinearVelocity a, LinearVelocity b) => a.MetersPerSecond < b.MetersPerSecond;

    public static bool operator >(LinearVelocity a, LinearVelocity b) => a.MetersPerSecond > b.MetersPerSecond;

    public static bool operator <=(LinearVelocity a, LinearVelocity b) => a.MetersPerSecond <= b.MetersPerSecond;

    public static bool operator >=(LinearVelocity a, LinearVelocity b) => a.MetersPerSecond >= b.MetersPerSecond;

    public static LinearVelocity operator +(LinearVelocity a, LinearVelocity b) =>
        new(a.MetersPerSecond + b.MetersPerSecond);

    public static LinearVelocity operator -(LinearVelocity a, LinearVelocity b) =>
        new(a.MetersPerSecond - b.MetersPerSecond);

    public static LinearVelocity operator *(LinearVelocity v, double scalar) => new(v.MetersPerSecond * scalar);

    public static LinearVelocity operator /(LinearVelocity v, double scalar) => new(v.MetersPerSecond / scalar);

    public static Distance operator *(LinearVelocity v, Time t) => new Distance(v.MetersPerSecond * t.Seconds);
    
    public object Serialize() => new { mps = MetersPerSecond };
    public void Deserialize(object value)
    {
        var obj = (JsonElement)value;
        MetersPerSecond = obj.GetProperty("mps").GetDouble();
    }
}