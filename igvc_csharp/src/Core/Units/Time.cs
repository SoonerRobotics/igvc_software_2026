using System.Text.Json;
using igvc_csharp.Core.Config;

namespace igvc_csharp.Core.Units;

using System;

public struct Time : IEquatable<Time>, IComparable<Time>, IConfigSerializable
{
    public double Seconds { set; get; }
    
    internal Time(double seconds)
    {
        Seconds = seconds;
    }

    public static Time FromSeconds(double seconds) => new(seconds);

    public bool Equals(Time other) => Seconds.Equals(other.Seconds);

    public override bool Equals(object? obj) => obj is Time other && Equals(other);

    public override int GetHashCode() => Seconds.GetHashCode();

    public static bool operator ==(Time a, Time b) => a.Equals(b);

    public static bool operator !=(Time a, Time b) => !a.Equals(b);

    public int CompareTo(Time other) => Seconds.CompareTo(other.Seconds);

    public static bool operator <(Time a, Time b) => a.Seconds < b.Seconds;

    public static bool operator >(Time a, Time b) => a.Seconds > b.Seconds;

    public static bool operator <=(Time a, Time b) => a.Seconds <= b.Seconds;

    public static bool operator >=(Time a, Time b) => a.Seconds >= b.Seconds;

    public static Time operator +(Time a, Time b) => new(a.Seconds + b.Seconds);

    public static Time operator -(Time a, Time b) => new(a.Seconds - b.Seconds);

    public static Time operator *(Time t, double scalar) => new(t.Seconds * scalar);

    public static Time operator /(Time t, double scalar) => new(t.Seconds / scalar);

    public override string ToString() => $"{Seconds} s";
    
    public object Serialize() => new { seconds = Seconds };
    public object Deserialize(object value)
    {
        var obj = (JsonElement)value;
        Seconds = obj.GetProperty("seconds").GetDouble();
        return this;
    }
}