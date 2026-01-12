namespace igvc_csharp.Core.Units;

public sealed class AngularVelocityUnit
{
    private readonly double _radiansPerSecondPerUnit;

    private AngularVelocityUnit(double radiansPerSecondPerUnit, string symbol)
    {
        _radiansPerSecondPerUnit = radiansPerSecondPerUnit;
        Symbol = symbol;
    }

    public string Symbol { get; }

    public AngularVelocity Of(double value)
        => new AngularVelocity(value * _radiansPerSecondPerUnit);

    public double FromRadiansPerSecond(double radiansPerSecond)
        => radiansPerSecond / _radiansPerSecondPerUnit;

    public static readonly AngularVelocityUnit RadiansPerSecond =
        new(1.0, "rad/s");

    public static readonly AngularVelocityUnit DegreesPerSecond =
        new(Math.PI / 180.0, "deg/s");

    public static readonly AngularVelocityUnit RevolutionsPerMinute =
        new(2.0 * Math.PI / 60.0, "rpm");
}
