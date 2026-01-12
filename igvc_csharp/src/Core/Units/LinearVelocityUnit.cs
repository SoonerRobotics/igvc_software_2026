namespace igvc_csharp.Core.Units;

public sealed class LinearVelocityUnit
{
    private readonly double _metersPerSecondPerUnit;

    private LinearVelocityUnit(double metersPerSecondPerUnit, string symbol)
    {
        _metersPerSecondPerUnit = metersPerSecondPerUnit;
        Symbol = symbol;
    }

    public string Symbol { get; }

    public LinearVelocity Of(double value) => new(value * _metersPerSecondPerUnit);

    public double FromMetersPerSecond(double metersPerSecond) => metersPerSecond / _metersPerSecondPerUnit;

    public static readonly LinearVelocityUnit MetersPerSecond = new(1.0, "m/s");

    public static readonly LinearVelocityUnit FeetPerSecond = new(0.3048, "ft/s");

    public static readonly LinearVelocityUnit MilesPerHour = new(0.44704, "mph");
}