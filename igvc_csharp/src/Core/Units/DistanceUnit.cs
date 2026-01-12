namespace igvc_csharp.Core.Units;

public sealed class DistanceUnit
{
    private readonly double _metersPerUnit;
    public string Symbol { get; }

    private DistanceUnit(double metersPerUnit, string symbol)
    {
        _metersPerUnit = metersPerUnit;
        Symbol = symbol;
    }

    public Distance Of(double value) => new Distance(value * _metersPerUnit);

    public double FromMeters(double meters) => meters / _metersPerUnit;

    // Metric
    public static readonly DistanceUnit Meters = new(1.0, "m");
    public static readonly DistanceUnit Centimeters = new(0.01, "cm");
    public static readonly DistanceUnit Millimeters = new(0.001, "mm");

    // Imperial
    public static readonly DistanceUnit Feet = new(0.3048, "ft");
    public static readonly DistanceUnit Inches = new(0.0254, "in");
}