namespace igvc_csharp.Core.Units;

public sealed class AngleUnit
{
    private readonly double _radiansPerUnit;

    private AngleUnit(double radiansPerUnit, string symbol)
    {
        _radiansPerUnit = radiansPerUnit;
        Symbol = symbol;
    }

    public string Symbol { get; }

    public Angle Of(double value) => new(value * _radiansPerUnit);

    public double FromRadians(double radians) => radians / _radiansPerUnit;

    public static readonly AngleUnit Radians = new(1.0, "rad");

    public static readonly AngleUnit Degrees = new(Math.PI / 180.0, "°");
}