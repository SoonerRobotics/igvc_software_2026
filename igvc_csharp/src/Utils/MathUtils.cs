namespace igvc_csharp.Utils;

public class MathUtils
{
    /// <summary>
    /// Returns the result of a mod b, similar to Python % operator, which is different from C# % operator for negative numbers.
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <returns></returns>
    public static double Modulo(double a, double b)
    {
        return ((a % b) + b) % b;
    }

    public static double ToRadians(double degrees)
    {
        return degrees * Math.PI / 180.0;
    }

    public static double ToDegrees(double radians)
    {
        return radians * 180.0 / Math.PI;
    }
}