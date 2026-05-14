namespace igvc_csharp.Core.Units;

public sealed class Ypr(double yaw, double pitch, double roll)
{
    public double Yaw { get; } = yaw;
    public double Pitch { get; } = pitch;
    public double Roll { get; } = roll;

    public override string ToString()
    {
        return $"YawPitchRoll(Yaw={Yaw}°, Pitch={Pitch}°, Roll={Roll}°)";
    }
}