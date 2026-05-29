using igvc_csharp.Core.Units;

public class RobotPosition(LatLng coordinates, Angle? heading = null)
{
    public LatLng Coordinates = coordinates;
    public Angle? Heading = heading;
}