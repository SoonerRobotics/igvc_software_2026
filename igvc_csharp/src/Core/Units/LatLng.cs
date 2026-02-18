using igvc_csharp.Utils;

namespace igvc_csharp.Core.Units;

public sealed class LatLng(double latitude, double longitude)
{
    public double Latitude { get; } = latitude;

    public double Longitude { get; } = longitude;

    // Helpers

    public Distance Distance(LatLng other)
    {
        return GeoUtils.LatLngDistance(this, other);
    }

    public static Angle? TravelHeading(LatLng start, LatLng end)
    {
        return GeoUtils.EstimateHeading(start, end);
    }
}