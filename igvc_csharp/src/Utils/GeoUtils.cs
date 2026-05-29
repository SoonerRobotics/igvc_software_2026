using System;
using igvc_csharp.Core.Config;
using igvc_csharp.Core.Units;

namespace igvc_csharp.Utils
{
    public class GeoUtils
    {
        [Config("geoutils.latitude_length")]
        public static double LatitudeLength = 110086.2;

        [Config("geoutils.longitude_length")]
        public static double LongitudeLength = 81978.2;

        // Calculates the distance between two GPS coordinates in meters using a simple flat Earth approximation
        public static Distance LatLngDistance(LatLng a, LatLng b)
        {
            var northToGps = (b.Latitude - a.Latitude) * LatitudeLength;
            var eastToGps = (b.Longitude - a.Longitude) * LongitudeLength;

            return new Distance(Math.Sqrt(northToGps * northToGps + eastToGps * eastToGps));
        }

        // Estimates the heading from the previous position to the current position as an absolute angle (e.g. compass)
        public static Angle? EstimateHeading(LatLng prev, LatLng current)
        {
            var distance = LatLngDistance(prev, current);
            if (distance.To(DistanceUnit.Meters) < 0.2) // If the robot hasn't moved, we can't estimate a heading
            {
                return null;
            }

            var deltaLon = MathUtils.ToRadians(current.Longitude - prev.Longitude);
            var lat1 = MathUtils.ToRadians(prev.Latitude);
            var lat2 = MathUtils.ToRadians(current.Latitude);

            var x = Math.Sin(deltaLon) * Math.Cos(lat2);
            var y = Math.Cos(lat1) * Math.Sin(lat2) - (
                Math.Sin(lat1) * Math.Cos(lat2) * Math.Cos(deltaLon)
            );
            var heading = MathUtils.Modulo(Math.Atan2(x, y), 2 * Math.PI);
            return new Angle(heading, isRadians: true);
        }

        // Calculates the heading from the current position to a target position as an absolute angle (e.g. compass)
        public static Angle HeadingToPosition(LatLng current, LatLng target)
        {
            var northToGps = (target.Latitude - current.Latitude) * LatitudeLength;
            var eastToGps = (target.Longitude - current.Longitude) * LongitudeLength;
            var heading = MathUtils.Modulo(Math.Atan2(eastToGps, northToGps), 2 * Math.PI);
            return new Angle(heading, isRadians: true);
        }

        // Returns the smallest difference between two angles, wrapped to [-pi, pi]
        public static Angle GetAngleDifference(Angle a, Angle b)
        {
            var delta = a.To(AngleUnit.Radians) - b.To(AngleUnit.Radians);
            delta = MathUtils.Modulo(delta + Math.PI, 2 * Math.PI) - Math.PI; // Wrap to [-pi, pi]
            return new Angle(delta, isRadians: true);
        }
    }
}