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
        
        /**
         * Returns the distance between two LatLng points in meters
         */
        public static Distance LatLngDistance(LatLng a, LatLng b)
        {
            var northToGps = (b.Latitude - a.Latitude) * LatitudeLength;
            var eastToGps = (b.Longitude - a.Longitude) * LongitudeLength;
            
            return new Distance(Math.Sqrt(northToGps * northToGps + eastToGps * eastToGps));
        }

        /**
         * Calculates the direction of travel between two LatLng points as a absolute angle (e.g. compass)
         * Returns null if no movement has occured
         */
        public static Angle? EstimateHeading(LatLng prev, LatLng current)
        {
            var distance = LatLngDistance(prev, current);
            if (distance.To(DistanceUnit.Meters) < 0.1) // If the robot hasn't moved, we can't estimate a heading
            {
                return null;
            }
            
            var deltaLon = (current.Longitude - prev.Longitude) * (Math.PI / 180);
            var lat1 = prev.Latitude * (Math.PI / 180);
            var lat2 = current.Latitude * (Math.PI / 180);
            
            var x = Math.Sin(deltaLon) * Math.Cos(lat2);
            var y = Math.Cos(lat1) * Math.Sin(lat2) - Math.Sin(lat1) * Math.Cos(lat2) * Math.Cos(deltaLon);
            var heading = Math.Atan2(x, y) % (2 * Math.PI);
            return new Angle(heading, isRadians: true);
        }
    }
}