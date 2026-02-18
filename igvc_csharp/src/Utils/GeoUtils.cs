using System;
using igvc_csharp.Core.Units;

namespace igvc_csharp.Utils
{
    public static class GeoUtils
    {
        // WGS-84 ellipsoid
        private const double SemiMajorAxis = 6378137.0;
        private const double Flattening = 1.0 / 298.257223563;
        private const double EccentricitySqrd = Flattening * (2 - Flattening);
        private const double RadPerDeg = Math.PI / 180.0;
        private const double DegPerRad = 180.0 / Math.PI;
        
        private static (double east_m, double north_m) GenerateOffsetMeters(LatLng a, LatLng b)
        {
            var phi0 = a.Latitude * RadPerDeg;
            var lambda0 = a.Longitude * RadPerDeg;
            var phi = b.Latitude * RadPerDeg;
            var lambda = b.Longitude * RadPerDeg;

            var sinPhi0 = Math.Sin(phi0);
            var cosPhi0 = Math.Cos(phi0);
            var denom = Math.Sqrt(1.0 - EccentricitySqrd * sinPhi0 * sinPhi0);

            var n = SemiMajorAxis / denom;
            var m = SemiMajorAxis * (1.0 - EccentricitySqrd) / (denom * denom * denom);
            
            var dPhi = phi - phi0;
            var dLambda = lambda - lambda0;

            var north = m * dPhi;
            var east  = n * cosPhi0 * dLambda;

            return (east, north);
        }
        
        /**
         * Returns the distance between two LatLng points
         */
        public static Distance LatLngDistance(LatLng a, LatLng b)
        {
            var (e, n) = GenerateOffsetMeters(a, b);
            return DistanceUnit.Meters.Of(Math.Sqrt(e * e + n * n));
        }

        /**
         * Calculates the direction of travel between two LatLng points as a absolute angle (e.g. compass)
         * Returns null if no movement has occured
         */
        public static Angle? EstimateHeading(LatLng prev, LatLng current)
        {
            var (east, north) = GenerateOffsetMeters(prev, current);

            if (east == 0.0 && north == 0.0)
            {
                return null;
            }

            var theta = Math.Atan2(east, north);
            if (theta < 0)
            {
                theta += 2.0 * Math.PI;
            }

            return AngleUnit.Radians.Of(theta);
        }
    }
}