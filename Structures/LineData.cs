using System;
namespace DLS.Structures
{
    public struct LineData
    {
        public double X { get; set; } // in meters
        public double Y { get; set; } // in meters
        public double CumulativeDistance { get; set; } // in meters
        public double Curvature { get; set; } // in 1/meters
        public double TangentDirection { get; set; } // in radians
        /*
        public double Width; // in meters
        public double Elevation; // in meters
        public double Banking; // in degrees
        public string SurfaceType; // e.g. "asphalt", "gravel", etc.
        */
    }
}
