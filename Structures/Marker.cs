using System;
namespace DLS.Structures
{
    public struct Marker
    {
        public double X { get; set; } // in meters
        public double Y { get; set; } // in meters
        public double CumulativeDistance { get; set; } // in meters
        public double TangentDirection { get; set; } // in radians
        public int? MarkerNumber { get; set; } // e.g. 1, 2, 3, ...
        public string Type { get; set; } // e.g. "braking", "apex", "acceleration"
    }
}