using System;

namespace DLS.Structures
{
    public class SpeedProfile
    {
        public int TurnIndex { get; set; }
        public double[] Speeds { get; set; } = Array.Empty<double>();
        public double[] LateralGs { get; set; } = Array.Empty<double>();
        public double[] LongitudinalGs { get; set; } = Array.Empty<double>();
    }
}