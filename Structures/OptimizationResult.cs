namespace DLS.Structures
{
    public struct OptimizationResult
    {
        public double Time { get; set; }           // Time at this point (seconds)
        public double Speed { get; set; }          // Speed at this point (m/s)
        public double LateralG { get; set; }       // Lateral G-force used for this speed (g)
        public double LongitudinalG { get; set; }  // Longitudinal G-force used for this speed (g)
        public double TotalG { get; set; }         // Total G-force magnitude (g)
        
        public OptimizationResult(double time, double speed, double lateralG, double longitudinalG)
        {
            Time = time;
            Speed = speed;
            LateralG = lateralG;
            LongitudinalG = longitudinalG;
            TotalG = Math.Sqrt(lateralG * lateralG + longitudinalG * longitudinalG);
        }
        
        // Convenience properties for different speed units
        public double SpeedMph => Speed * 2.23694; // Convert m/s to mph
        public double SpeedKph => Speed * 3.6;     // Convert m/s to km/h
    }
}