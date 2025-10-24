using System;

namespace DLS.Structures
{
    public struct GGData
    {
        public double Magnitude { get; set; }     // G-force magnitude (in g's)
        public double Angle { get; set; }         // Angle in radians (0 = forward, +π/2 = right, -π/2 = left, ±π = backward)
        public double LongitudinalG { get; set; } // Forward/backward G-force (calculated from magnitude and angle)
        public double LateralG { get; set; }      // Left/right G-force (calculated from magnitude and angle)
        
        // Helper method to calculate X,Y components from magnitude and angle
        public (double x, double y) GetXYComponents()
        {
            double x = Magnitude * Math.Cos(Angle);  // Longitudinal (forward/backward)
            double y = Magnitude * Math.Sin(Angle);  // Lateral (right/left)
            return (x, y);
        }
        
        // Helper method to set magnitude and angle from X,Y components
        public void SetFromXY(double x, double y)
        {
            Magnitude = Math.Sqrt(x * x + y * y);
            
            // Convert from standard math convention to convention
            double mathAngle = Math.Atan2(y, x);  // Standard atan2 (0 = right, π/2 = up)
            Angle = (Math.PI / 2.0) - mathAngle;  // Convert to convention (0 = up, π/2 = right)
            
            // Normalize angle to [-π, +π] range
            while (Angle > Math.PI) Angle -= 2 * Math.PI;
            while (Angle < -Math.PI) Angle += 2 * Math.PI;
                
            LongitudinalG = x; // Forward/backward
            LateralG = y;      // Left/right
        }
    }
}