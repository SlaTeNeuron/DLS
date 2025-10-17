using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using DLS.Structures;
using DLS.Extensions;

namespace DLS
{
    public class AccelerationOptimiser
    {
        #region Fields
        private LineData[] trackCentre = Array.Empty<LineData>();
        private LineData[] racingLine = Array.Empty<LineData>();
        private GGData[] ggData = Array.Empty<GGData>();
        private OptimizationResult[] optimizationResults = Array.Empty<OptimizationResult>();
        private List<Turn> turns = new List<Turn>();
        private Marker[] optimizationMarkers = Array.Empty<Marker>();
        private string trackFile = string.Empty;
        private string ggFile = string.Empty;
        #endregion

        #region Public Properties and Accessors for Extensions
        public LineData[] TrackCentre => trackCentre;
        public LineData[] RacingLine => racingLine;
        public GGData[] GGDataPoints => ggData;
        public Marker[] OptimizationMarkers => optimizationMarkers;
        public List<Turn> Turns => turns;
        public OptimizationResult[] GetOptimizationResults() => optimizationResults;
        public LineData[] GetRacingLine() => racingLine;
        public void SetTrackCentre(LineData[] data) => trackCentre = data;
        public void SetRacingLine(LineData[] data) => racingLine = data;
        public void SetGGData(GGData[] data) => ggData = data;
        public void SetTurns(List<Turn> data) => turns = data;
        public void SetOptimizationResults(OptimizationResult[] data) => optimizationResults = data;
        public void SetOptimizationMarkers(Marker[] data) => optimizationMarkers = data;
        #endregion

        #region Main Entry Point
        public static void Main()
        {
            var optimiser = new AccelerationOptimiser();
            optimiser.OptimiseAcceleration();
        }

        public void OptimiseAcceleration()
        {
            this.ReadTrackData();    // Extension method
            this.ReadGGData();       // Extension method  
            this.DetectAndAnalyzeTurns();     // Extension method
            this.GenerateOptimizationMarkers(); // Extension method
            this.BuildRacingLine();   // Extension method
            this.PerformSpeedOptimization();    // Extension method
        }
        #endregion

        #region GG Data

        #region GG Data Calculations
        // GG Diagram Lookup and Interpolation Methods
        public double MaxG(double targetAngle)
        {
            if (ggData == null || ggData.Length == 0)
                throw new InvalidOperationException("GG data not loaded. Call ReadGGData() first.");

            // Normalize target angle to [-π, +π] range
            while (targetAngle > Math.PI) targetAngle -= 2 * Math.PI;
            while (targetAngle < -Math.PI) targetAngle += 2 * Math.PI;

            // Binary search for insertion point - O(log n) instead of O(n)
            int insertIndex = BinarySearchAngle(targetAngle);

            // Get adjacent indices for interpolation
            int lowerIndex, upperIndex;
            GetAdjacentIndices(insertIndex, targetAngle, out lowerIndex, out upperIndex);

            // Handle exact match
            if (lowerIndex == upperIndex)
                return ggData[lowerIndex].Magnitude;

            // Linear interpolation between the two points
            return InterpolateGForce(targetAngle, lowerIndex, upperIndex);
        }

        private int BinarySearchAngle(double targetAngle)
        {
            int left = 0, right = ggData.Length;

            while (left < right)
            {
                int mid = (left + right) / 2;
                if (ggData[mid].Angle < targetAngle)
                    left = mid + 1;
                else
                    right = mid;
            }

            return left; // Insertion point
        }

        private void GetAdjacentIndices(int insertIndex, double targetAngle, out int lowerIndex, out int upperIndex)
        {
            int n = ggData.Length;

            if (insertIndex == 0)
            {
                // Target is before first element
                lowerIndex = upperIndex = 0;
            }
            else if (insertIndex == n)
            {
                // Target is after last element
                lowerIndex = upperIndex = n - 1;
            }
            else
            {
                // Target is between elements - use adjacent points
                lowerIndex = insertIndex - 1;
                upperIndex = insertIndex;

                // Check for exact match
                if (Math.Abs(ggData[lowerIndex].Angle - targetAngle) < 1e-10)
                    upperIndex = lowerIndex;
                else if (Math.Abs(ggData[upperIndex].Angle - targetAngle) < 1e-10)
                    lowerIndex = upperIndex;
            }
        }

        private double InterpolateGForce(double targetAngle, int lowerIndex, int upperIndex)
        {
            double angle1 = ggData[lowerIndex].Angle;
            double angle2 = ggData[upperIndex].Angle;
            double mag1 = ggData[lowerIndex].Magnitude;
            double mag2 = ggData[upperIndex].Magnitude;

            // Simple linear interpolation (no wrap-around needed for adjacent points)
            double angleDelta = angle2 - angle1;
            if (Math.Abs(angleDelta) < 1e-10) return mag1; // Same angle

            double interpolationFactor = (targetAngle - angle1) / angleDelta;
            return mag1 + interpolationFactor * (mag2 - mag1);
        }

        public double GetMaxLateralGForTurn(bool isRightTurn)
        {
            double angle = isRightTurn ? Math.PI / 2.0 : -Math.PI / 2.0;
            return MaxG(angle);
        }

        public double GetMaxLongitudinalGForLateralG(double requiredLateralG, bool isBraking)
        {
            if (ggData == null || ggData.Length == 0)
                return isBraking ? 0.0 : 0.0; // Conservative fallback

            double maxLongitudinalG = 0.0;

            // Search through GG data to find the maximum longitudinal G available
            // when the required lateral G is being used
            foreach (var gg in ggData)
            {
                var (longitudinalG, lateralG) = gg.GetXYComponents();

                // Check if this point provides sufficient lateral G
                if (Math.Abs(lateralG) >= Math.Abs(requiredLateralG))
                {
                    if (isBraking)
                    {
                        // For braking, we want maximum negative longitudinal G
                        if (longitudinalG < -maxLongitudinalG)
                            maxLongitudinalG = Math.Abs(longitudinalG);
                    }
                    else
                    {
                        // For acceleration, we want maximum positive longitudinal G
                        if (longitudinalG > maxLongitudinalG)
                            maxLongitudinalG = longitudinalG;
                    }
                }
            }

            return maxLongitudinalG >= 0 ? maxLongitudinalG : (isBraking ? 1.0 : 1.0);
        }

        #endregion

        #endregion
    }
}
