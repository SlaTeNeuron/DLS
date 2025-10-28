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
        private int chicaneCount = 0;
        private int indicesPerMeter = 100; // Default value, will be set during data processing
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
        public double IndicesPerMeter => indicesPerMeter;
        public double trackWidth = 3.0; // in meters
        public double carWidth = 1.0; // in meters
        public int MarkersPerMeter = 1;
        #endregion

        #region Main Entry Point
        public static void Main()
        {
            var optimiser = new AccelerationOptimiser();
            optimiser.ChooseOptimizationMethod();
        }

        public void ChooseOptimizationMethod()
        {
            Console.WriteLine("Choose optimization method:");
            Console.WriteLine("1. Classic Acceleration Optimization");
            Console.WriteLine("2. New CMA-ES Based Optimization");
            Console.Write("Enter your choice (1 or 2): ");

            string? input = Console.ReadLine();
            
            switch (input?.Trim())
            {
                case "1":
                    Console.WriteLine("Running Classic Acceleration Optimization...");
                    OptimiseAcceleration();
                    break;
                case "2":
                    Console.WriteLine("Running CMA-ES Based Optimization...");
                    OptimiseWithCMAES();
                    break;
                default:
                    Console.WriteLine("Invalid choice. Running default Classic Acceleration Optimization...");
                    OptimiseAcceleration();
                    break;
            }
        }

        public void OptimiseAcceleration()
        {
            ProcessData();
            GenerateTrack();
            this.PerformSpeedOptimization();    // Extension method
        }

        public void OptimiseWithCMAES()
        {
            Console.WriteLine("Starting CMA-ES based track optimization...");
            
            // Basic setup - load data and analyze turns
            ProcessData();
            
            // Count chicanes and calculate parameters
            chicaneCount = turns.Count(turn => turn.IsChicaneStart);
            
            // Calculate dimension: Each regular turn has 3 parameters (start, apex, end)
            // Each chicane pair has 4 parameters (start1, apex1, transition, apex2, end2)
            int regularTurns = turns.Count - (chicaneCount * 2); // Subtract chicane turn pairs
            int dimension = regularTurns * 3 + chicaneCount * 4;
            
            Console.WriteLine($"Optimizing {turns.Count} turns ({chicaneCount} chicanes) with {dimension} parameters...");
            
            // Define the objective function for CMA-ES
            Func<double[], double> lapTimeObjective = parameters => 
            {
                try
                {
                    return CalculateLapTimeFromParameters(parameters);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error in objective function: {ex.Message}");
                    return double.MaxValue; // Return worst possible lap time on error
                }
            };
            
            // Set up initial parameters based on current turn indices
            double[] initialMean = GenerateInitialParameters();
            
            // Run CMA-ES optimization
            var result = CMA.Optimize(
                lapTimeObjective, 
                dimension, 
                initialMean, 
                initialSigma: 5.0, // Allow reasonable variation in indices
                maxEvaluations: 1000,
                minimization: true  // Minimize lap time
            );
            
            Console.WriteLine($"CMA-ES optimization completed!");
            Console.WriteLine($"Best lap time: {result.BestFitness:F3} seconds");
            Console.WriteLine($"Evaluations used: {result.Evaluations}");
            
            // Apply the best solution
            ApplyOptimizedParameters(result.BestSolution);
            
            // Generate final racing line with optimized parameters
            GenerateTrack();
            this.PerformSpeedOptimization();
            
            Console.WriteLine("Final racing line generated with optimized turn points.");
        }

        public void ProcessData()
        {
            this.ReadTrackData();    // Extension method
            this.ReadGGData();       // Extension method
            this.DetectAndAnalyzeTurns();     // Extension method
        }
        #endregion

        public void GenerateTrack()
        {
            this.GenerateOptimizationMarkers(); // Extension method
            this.BuildRacingLine();   // Extension method
        }

        private double CalculateLapTimeFromParameters(double[] parameters)
        {
            // Update turn indices based on parameters
            UpdateTurnIndicesFromParameters(parameters);
            
            // Generate markers for current configuration (lightweight)
            var tempMarkers = GenerateMarkersFromCurrentTurns();
            
            // Build a simplified racing line (just for lap time calculation)
            var tempRacingLine = GenerateSimpleRacingLine(tempMarkers);
            
            // Calculate lap time efficiently (skinny implementation)
            return CalculateLapTimeEfficiently(tempRacingLine);
        }
        
        private double[] GenerateInitialParameters()
        {
            var parameters = new List<double>();
            
            for (int i = 0; i < turns.Count; i++)
            {
                var turn = turns[i];
                
                if (turn.IsChicaneStart)
                {
                    // Chicane: start1, apex1, transition, apex2, end2
                    parameters.Add(turn.StartIndex);
                    parameters.Add(turn.ApexIndex);
                    parameters.Add(turn.ChicaneTransitionIndex != -1 ? turn.ChicaneTransitionIndex : turn.EndIndex);
                    
                    // Find the chicane end turn (next turn should be chicane end)
                    if (i + 1 < turns.Count && turns[i + 1].IsChicaneEnd)
                    {
                        var endTurn = turns[i + 1];
                        parameters.Add(endTurn.ApexIndex);
                        parameters.Add(endTurn.EndIndex);
                    }
                    else
                    {
                        // Fallback if chicane end not found properly
                        parameters.Add(turn.ApexIndex + 5);
                        parameters.Add(turn.EndIndex + 10);
                    }
                }
                else if (!turn.IsChicaneEnd) // Regular turn (not part of chicane)
                {
                    parameters.Add(turn.StartIndex);
                    parameters.Add(turn.ApexIndex);
                    parameters.Add(turn.EndIndex);
                }
                // Skip chicane end turns as they're handled with chicane start
            }
            
            return parameters.ToArray();
        }
        
        private void UpdateTurnIndicesFromParameters(double[] parameters)
        {
            int paramIndex = 0;
            
            for (int i = 0; i < turns.Count; i++)
            {
                var turn = turns[i];
                
                if (turn.IsChicaneStart && paramIndex + 4 < parameters.Length)
                {
                    // Update chicane start turn
                    turn.StartIndex = Math.Max(0, (int)Math.Round(parameters[paramIndex++]));
                    turn.ApexIndex = Math.Max(turn.StartIndex + 1, (int)Math.Round(parameters[paramIndex++]));
                    int transitionIndex = Math.Max(turn.ApexIndex + 1, (int)Math.Round(parameters[paramIndex++]));
                    
                    // Update chicane end turn (should be next turn)
                    if (i + 1 < turns.Count && turns[i + 1].IsChicaneEnd)
                    {
                        var endTurn = turns[i + 1];
                        endTurn.StartIndex = transitionIndex;
                        endTurn.ApexIndex = Math.Max(transitionIndex + 1, (int)Math.Round(parameters[paramIndex++]));
                        endTurn.EndIndex = Math.Max(endTurn.ApexIndex + 1, (int)Math.Round(parameters[paramIndex++]));
                        
                        turn.EndIndex = transitionIndex;
                        turn.ChicaneTransitionIndex = transitionIndex;
                        endTurn.ChicaneTransitionIndex = transitionIndex;
                        
                        turns[i + 1] = endTurn;
                    }
                    
                    turns[i] = turn;
                }
                else if (!turn.IsChicaneEnd && paramIndex + 2 < parameters.Length) // Regular turn
                {
                    turn.StartIndex = Math.Max(0, (int)Math.Round(parameters[paramIndex++]));
                    turn.ApexIndex = Math.Max(turn.StartIndex + 1, (int)Math.Round(parameters[paramIndex++]));
                    turn.EndIndex = Math.Max(turn.ApexIndex + 1, (int)Math.Round(parameters[paramIndex++]));
                    
                    turns[i] = turn;
                }
                // Skip chicane end turns as they're handled above
            }
        }
        
        private Marker[] GenerateMarkersFromCurrentTurns()
        {
            var markers = new List<Marker>();
            
            foreach (var turn in turns)
            {
                if (turn.StartIndex < trackCentre.Length && turn.ApexIndex < trackCentre.Length && turn.EndIndex < trackCentre.Length)
                {
                    // Add braking marker
                    markers.Add(new Marker
                    {
                        X = trackCentre[turn.StartIndex].X,
                        Y = trackCentre[turn.StartIndex].Y,
                        CumulativeDistance = trackCentre[turn.StartIndex].CumulativeDistance,
                        Type = "braking"
                    });
                    
                    // Add apex marker
                    markers.Add(new Marker
                    {
                        X = trackCentre[turn.ApexIndex].X,
                        Y = trackCentre[turn.ApexIndex].Y,
                        CumulativeDistance = trackCentre[turn.ApexIndex].CumulativeDistance,
                        Type = "apex"
                    });
                    
                    // Add acceleration marker
                    markers.Add(new Marker
                    {
                        X = trackCentre[turn.EndIndex].X,
                        Y = trackCentre[turn.EndIndex].Y,
                        CumulativeDistance = trackCentre[turn.EndIndex].CumulativeDistance,
                        Type = "acceleration"
                    });
                }
            }
            
            return markers.ToArray();
        }
        
        private LineData[] GenerateSimpleRacingLine(Marker[] markers)
        {
            // Simplified racing line generation - just use track center with marker modifications
            var racingLine = new LineData[trackCentre.Length];
            Array.Copy(trackCentre, racingLine, trackCentre.Length);
            
            // Apply simple smoothing around markers (basic implementation)
            foreach (var marker in markers)
            {
                int closestIndex = FindClosestTrackIndex(marker);
                if (closestIndex >= 0 && closestIndex < racingLine.Length)
                {
                    racingLine[closestIndex].X = marker.X;
                    racingLine[closestIndex].Y = marker.Y;
                }
            }
            
            return racingLine;
        }
        
        private int FindClosestTrackIndex(Marker marker)
        {
            double minDistance = double.MaxValue;
            int closestIndex = 0;
            
            for (int i = 0; i < trackCentre.Length; i++)
            {
                double dx = trackCentre[i].X - marker.X;
                double dy = trackCentre[i].Y - marker.Y;
                double distance = Math.Sqrt(dx * dx + dy * dy);
                
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestIndex = i;
                }
            }
            
            return closestIndex;
        }
        
        private double CalculateLapTimeEfficiently(LineData[] racingLine)
        {
            // Skinny lap time calculation - minimal computation
            double totalTime = 0.0;
            const double maxSpeed = 44.7; // ~100 mph in m/s as reference terminal speed
            
            var (maxAccelG, maxBrakingG, maxLateralG) = this.GetGGLimits();
            
            double currentSpeed = 0.0; // Start from standstill
            
            for (int i = 1; i < racingLine.Length; i++)
            {
                double distance = racingLine[i].CumulativeDistance - racingLine[i - 1].CumulativeDistance;
                if (distance <= 0) continue;
                
                double curvature = Math.Abs(racingLine[i].Curvature);
                
                // Calculate curvature-limited speed
                double maxCurveSpeed = maxSpeed;
                if (curvature > 0.0001)
                {
                    maxCurveSpeed = Math.Sqrt((maxLateralG * 9.81) / curvature);
                }
                
                // Simple acceleration/deceleration model
                double targetSpeed = Math.Min(maxSpeed, maxCurveSpeed);
                
                if (targetSpeed > currentSpeed)
                {
                    // Accelerating
                    double maxAccelSpeed = Math.Sqrt(currentSpeed * currentSpeed + 2 * maxAccelG * 9.81 * distance);
                    currentSpeed = Math.Min(targetSpeed, maxAccelSpeed);
                }
                else
                {
                    // Need to slow down - check if we can brake in time
                    double maxBrakeSpeed = Math.Sqrt(targetSpeed * targetSpeed + 2 * maxBrakingG * 9.81 * distance);
                    currentSpeed = Math.Min(currentSpeed, maxBrakeSpeed);
                }
                
                // Calculate time for this segment
                double avgSpeed = Math.Max(0.1, (currentSpeed + Math.Min(targetSpeed, currentSpeed)) / 2.0);
                totalTime += distance / avgSpeed;
            }
            
            return totalTime;
        }
        
        private void ApplyOptimizedParameters(double[] bestParameters)
        {
            // Apply the best parameters found by CMA-ES
            UpdateTurnIndicesFromParameters(bestParameters);
            
            Console.WriteLine("Applied optimized turn parameters:");
            for (int i = 0; i < turns.Count; i++)
            {
                var turn = turns[i];
                Console.WriteLine($"Turn {i + 1}: Start={turn.StartIndex}, Apex={turn.ApexIndex}, End={turn.EndIndex}");
                if (turn.IsChicaneStart || turn.IsChicaneEnd)
                {
                    Console.WriteLine($"  Chicane transition: {turn.ChicaneTransitionIndex}");
                }
            }
        }

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

            // Determine the relevant quadrant based on braking state and lateral G direction
            double minAngle, maxAngle;
            if (isBraking && requiredLateralG > 0)
            {
                // Braking with right turn (positive lateral G): π/2 to π
                minAngle = Math.PI / 2;
                maxAngle = Math.PI;
            }
            else if (isBraking && requiredLateralG < 0)
            {
                // Braking with left turn (negative lateral G): -π/2 to -π
                minAngle = -Math.PI;
                maxAngle = -Math.PI / 2;
            }
            else if (!isBraking && requiredLateralG > 0)
            {
                // Accelerating with right turn (positive lateral G): 0 to π/2
                minAngle = 0;
                maxAngle = Math.PI / 2;
            }
            else // !isBraking && requiredLateralG < 0
            {
                // Accelerating with left turn (negative lateral G): -π/2 to 0
                minAngle = -Math.PI / 2;
                maxAngle = 0;
            }

            // Search through GG data to find the maximum longitudinal G available
            // when the required lateral G is being used, only in the relevant quadrant
            foreach (var gg in ggData)
            {
                // Skip if not in the relevant quadrant
                if (gg.Angle < minAngle || gg.Angle > maxAngle)
                    continue;

                var (longitudinalG, lateralG) = gg.GetXYComponents();

                // Check if this point provides sufficient lateral G
                if (Math.Abs(lateralG) >= Math.Abs(requiredLateralG))
                {
                    if (isBraking)
                    {
                        // For braking, we want maximum negative longitudinal G
                        if (longitudinalG < maxLongitudinalG)
                            maxLongitudinalG = longitudinalG;
                    }
                    else
                    {
                        // For acceleration, we want maximum positive longitudinal G
                        if (longitudinalG > maxLongitudinalG)
                            maxLongitudinalG = longitudinalG;
                    }
                }
            }

            return maxLongitudinalG;
        }

        private (double maxAccel, double maxBraking, double maxLateral) GetGGLimits()
        {
            if (ggData == null || ggData.Length == 0)
                return (1.0, 1.0, 1.0); // Default fallback values
            
            double maxAccel = 0.0, maxBraking = 0.0, maxLateral = 0.0;

            foreach (var gg in ggData)
            {
                var (longitudinalG, lateralG) = gg.GetXYComponents();
                
                if (longitudinalG > maxAccel) maxAccel = longitudinalG;
                if (longitudinalG < maxBraking) maxBraking = longitudinalG;
                if (Math.Abs(lateralG) > maxLateral) maxLateral = Math.Abs(lateralG);
            }

            return (maxAccel, Math.Abs(maxBraking), maxLateral);
        }

        #endregion

        #endregion
    }
}
