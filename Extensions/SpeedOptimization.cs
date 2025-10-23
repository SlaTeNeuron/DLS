using System;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using DLS.Structures;

namespace DLS.Extensions
{
    /// <summary>
    /// Extension methods for speed optimization and performance analysis
    /// </summary>
    public static class SpeedOptimization
    {
        public static void PerformSpeedOptimization(this AccelerationOptimiser optimiser)
        {
            var racingLine = optimiser.RacingLine;
            var turns = optimiser.Turns;
            var ggData = optimiser.GGDataPoints;

            if (racingLine == null || racingLine.Length == 0)
                throw new InvalidOperationException("Racing line not calculated. Call FindRacingLine() first.");

            if (turns == null || turns.Count == 0)
                throw new InvalidOperationException("Turn analysis not completed. Call AnalyzeTurns() first.");

            if (ggData == null || ggData.Length == 0)
                throw new InvalidOperationException("GG data not loaded. Call ReadGGData() first.");

            Console.WriteLine($"Optimizing speed for {turns.Count} corners in parallel...");
            var startTime = DateTime.Now;

            var (maxAccelG, maxBrakingG, maxLateralG) = optimiser.GetGGLimits();
            Console.WriteLine($"Using GG interpolation table with reference limits - Lateral: {maxLateralG:F3}g, Accel: {maxAccelG:F3}g, Braking: {maxBrakingG:F3}g");

            // Create speed profiles for each corner in parallel
            var speedProfiles = new SpeedProfile[turns.Count];

            Parallel.For(0, turns.Count, i =>
            {
                speedProfiles[i] = AnalyzeCornerSpeedProfile(turns[i], racingLine, optimiser, maxAccelG, maxBrakingG);
            });

            // Resolve conflicts between overlapping profiles
            var (finalSpeedProfile, finalLateralGs, finalLongitudinalGs) = ResolveSpeedAndGForceConflicts(speedProfiles, racingLine, maxAccelG);

            // Apply final speed profile to racing line
            ApplySpeedProfileToRacingLine(finalSpeedProfile, turns.ToArray());

            // Calculate time profile
            var timeProfile = CalculateTimeProfile(finalSpeedProfile, racingLine);

            // Create optimization results
            CreateOptimizationResults(optimiser, finalSpeedProfile, timeProfile, finalLateralGs, finalLongitudinalGs, racingLine);

            var optimizeTime = DateTime.Now - startTime;
            Console.WriteLine($"Speed optimization completed in {optimizeTime.TotalMilliseconds:F1} ms");

            // Export speed data and create visualization
            ExportAndVisualizeSpeeds(optimiser, finalSpeedProfile, timeProfile);
        }

        private static SpeedProfile AnalyzeCornerSpeedProfile(Turn turn, LineData[] racingLine, AccelerationOptimiser optimiser, double maxAccelG, double maxBrakingG)
        {
            const double terminalSpeedMph = 100.0;
            const double terminalSpeedMs = terminalSpeedMph * 0.44704;
            bool isRightTurn = racingLine[turn.ApexIndex].Curvature > 0;
            double maxLateralG = optimiser.GetMaxLateralGForTurn(isRightTurn);
            int trueStartIndex = turn.StartIndex;
            int trueEndIndex = turn.EndIndex;

            var profile = new SpeedProfile
            {
                TurnIndex = Array.IndexOf(optimiser.Turns.ToArray(), turn),
                Speeds = new double[racingLine.Length],
                LateralGs = new double[racingLine.Length],
                LongitudinalGs = new double[racingLine.Length]
            };

            // Initialize all speeds to terminal velocity
            for (int i = 0; i < profile.Speeds.Length; i++)
            {
                profile.Speeds[i] = terminalSpeedMs;
                profile.LateralGs[i] = 0.0;
                profile.LongitudinalGs[i] = 0.0;
            }

            for (int i = turn.ApexIndex - 1; i >= 0 && Math.Abs(racingLine[i].Curvature) >= Math.Abs(turn.MaxCurvature * 0.05); i--)
            {
                trueStartIndex = i;
            }
            for (int i = turn.ApexIndex + 1; i < racingLine.Length && Math.Abs(racingLine[i].Curvature) >= Math.Abs(turn.MaxCurvature * 0.05); i++)
            {
                trueEndIndex = i;
            }

            // Calculate curvature-limited speeds
            for (int i = trueStartIndex; i <= trueEndIndex; i++)
            {
                double curvature = Math.Abs(racingLine[i].Curvature);
                if (curvature > 0.0001)
                {
                    double maxSpeed = Math.Sqrt(maxLateralG * 9.81 / curvature);
                    profile.Speeds[i] = Math.Min(profile.Speeds[i], maxSpeed);
                    profile.LateralGs[i] = maxLateralG;
                }
            }

            // Propagate backwards from apex - ensure we can brake to apex speed
            for (int i = turn.ApexIndex - 1; i >= trueStartIndex; i--)
            {
                double distance = racingLine[i + 1].CumulativeDistance - racingLine[i].CumulativeDistance;
                if (distance > 0)
                {
                    double curvature = Math.Abs(racingLine[i].Curvature);
                    double requiredLateralG = 0.0;

                    if (curvature > 0.0001 && profile.Speeds[i] > 0.1)
                    {
                        requiredLateralG = (profile.Speeds[i] * profile.Speeds[i] * curvature) / 9.81;
                    }

                    double availableBrakingG = optimiser.GetMaxLongitudinalGForLateralG(requiredLateralG, true);

                    double maxSpeedFromBraking = Math.Sqrt(
                        profile.Speeds[i + 1] * profile.Speeds[i + 1] +
                        2 * availableBrakingG * 9.81 * distance);

                    double oldSpeed = profile.Speeds[i];
                    profile.Speeds[i] = Math.Max(profile.Speeds[i], maxSpeedFromBraking);

                    //System.Console.WriteLine($"{profile.Speeds[i] * profile.Speeds[i] * racingLine[i].Curvature / 9.81} at index {i}");
                    //System.Console.WriteLine($"{availableBrakingG} at index {i}");

                    if (curvature > 0.0001)
                    {
                        profile.LateralGs[i] = (profile.Speeds[i] * profile.Speeds[i] * racingLine[i].Curvature) / 9.81;
                    }
                    if (profile.Speeds[i] < oldSpeed)
                    {
                        profile.LongitudinalGs[i] = -availableBrakingG;
                        //System.Console.WriteLine($"{-availableBrakingG} at index {i}");
                    }
                }
            }

            // Propagate forwards from apex - ensure we can accelerate from apex speed
            for (int i = turn.ApexIndex + 1; i < trueEndIndex; i++)
            {
                double distance = racingLine[i].CumulativeDistance - racingLine[i - 1].CumulativeDistance;
                if (distance > 0)
                {
                    double curvature = Math.Abs(racingLine[i].Curvature);
                    double requiredLateralG = 0.0;

                    if (curvature > 0.0001 && profile.Speeds[i] > 0.1)
                    {
                        requiredLateralG = (profile.Speeds[i] * profile.Speeds[i] * curvature) / 9.81;
                    }

                    double availableAccelG = optimiser.GetMaxLongitudinalGForLateralG(requiredLateralG, false);

                    double maxSpeedFromAccel = Math.Sqrt(
                        profile.Speeds[i - 1] * profile.Speeds[i - 1] +
                        2 * availableAccelG * 9.81 * distance);

                    double oldSpeed = profile.Speeds[i];
                    profile.Speeds[i] = Math.Max(profile.Speeds[i], maxSpeedFromAccel);

                    if (curvature > 0.0001)
                    {
                        profile.LateralGs[i] = (profile.Speeds[i] * profile.Speeds[i] * racingLine[i].Curvature) / 9.81;
                    }

                    if (profile.Speeds[i] == oldSpeed)
                    {
                        profile.LongitudinalGs[i] = 0.0;
                    }
                    else if (profile.Speeds[i] > profile.Speeds[i - 1])
                    {
                        profile.LongitudinalGs[i] = availableAccelG;
                    }
                }
            }

            // Extend braking zone backwards beyond corner start
            for (int i = trueStartIndex - 1; i >= 0; i--)
            {
                double distance = racingLine[i + 1].CumulativeDistance - racingLine[i].CumulativeDistance;
                if (distance > 0)
                {
                    double maxSpeedFromBraking = Math.Sqrt(
                        profile.Speeds[i + 1] * profile.Speeds[i + 1] +
                        2 * maxBrakingG * 9.81 * distance);

                    profile.Speeds[i] = Math.Max(profile.Speeds[i], maxSpeedFromBraking);
                }

                profile.LongitudinalGs[i] = -maxBrakingG;

                if (profile.Speeds[i] >= terminalSpeedMs * 0.99) break;
            }

            // Extend acceleration zone forwards beyond corner exit
            for (int i = trueEndIndex + 1; i < racingLine.Length; i++)
            {
                double distance = racingLine[i].CumulativeDistance - racingLine[i - 1].CumulativeDistance;
                if (distance > 0)
                {
                    double maxSpeedFromAccel = Math.Sqrt(
                        profile.Speeds[i - 1] * profile.Speeds[i - 1] +
                        2 * maxAccelG * 9.81 * distance);

                    profile.Speeds[i] = Math.Max(profile.Speeds[i], maxSpeedFromAccel);
                }
                profile.LongitudinalGs[i] = maxAccelG;

                if (profile.Speeds[i] >= terminalSpeedMs * 0.99) break;
            }

            return profile;
        }

        private static (double[] speeds, double[] lateralGs, double[] longitudinalGs) ResolveSpeedAndGForceConflicts(SpeedProfile[] profiles, LineData[] racingLine, double maxAccelG)
        {
            var finalSpeeds = new double[racingLine.Length];
            var finalLateralGs = new double[racingLine.Length];
            var finalLongitudinalGs = new double[racingLine.Length];

            const double terminalSpeedMs = 100.0 * 0.44704;
            for (int i = 0; i < finalSpeeds.Length; i++)
            {
                finalSpeeds[i] = terminalSpeedMs;
                finalLateralGs[i] = 0.0;
                finalLongitudinalGs[i] = 0.0;
            }

            // Take minimum speed from all profiles
            foreach (var profile in profiles)
            {
                for (int i = 0; i < finalSpeeds.Length; i++)
                {
                    if (profile.Speeds[i] < finalSpeeds[i])
                    {
                        finalSpeeds[i] = profile.Speeds[i];
                        finalLateralGs[i] = profile.LateralGs[i];
                        finalLongitudinalGs[i] = profile.LongitudinalGs[i];
                    }
                }
            }

            // Set starting velocity to 0 and propagate acceleration limits forward
            finalSpeeds[0] = 0.0;
            finalLateralGs[0] = 0.0;
            finalLongitudinalGs[0] = 0.0;

            for (int i = 1; i < finalSpeeds.Length; i++)
            {
                double distance = racingLine[i].CumulativeDistance - racingLine[i - 1].CumulativeDistance;
                if (distance > 0)
                {
                    double maxSpeedFromAccel = Math.Sqrt(
                        finalSpeeds[i - 1] * finalSpeeds[i - 1] +
                        2 * maxAccelG * 9.81 * distance);

                    double oldSpeed = finalSpeeds[i];
                    finalSpeeds[i] = Math.Min(finalSpeeds[i], maxSpeedFromAccel);

                    double curvature = Math.Abs(racingLine[i].Curvature);
                    if (curvature > 0.0001)
                    {
                        finalLateralGs[i] = (finalSpeeds[i] * finalSpeeds[i] * curvature) / 9.81;
                    }

                    if (finalSpeeds[i] < oldSpeed - 0.01)
                    {
                        finalLongitudinalGs[i] = maxAccelG;
                    }
                    else if (finalSpeeds[i] > finalSpeeds[i - 1] + 0.01)
                    {
                        double deltaV = finalSpeeds[i] - finalSpeeds[i - 1];
                        double estimatedTime = distance / ((finalSpeeds[i] + finalSpeeds[i - 1]) / 2.0);
                        if (estimatedTime > 0)
                        {
                            double acceleration = deltaV / estimatedTime;
                            finalLongitudinalGs[i] = acceleration / 9.81;
                        }
                    }
                    else if (finalSpeeds[i] < finalSpeeds[i - 1] - 0.01)
                    {
                        double deltaV = finalSpeeds[i] - finalSpeeds[i - 1];
                        double estimatedTime = distance / ((finalSpeeds[i] + finalSpeeds[i - 1]) / 2.0);
                        if (estimatedTime > 0)
                        {
                            double deceleration = deltaV / estimatedTime;
                            finalLongitudinalGs[i] = deceleration / 9.81;
                        }
                    }
                }
            }

            return (finalSpeeds, finalLateralGs, finalLongitudinalGs);
        }

        private static void ApplySpeedProfileToRacingLine(double[] speeds, Turn[] turns)
        {
            for (int i = 0; i < turns.Length; i++)
            {
                var turn = turns[i];
                turn.EntrySpeed = speeds[turn.StartIndex] * 2.237;
                turn.ApexSpeed = speeds[turn.ApexIndex] * 2.237;
                turn.ExitSpeed = speeds[turn.EndIndex] * 2.237;

                Console.WriteLine($"Turn {i + 1} speeds - Entry: {turn.EntrySpeed:F1} mph, " +
                                $"Apex: {turn.ApexSpeed:F1} mph, Exit: {turn.ExitSpeed:F1} mph");
            }
        }

        private static double[] CalculateTimeProfile(double[] speeds, LineData[] racingLine)
        {
            double[] times = new double[racingLine.Length];
            times[0] = 0.0;

            for (int i = 1; i < racingLine.Length; i++)
            {
                double distance = racingLine[i].CumulativeDistance - racingLine[i - 1].CumulativeDistance;
                double avgSpeed = (speeds[i - 1] + speeds[i]) / 2.0;

                if (avgSpeed < 0.01) avgSpeed = 0.01;

                double deltaTime = distance / avgSpeed;
                times[i] = times[i - 1] + deltaTime;
            }

            return times;
        }

        private static void CreateOptimizationResults(AccelerationOptimiser optimiser, double[] speedProfile, double[] timeProfile, double[] lateralGs, double[] longitudinalGs, LineData[] racingLine)
        {
            var optimizationResults = new OptimizationResult[racingLine.Length];

            for (int i = 0; i < racingLine.Length; i++)
            {
                optimizationResults[i] = new OptimizationResult(
                    timeProfile[i],
                    speedProfile[i],
                    lateralGs[i],
                    longitudinalGs[i]);
            }

            optimiser.SetOptimizationResults(optimizationResults);
        }

        private static void ExportAndVisualizeSpeeds(AccelerationOptimiser optimiser, double[] speeds, double[] times)
        {
            optimiser.ExportSpeedData(speeds, times);

            // Look for Python visualization script
            string sourceDir = Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location) ?? "";
            string projectDir = Directory.GetParent(sourceDir)?.Parent?.Parent?.FullName ?? sourceDir;
            string scriptPath = Path.Combine(projectDir, "SpeedVisualiser.py");

            if (!File.Exists(scriptPath))
            {
                scriptPath = Path.Combine(sourceDir, "SpeedVisualiser.py");
            }

            if (File.Exists(scriptPath))
            {
                Console.WriteLine("Attempting to run Python visualization script automatically...");
                Console.WriteLine($"Running: python \"{Path.GetFileName(scriptPath)}\"");
                
                try
                {
                    var processInfo = new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = "python",
                        Arguments = $"\"{scriptPath}\"",
                        WorkingDirectory = Path.GetDirectoryName(scriptPath),
                        UseShellExecute = false,
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        CreateNoWindow = true
                    };

                    using (var process = System.Diagnostics.Process.Start(processInfo))
                    {
                        if (process != null)
                        {
                            process.WaitForExit(10000); // Wait up to 10 seconds
                            
                            if (process.ExitCode == 0)
                            {
                                Console.WriteLine("Python visualization script completed successfully.");
                            }
                            else
                            {
                                string error = process.StandardError.ReadToEnd();
                                Console.WriteLine($"Python script failed with exit code {process.ExitCode}");
                                if (!string.IsNullOrEmpty(error))
                                {
                                    Console.WriteLine($"Error: {error}");
                                }
                            }
                        }
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Failed to execute Python script: {ex.Message}");
                    Console.WriteLine("Make sure Python is installed and available in PATH.");
                }
            }
        }

        #region Helper Methods

        private static (double maxAccel, double maxBraking, double maxLateral) GetGGLimits(this AccelerationOptimiser optimiser)
        {
            var ggData = optimiser.GGDataPoints;
            if (ggData == null || ggData.Length == 0)
                return (0.0, 0.0, 0.0);

            double maxAccel = 0.0, maxBraking = 0.0, maxLateral = 0.0;

            foreach (var gg in ggData)
            {
                if (gg.LongitudinalG > maxAccel) maxAccel = gg.LongitudinalG;
                if (gg.LongitudinalG < maxBraking) maxBraking = gg.LongitudinalG;
                if (Math.Abs(gg.LateralG) > maxLateral) maxLateral = Math.Abs(gg.LateralG);
            }

            return (maxAccel, Math.Abs(maxBraking), maxLateral);
        }

        #endregion
    }
}