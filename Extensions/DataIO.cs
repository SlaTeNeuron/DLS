using System;
using System.IO;
using System.Linq;
using DLS.Structures;

namespace DLS.Extensions
{
    /// <summary>
    /// Extension methods for data input/output operations including track data, GG data, 
    /// caching, and result export functionality
    /// </summary>
    public static class DataIO
    {
        #region Track Data Operations
        
        public static void ReadTrackData(this AccelerationOptimiser optimiser)
        {
            Console.Write("Enter track file path (or press Enter for default)\n[InputData/SkidPadTrackMap.csv]: ");
            string? input = Console.ReadLine();
            string trackFile = string.IsNullOrWhiteSpace(input) ? "InputData/SkidPadTrackMap.csv" : input;

            Console.WriteLine($"Loading track data from: {trackFile}");
            var startTime = DateTime.Now;
            optimiser.LoadTrackData(trackFile);
            var loadTime = DateTime.Now - startTime;
            
            Console.WriteLine($"Loaded {optimiser.GetTrackPointCount()} track points in {loadTime.TotalMilliseconds:F1} ms");
        }

        public static bool LoadProcessedTrackData(this AccelerationOptimiser optimiser, string filePath)
        {
            string[] lines = File.ReadAllLines(filePath);

            if (lines.Length < 2)
                throw new InvalidDataException("Cached track file must contain at least a header and one data row");

            int dataRows = lines.Length - 1;
            var trackCentre = new LineData[dataRows];

            // Parse cached data (already in full format)
            for (int i = 1; i <= dataRows; i++)
            {
                string[] values = lines[i].Split(',');

                if (values.Length < 5)
                {
                    throw new InvalidDataException($"Cached track file has malformed line {i}: {lines[i]}");
                }

                int arrayIndex = i - 1;
                trackCentre[arrayIndex] = new LineData
                {
                    X = ParseDoubleWithFallback(values[0], 0.0, $"X at cached line {i}"),
                    Y = ParseDoubleWithFallback(values[1], 0.0, $"Y at cached line {i}"),
                    CumulativeDistance = ParseDoubleWithFallback(values[2], 0.0, $"CumulativeDistance at cached line {i}"),
                    Curvature = ParseDoubleWithFallback(values[3], 0.0, $"Curvature at cached line {i}"),
                    TangentDirection = ParseDoubleWithFallback(values[4], 0.0, $"TangentDirection at cached line {i}")
                };
            }

            optimiser.SetTrackCentre(trackCentre);
            Console.WriteLine($"Loaded {dataRows} track points from cache");
            return true;
        }

        public static void SaveProcessedTrackData(this AccelerationOptimiser optimiser, string filePath)
        {
            var trackCentre = optimiser.TrackCentre;
            if (trackCentre == null)
                throw new InvalidOperationException("Track data not loaded. Cannot save processed track data.");

            try
            {
                using (var writer = new StreamWriter(filePath))
                {
                    // Write header
                    writer.WriteLine("X,Y,CumulativeDistance,Curvature,TangentDirection");

                    // Write data
                    foreach (var point in trackCentre)
                    {
                        writer.WriteLine($"{point.X:F6},{point.Y:F6},{point.CumulativeDistance:F6},{point.Curvature:F8},{point.TangentDirection:F8}");
                    }
                }
                Console.WriteLine($"Saved processed track data to {filePath}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Warning: Could not save processed track data to {filePath}: {ex.Message}");
            }
        }

        public static void LoadTrackData(this AccelerationOptimiser optimiser, string filePath)
        {
            if (!File.Exists(filePath))
                throw new FileNotFoundException($"Track file not found: {filePath}");

            string cachedDataPath = GenerateCacheFilePath(filePath);

            if (File.Exists(cachedDataPath))
            {
                Console.Write("Cached file detected do you want to use the data previously computed from this track?\n[Y/N]: ");
                string? input = Console.ReadLine();
                if (input?.Trim().ToUpper() == "Y")
                {
                    optimiser.LoadProcessedTrackData(cachedDataPath);
                    return;
                }
            }

            string[] lines = File.ReadAllLines(filePath);
            if (lines.Length < 2)
                throw new InvalidDataException("File must contain at least a header and one data row");

            int dataRows = lines.Length - 1;
            var trackCentre = new LineData[dataRows];
            int xIndex = -1;
            int yIndex = -1;
            bool isLatLon = false;

            string[] firstDataValues = lines[0].Split(',');
            for (int i = 0; i < firstDataValues.Length; i++)
            {
                firstDataValues[i] = firstDataValues[i].Trim().ToLower();
                //System.Console.WriteLine(firstDataValues[i]);
                if (firstDataValues[i] == "x" || firstDataValues[i] == "lat") xIndex = i;
                if (firstDataValues[i] == "y" || firstDataValues[i] == "lon") yIndex = i;
                if (firstDataValues[i] == "lon") isLatLon = true; // Support for 'Longitude' header
            }
            if (xIndex == -1 || yIndex == -1)
                throw new InvalidDataException("File header must contain 'X' and 'Y' columns or 'Lat' and 'Lon' columns.");

            System.Console.WriteLine(isLatLon ? "Detected Latitude/Longitude format" : "Detected X/Y coordinate format");

            double x0 = ParseDoubleWithFallback(lines[1].Split(',')[xIndex], 0.0, $"X at line {1}");
            double y0 = ParseDoubleWithFallback(lines[1].Split(',')[yIndex], 0.0, $"Y at line {1}");

            // Parse X,Y coordinates only and calculate derived properties
            for (int i = 1; i <= dataRows; i++)
            {
                string[] values = lines[i].Split(',');
                if (values.Length < 2) continue;

                int arrayIndex = i - 1;
                trackCentre[arrayIndex] = new LineData
                {
                    X = isLatLon ? 113200 * (ParseDoubleWithFallback(values[xIndex], 0.0, $"X at line {i}") - x0) : ParseDoubleWithFallback(values[xIndex], 0.0, $"X at line {i}") - x0,
                    Y = isLatLon ? 113200 * Math.Cos(y0 * Math.PI / 180) * (ParseDoubleWithFallback(values[yIndex], 0.0, $"Y at line {i}") - y0) : ParseDoubleWithFallback(values[yIndex], 0.0, $"Y at line {i}") - y0,
                    CumulativeDistance = 0.0,
                    Curvature = 0.0,
                    TangentDirection = 0.0
                };
            }
            optimiser.SetTrackCentre(trackCentre);
            optimiser.CalculateTrackProperties();
            optimiser.SaveProcessedTrackData(cachedDataPath);
        }

        #endregion

        #region GG Data Operations

        public static void ReadGGData(this AccelerationOptimiser optimiser)
        {
            Console.Write("Enter GG data file path (or press Enter for default)\n[InputData/GGData.csv]: ");
            string? input = Console.ReadLine();
            string ggFile = string.IsNullOrWhiteSpace(input) ? "InputData/GGData.csv" : input;

            var startTime = DateTime.Now;
            optimiser.LoadGGData(ggFile);

            if (GetGGDataCount(optimiser) == 0)
            {
                optimiser.GenerateSyntheticCircularGGData();
            }

            var loadTime = DateTime.Now - startTime;
            Console.WriteLine($"Loaded {GetGGDataCount(optimiser)} GG data points in {loadTime.TotalMilliseconds:F1} ms");
        }

        public static void LoadGGData(this AccelerationOptimiser optimiser, string filePath)
        {
            if (!File.Exists(filePath))
            {
                Console.WriteLine($"Warning: GG data file not found: {filePath}");
                Console.WriteLine("Will generate synthetic circular GG data instead.");
                optimiser.SetGGData(Array.Empty<GGData>());
                return;
            }

            string[] lines = File.ReadAllLines(filePath);
            if (lines.Length < 2)
            {
                optimiser.SetGGData(Array.Empty<GGData>());
                return;
            }

            string header = lines[0].ToLower();
            bool isXYFormat = header.Contains("x") && header.Contains("y");

            int dataRows = lines.Length - 1;
            var ggData = new GGData[dataRows];

            for (int i = 1; i < lines.Length; i++)
            {
                string[] values = lines[i].Split(',');
                if (values.Length < 2) continue;

                int arrayIndex = i - 1;

                if (isXYFormat)
                {
                    double x = ParseDoubleWithFallback(values[0], 0.0, $"X GG at line {i}");
                    double y = ParseDoubleWithFallback(values[1], 0.0, $"Y GG at line {i}");
                    
                    ggData[arrayIndex] = new GGData();
                    ggData[arrayIndex].SetFromXY(x, y);
                }
                else
                {
                    double magnitude = ParseDoubleWithFallback(values[0], 0.0, $"GG magnitude at line {i}");
                    double angle = ParseDoubleWithFallback(values[1], 0.0, $"GG angle at line {i}");

                    ggData[arrayIndex] = new GGData
                    {
                        Magnitude = magnitude,
                        Angle = angle
                    };

                    var (x, y) = ggData[arrayIndex].GetXYComponents();
                    ggData[arrayIndex].LongitudinalG = x;
                    ggData[arrayIndex].LateralG = y;
                }
            }

            optimiser.SetGGData(ggData);
            optimiser.CreateSortedGGData();
        }

        public static void GenerateSyntheticCircularGGData(this AccelerationOptimiser optimiser)
        {
            const double maxG = 1.8;
            const int numPoints = 360;

            var ggData = new GGData[numPoints];

            for (int i = 0; i < numPoints; i++)
            {
                double angle = -Math.PI + (2.0 * Math.PI * i) / numPoints;

                ggData[i] = new GGData
                {
                    Magnitude = maxG,
                    Angle = angle
                };

                var (x, y) = ggData[i].GetXYComponents();
                ggData[i].LongitudinalG = x;
                ggData[i].LateralG = y;
            }

            Console.WriteLine($"Generated synthetic circular GG data: {numPoints} points at {maxG:F1}g magnitude");
            Console.WriteLine("Angle convention: 0=forward, +π/2=right, -π/2=left, ±π=backward");
            
            optimiser.SetGGData(ggData);
        }

        #endregion

        #region Result Export

        public static void ExportSpeedData(this AccelerationOptimiser optimiser, double[] speeds, double[] times, string outputPath = "OutputData/OptimizedSpeeds.csv")
        {
            var racingLine = optimiser.GetRacingLine();
            var optimizationResults = optimiser.GetOptimizationResults();
            var trackCentre = optimiser.TrackCentre;
            
            if (racingLine == null || optimizationResults == null)
                throw new InvalidOperationException("Racing line and optimization results must be calculated first.");

            string? directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }

            using (var writer = new StreamWriter(outputPath))
            {
                writer.WriteLine("X,Y,Velocity_ms,Velocity_mph,Time_s,CumulativeDistance,Curvature,LateralG,LongitudinalG,TotalG,trackX,trackY,trackTangentDirection");

                for (int i = 0; i < racingLine.Length; i++)
                {
                    double velocityMph = speeds[i] * 2.237;
                    var result = optimizationResults[i];
                    var trackPoint = trackCentre != null && i < trackCentre.Length ? trackCentre[i] : new LineData();
                    writer.WriteLine($"{racingLine[i].X:F6},{racingLine[i].Y:F6},{speeds[i]:F3},{velocityMph:F1}," +
                                   $"{times[i]:F3},{racingLine[i].CumulativeDistance:F2},{racingLine[i].Curvature:F6}," +
                                   $"{result.LateralG:F6},{result.LongitudinalG:F6},{result.TotalG:F6}," +
                                   $"{trackPoint.X:F6},{trackPoint.Y:F6},{trackPoint.TangentDirection:F6}");
                }
            }

            Console.WriteLine($"Speed data exported ({racingLine.Length} points) in {(DateTime.Now - DateTime.Now).TotalMilliseconds:F1} ms");
        }

        public static void ExportOptimizationMarkers(this AccelerationOptimiser optimiser, string outputPath = "OutputData/RacingLineMarkers.csv")
        {
            var markers = optimiser.OptimizationMarkers;
            var trackCentre = optimiser.TrackCentre;
            
            if (markers == null || markers.Length == 0)
                throw new InvalidOperationException("Optimization markers must be generated first.");

            string? directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }

            using (var writer = new StreamWriter(outputPath))
            {
                writer.WriteLine("Index,X,Y,Type,Description");

                for (int i = 0; i < markers.Length; i++)
                {
                    var marker = markers[i];
                    
                    // Generate a description based on the marker type and properties
                    string description = GenerateMarkerDescription(optimiser, marker, i, trackCentre);
                    
                    // Use the cumulative distance as index, or fall back to array index
                    int index = (int)Math.Min(Math.Round(marker.CumulativeDistance * 100), trackCentre.Length - 1); // Convert to centimeters for index
                    
                    writer.WriteLine($"{index},{marker.X:F6},{marker.Y:F6},{marker.Type ?? "Unknown"},\"{description}\"");
                }
            }

            Console.WriteLine($"Optimization markers exported ({markers.Length} points) to {outputPath}");
        }

        #endregion

        #region Helper Methods

        private static int GetTrackPointCount(this AccelerationOptimiser optimiser)
        {
            return optimiser.TrackCentre?.Length ?? 0;
        }

        private static int GetGGDataCount(this AccelerationOptimiser optimiser)
        {
            return optimiser.GGDataPoints?.Length ?? 0;
        }

        private static void CreateSortedGGData(this AccelerationOptimiser optimiser)
        {
            var ggData = optimiser.GGDataPoints;
            if (ggData == null || ggData.Length == 0) return;

            // Sort the original array in-place by angle for efficient lookup
            Array.Sort(ggData, (a, b) => a.Angle.CompareTo(b.Angle));

            Console.WriteLine($"Sorted GG data in-place for efficient lookup ({ggData.Length} points)");
        }

        private static void CalculateTrackProperties(this AccelerationOptimiser optimiser)
        {
            var trackCentre = optimiser.TrackCentre;
            if (trackCentre == null || trackCentre.Length < 3)
            {
                Console.WriteLine("Warning: Not enough points to calculate track properties");
                return;
            }

            int n = trackCentre.Length;
            System.Console.WriteLine(n);

            // Check if track is a closed loop
            double startX = trackCentre[0].X;
            double startY = trackCentre[0].Y;
            double endX = trackCentre[n - 1].X;
            double endY = trackCentre[n - 1].Y;
            double closeDistance = Math.Sqrt((endX - startX) * (endX - startX) + (endY - startY) * (endY - startY));

            // Consider closed if endpoints are within 1 meter of each other
            bool isClosedTrack = closeDistance < 1.0;

            Console.WriteLine($"Track analysis: {(isClosedTrack ? "Closed loop" : "Open track")} detected (start-end distance: {closeDistance:F2}m)");

            // Calculate cumulative distances
            trackCentre[0].CumulativeDistance = 0.0;
            for (int i = 1; i < n; i++)
            {
                double dx = trackCentre[i].X - trackCentre[i - 1].X;
                double dy = trackCentre[i].Y - trackCentre[i - 1].Y;
                double segmentDistance = Math.Sqrt(dx * dx + dy * dy);
                trackCentre[i].CumulativeDistance = trackCentre[i - 1].CumulativeDistance + segmentDistance;
            }

            // Calculate tangent directions and curvature using 3-point method
            for (int i = 0; i < n; i++)
            {
                // Get indices based on track type
                int prevIndex, nextIndex;

                if (isClosedTrack)
                {
                    // Use wrap-around for closed tracks
                    prevIndex = (i - 1 + n) % n;
                    nextIndex = (i + 1) % n;
                }
                else
                {
                    prevIndex = Math.Max(i - 5, 0);
                    nextIndex = Math.Min(i + 5, n - 1);
                }

                // Calculate tangent direction using adjacent points
                double dx, dy;

                if (!isClosedTrack && (i == 0 || i == n - 1))
                {
                    // For open track endpoints, use single-sided difference
                    if (i == 0)
                    {
                        dx = trackCentre[5].X - trackCentre[0].X;
                        dy = trackCentre[5].Y - trackCentre[0].Y;
                    }
                    else // i == n - 1
                    {
                        dx = trackCentre[n - 1].X - trackCentre[n - 6].X;
                        dy = trackCentre[n - 1].Y - trackCentre[n - 6].Y;
                    }
                }
                else
                {
                    // Use centered difference for interior points
                    dx = trackCentre[nextIndex].X - trackCentre[prevIndex].X;
                    dy = trackCentre[nextIndex].Y - trackCentre[prevIndex].Y;
                }

                // Handle case where dx and dy are both zero
                if (Math.Abs(dx) < 1e-10 && Math.Abs(dy) < 1e-10)
                {
                    // Use single-sided difference
                    if (i < n - 1)
                    {
                        dx = trackCentre[i + 1].X - trackCentre[i].X;
                        dy = trackCentre[i + 1].Y - trackCentre[i].Y;
                    }
                    else
                    {
                        dx = trackCentre[i].X - trackCentre[i - 1].X;
                        dy = trackCentre[i].Y - trackCentre[i - 1].Y;
                    }
                }

                // Convert to GG data coordinate system: +Y=0, +X=+π/2, -X=-π/2, -Y=±π
                // Standard atan2(dy, dx) gives: +X=0, +Y=+π/2, -X=±π, -Y=-π/2
                // We need to rotate by -π/2 to align with GG convention
                double standardAngle = Math.Atan2(dy, dx);
                trackCentre[i].TangentDirection = -1 * standardAngle + Math.PI / 2.0;

                // Normalize to [-π, π] range
                if (trackCentre[i].TangentDirection > Math.PI)
                    trackCentre[i].TangentDirection -= 2.0 * Math.PI;
                else if (trackCentre[i].TangentDirection < -Math.PI)
                    trackCentre[i].TangentDirection += 2.0 * Math.PI;

                // Calculate curvature using 3-point circle method
                // For open track endpoints, set curvature to zero since we can't reliably calculate it
                if (!isClosedTrack && (i == 0 || i == n - 1))
                {
                    trackCentre[i].Curvature = 0.0; // Assume straight at endpoints
                }
                else
                {
                    double x1 = trackCentre[prevIndex].X;
                    double y1 = trackCentre[prevIndex].Y;
                    double x2 = trackCentre[i].X;
                    double y2 = trackCentre[i].Y;
                    double x3 = trackCentre[nextIndex].X;
                    double y3 = trackCentre[nextIndex].Y;

                    // Calculate curvature using the formula: K = 2*Area / (a*b*c)
                    // where Area is the area of triangle formed by 3 points
                    // and a, b, c are the side lengths
                    double area = Math.Abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);

                    double a = Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
                    double b = Math.Sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
                    double c = Math.Sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));

                    // Avoid division by zero
                    if (a > 1e-10 && b > 1e-10 && c > 1e-10 && area > 1e-10)
                    {
                        double radius = (a * b * c) / (4.0 * area);
                        trackCentre[i].Curvature = 1.0 / radius;

                        // Determine sign of curvature (left turn negative, right turn positive)
                        // Cross product to determine turning direction
                        double v1x = x2 - x1;
                        double v1y = y2 - y1;
                        double v2x = x3 - x2;
                        double v2y = y3 - y2;
                        double crossProduct = v1x * v2y - v1y * v2x;

                        if (crossProduct < 0) // Right turn
                            trackCentre[i].Curvature = Math.Abs(trackCentre[i].Curvature);
                        else // Left turn
                            trackCentre[i].Curvature = -Math.Abs(trackCentre[i].Curvature);
                    }
                    else
                    {
                        trackCentre[i].Curvature = 0.0; // Straight section or degenerate case
                    }
                }
            }

            Console.WriteLine($"Calculated track properties: {n} points, total distance = {trackCentre[n - 1].CumulativeDistance:F2}m");
        }

        private static double ParseDoubleWithFallback(string value, double fallback, string context = "")
        {
            if (double.TryParse(value, out double result))
                return result;

            if (!string.IsNullOrEmpty(context))
            {
                Console.WriteLine($"Warning: Could not parse '{value}' as double for {context}, using {fallback}");
            }
            return fallback;
        }

        private static string GenerateCacheFilePath(string originalFilePath)
        {
            string directory = Path.GetDirectoryName(originalFilePath) ?? "";
            string fileNameWithoutExt = Path.GetFileNameWithoutExtension(originalFilePath);
            string fileContent = File.ReadAllText(originalFilePath);

            uint hash = 2166136261u;
            foreach (byte b in System.Text.Encoding.UTF8.GetBytes(fileContent))
            {
                hash ^= b;
                hash *= 16777619u;
            }

            string hashString = hash.ToString("X8");
            string cacheFileName = $"{fileNameWithoutExt}_{hashString}.cache.csv";

            return Path.Combine(directory, cacheFileName);
        }

        private static string GenerateMarkerDescription(this AccelerationOptimiser optimiser, Marker marker, int markerNum, LineData[]? trackCentre)
        {
            string description = "";
            
            switch (marker.Type?.ToLower())
            {
                case "entry":
                    description = "Turn Entry";
                    break;
                case "exit":
                    description = "Turn Exit";
                    break;
                case "segment":
                    description = "Intermediate";
                    if (marker.MarkerNumber.HasValue)
                        description += $" @{marker.MarkerNumber.Value * 22.5:F1}° ({marker.MarkerNumber}/{32})";
                    break;
                case "straight":
                    description = "Straight Section";
                    break;
                case "chicane transition":
                    description = "Chicane Transition";
                    break;
                default:
                    if (markerNum == 0)
                        description = "Start";
                    else
                        description = marker.Type ?? "Unknown";
                    break;
            }

            // Add additional context if available
            if (trackCentre != null && markerNum <= Math.Ceiling(trackCentre.Length / optimiser.IndicesPerMeter * optimiser.MarkersPerMeter))
            {
                if (Math.Abs(trackCentre[(int)Math.Min(trackCentre.Length - 1, Math.Round(markerNum * optimiser.IndicesPerMeter / optimiser.MarkersPerMeter, 0))].Curvature) < 0.01)
                    description += " (Straight)";
                else if (trackCentre[(int)Math.Min(Math.Round(markerNum * optimiser.IndicesPerMeter / optimiser.MarkersPerMeter, 0), trackCentre.Length - 1)].Curvature > 0)
                    description += " (Right Turn)";
                else
                    description += " (Left Turn)";
            }
            return description;
        }

        #endregion
    }
}