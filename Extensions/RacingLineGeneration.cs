using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Intrinsics.Arm;
using DLS.Structures;

namespace DLS.Extensions
{
    /// <summary>
    /// Extension methods for racing line generation and spline interpolation
    /// </summary>
    public static class RacingLineGeneration
    {
        public static void BuildRacingLine(this AccelerationOptimiser optimiser)
        {
            var trackCentre = optimiser.TrackCentre;
            var turns = optimiser.Turns;
            
            if (trackCentre == null || trackCentre.Length == 0)
                throw new InvalidOperationException("Track data not loaded. Call ReadTrackData() first.");

            if (turns == null || turns.Count == 0)
                throw new InvalidOperationException("Turn analysis not completed. Call AnalyzeTurns() first.");

            Console.WriteLine("Building optimized racing line using spline interpolation...");
            var startTime = DateTime.Now;

            // Generate key racing points with proper offsets
            var keyPoints = GenerateKeyRacingPoints(optimiser);

            // Create spline interpolation for smooth racing line
            var racingLine = InterpolateSplineRacingLine(optimiser, keyPoints, trackCentre);
            optimiser.SetRacingLine(racingLine);

            var buildTime = DateTime.Now - startTime;
            Console.WriteLine($"Racing line built in {buildTime.TotalMilliseconds:F1} ms with {racingLine.Length} points");
            Console.WriteLine($"Spline interpolates through {keyPoints.Count} key points from optimization markers");
        }

        private static List<(int index, double x, double y, double tangentDirection, string type)> GenerateKeyRacingPoints(AccelerationOptimiser optimiser)
        {
            // Use the optimization markers that were already generated
            var keyPoints = new List<(int index, double x, double y, double tangentDirection, string type)>();
            var trackCentre = optimiser.TrackCentre;
            var optimizationMarkers = optimiser.OptimizationMarkers;

            // Convert optimization markers to key points
            if (optimizationMarkers != null && optimizationMarkers.Length > 0)
            {
                Console.WriteLine($"Using {optimizationMarkers.Length} optimization markers for racing line generation");
                
                for (int i = 0; i < optimizationMarkers.Length; i++)
                {
                    var marker = optimizationMarkers[i];
                    
                    // Find corresponding track index by cumulative distance
                    int trackIndex = FindClosestTrackIndex(marker, trackCentre);

                    keyPoints.Add((trackIndex, marker.X, marker.Y, marker.TangentDirection, marker.Type + " " + marker.MarkerNumber.ToString()));
                }
            }
            else
            {
                Console.WriteLine("No optimization markers found, generating basic key points from turns");
                var turns = optimiser.Turns;

                // Fallback: Add key points for each turn (entry, apex, exit)
                if (turns != null && turns.Count > 0)
                {
                    foreach (var turn in turns)
                    {
                        // Entry point
                        var (entryX, entryY) = optimiser.ApplyRacingLineOffset(turn.StartIndex, -0.5);
                        double entryTangent = trackCentre[turn.StartIndex].TangentDirection;
                        keyPoints.Add((turn.StartIndex, entryX, entryY, entryTangent, $"Turn Entry"));

                        // Apex point
                        var (apexX, apexY) = optimiser.ApplyRacingLineOffset(turn.ApexIndex, 0.5);
                        double apexTangent = trackCentre[turn.ApexIndex].TangentDirection;
                        keyPoints.Add((turn.ApexIndex, apexX, apexY, apexTangent, $"Turn Apex"));

                        // Exit point
                        var (exitX, exitY) = optimiser.ApplyRacingLineOffset(turn.EndIndex, -0.5);
                        double exitTangent = trackCentre[turn.EndIndex].TangentDirection;
                        keyPoints.Add((turn.EndIndex, exitX, exitY, exitTangent, $"Turn Exit"));
                    }
                }
            }

            // Add start and end points if not already included
            bool hasStart = keyPoints.Any(p => p.index == 0);
            bool hasEnd = keyPoints.Any(p => p.index == trackCentre.Length - 1);

            if (!hasStart)
            {
                keyPoints.Insert(0, (0, trackCentre[0].X, trackCentre[0].Y, trackCentre[0].TangentDirection, "Start"));
            }

            if (!hasEnd)
            {
                int lastIndex = trackCentre.Length - 1;
                keyPoints.Add((lastIndex, trackCentre[lastIndex].X, trackCentre[lastIndex].Y, trackCentre[lastIndex].TangentDirection, "Finish"));
            }

            // Sort by track index
            keyPoints.Sort((a, b) => a.index.CompareTo(b.index));

            // Remove duplicates
            var uniqueKeyPoints = new List<(int index, double x, double y, double tangentDirection, string type)>();
            int lastAddedIndex = -1;

            foreach (var point in keyPoints)
            {
                if (point.index != lastAddedIndex)
                {
                    uniqueKeyPoints.Add(point);
                    lastAddedIndex = point.index;
                }
            }

            return uniqueKeyPoints;
        }
        private static LineData[] InterpolateSplineRacingLine(this AccelerationOptimiser optimiser, List<(int index, double x, double y, double tangentDirection, string type)> keyPoints, LineData[] trackCentre)
        {
            if (keyPoints.Count < 2)
                throw new ArgumentException("Need at least 2 key points for spline interpolation");

            Console.WriteLine("Using Catmull-Rom spline interpolation for smooth racing line...");
            
            var racingLine = new LineData[trackCentre.Length];
            var sortedKeyPoints = keyPoints.OrderBy(p => p.index).ToList();
            int lastSegmentIndex = -1;

            // Use Catmull-Rom spline interpolation
            for (int i = 0; i < trackCentre.Length; i++)
            {
                double racingX, racingY;

                int segmentIndex = FindSplineSegment(i, sortedKeyPoints);
                
                if (segmentIndex == -1)
                {
                    racingX = LinearInterpolate(i, sortedKeyPoints, out racingY);
                }
                else
                {
                    var (x, y) = CatmullRomInterpolate(i, segmentIndex, sortedKeyPoints);
                    //var (x, y , last) = BezierInterpolate(i, segmentIndex, sortedKeyPoints, lastSegmentIndex);
                    racingX = x;
                    racingY = y;
                    //lastSegmentIndex = last;
                }

                racingLine[i] = new LineData
                {
                    X = racingX,
                    Y = racingY,
                    CumulativeDistance = 0.0,
                    Curvature = 0.0,
                    TangentDirection = 0.0
                };
            }

            CalculateRacingLineProperties(optimiser, racingLine);
            Console.WriteLine("Catmull-Rom spline interpolation completed - smooth curves through markers");
            
            return racingLine;
        }

        private static int FindSplineSegment(int trackIndex, List<(int index, double x, double y, double tangentDirection, string type)> keyPoints)
        {
            for (int j = 1; j < keyPoints.Count - 2; j++)
            {
                if (trackIndex >= keyPoints[j].index && trackIndex <= keyPoints[j + 1].index)
                {
                    return j;
                }
            }
            return -1;
        }

        private static double LinearInterpolate(int trackIndex, List<(int index, double x, double y, double tangentDirection, string type)> keyPoints, out double y)
        {
            if (trackIndex <= keyPoints[0].index)
            {
                y = keyPoints[0].y;
                return keyPoints[0].x;
            }
            
            if (trackIndex >= keyPoints[keyPoints.Count - 1].index)
            {
                y = keyPoints[keyPoints.Count - 1].y;
                return keyPoints[keyPoints.Count - 1].x;
            }

            for (int j = 0; j < keyPoints.Count - 1; j++)
            {
                if (trackIndex >= keyPoints[j].index && trackIndex <= keyPoints[j + 1].index)
                {
                    var p1 = keyPoints[j];
                    var p2 = keyPoints[j + 1];
                    
                    if (p1.index == p2.index)
                    {
                        y = p1.y;
                        return p1.x;
                    }
                    
                    double t = (double)(trackIndex - p1.index) / (p2.index - p1.index);
                    t = Math.Max(0.0, Math.Min(1.0, t));
                    
                    y = p1.y + t * (p2.y - p1.y);
                    return p1.x + t * (p2.x - p1.x);
                }
            }
            
            y = keyPoints[0].y;
            return keyPoints[0].x;
        }

        private static (double x, double y) CatmullRomInterpolate(int trackIndex, int segmentIndex,
                                                                List<(int index, double x, double y, double tangentDirection, string type)> keyPoints)
        {
            var p0 = keyPoints[segmentIndex - 1];
            var p1 = keyPoints[segmentIndex];
            var p2 = keyPoints[segmentIndex + 1];
            var p3 = keyPoints[segmentIndex + 2];

            double t;
            if (p2.index == p1.index)
            {
                t = 0.0;
            }
            else
            {
                t = (double)(trackIndex - p1.index) / (p2.index - p1.index);
                t = Math.Max(0.0, Math.Min(1.0, t));
            }

            double t2 = t * t;
            double t3 = t2 * t;

            double x = 0.5 * ((2.0 * p1.x) +
                              (-p0.x + p2.x) * t +
                              (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2 +
                              (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);

            double y = 0.5 * ((2.0 * p1.y) +
                              (-p0.y + p2.y) * t +
                              (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2 +
                              (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);

            return (x, y);
        }

        private static (double x, double y) newCatmullRomInterpolate(int trackIndex, int segmentIndex,
                                                                List<(int index, double x, double y, double tangentDirection, string type)> keyPoints)
        {
            var p0 = keyPoints[segmentIndex - 1];
            var p1 = keyPoints[segmentIndex];
            var p2 = keyPoints[segmentIndex + 1];
            var p3 = keyPoints[segmentIndex + 2];

            var m1 = (x: 1, y: 0.5 * ((p2.y - p1.y) / (p2.x - p1.x) + (p1.y - p0.y) / (p1.x - p0.x)));
            var m2 = (x: 1, y: 0.5 * ((p3.y - p2.y) / (p3.x - p2.x) + (p2.y - p1.y) / (p2.x - p1.x)));

            double t;
            if (p2.index == p1.index)
            {
                t = 0.0;
            }
            else
            {
                t = (double)(trackIndex - p1.index) / (p2.index - p1.index);
                t = Math.Max(0.0, Math.Min(1.0, t));
            }
            System.Console.WriteLine($"Catmull-Rom t for track index {trackIndex} between key points {segmentIndex} and {segmentIndex + 1}: t = {t:F2}");

            double t2 = t * t;
            double t3 = t2 * t;

            double x =  (2.0 * t3 - 3.0 * t2 + 1.0) * p1.x +
                        (t3 - 2.0 * t2 + t) * m1.x +
                        (-2.0 * t3 + 3.0 * t2) * p2.x +
                        (t3 - t2) * m2.x;

            double y =  (2.0 * t3 - 3.0 * t2 + 1.0) * p1.y +
                        (t3 - 2.0 * t2 + t) * m1.y +
                        (-2.0 * t3 + 3.0 * t2) * p2.y +
                        (t3 - t2) * m2.y;

            return (x, y);
        }

        private static (double x, double y, int lastSegmentIndex) BezierInterpolate(int trackIndex, int segmentIndex,
                                                                List<(int index, double x, double y, double tangentDirection, string type)> keyPoints, int lastSegmentIndex = -1)
        {
            var p0 = keyPoints[segmentIndex];
            var p1 = keyPoints[segmentIndex + 1];
            var theta0 = p0.tangentDirection;
            var theta1 = p1.tangentDirection;
            if (lastSegmentIndex != segmentIndex)
            {
                System.Console.WriteLine($"Slopes for segment {segmentIndex}: theta0 = {theta0:F2}, theta1 = {theta1:F2}");
            }
            var m0 = Math.Tan(Math.PI / 2 - theta0);
            var m1 = Math.Tan(Math.PI / 2 - theta1);
            var c0 = p0.y - m0 * p0.x;
            var c1 = p1.y - m1 * p1.x;
            var pmx = (c1 - c0) / (m0 - m1);
            var pmy = m0 * pmx + c0;
            var simpleGrad = (p1.y - p0.y) / (p1.x - p0.x);

            if (lastSegmentIndex != segmentIndex)
            {
                System.Console.WriteLine(((pmx < p0.x && pmx < p1.x) || (pmx > p0.x && pmx > p1.x))||((pmy < p0.y && pmy < p1.y) || (pmy > p0.y && pmy > p1.y)));
                System.Console.WriteLine($"Bezier control points for segment {segmentIndex}: (p0)({p0.x:F2}, {p0.y:F2}), (pm)({pmx:F2}, {pmy:F2}), (p1)({p1.x:F2}, {p1.y:F2})");
            }

            if ((pmx < p0.x && pmx < p1.x) || (pmx > p0.x && pmx > p1.x))
            {
                pmx = 0.5 * (p0.x + p1.x);
                pmy = simpleGrad * pmx + p0.y - simpleGrad * p0.x;
            }
            else if ((pmy < p0.y && pmy < p1.y) || (pmy > p0.y && pmy > p1.y))
            {
                pmx = 0.5 * (p0.x + p1.x);
                pmy = simpleGrad * pmx + p0.y - simpleGrad * p0.x;
            }

            if (lastSegmentIndex != segmentIndex)
            {
                System.Console.WriteLine(((pmx < p0.x && pmx < p1.x) || (pmx > p0.x && pmx > p1.x))||((pmy < p0.y && pmy < p1.y) || (pmy > p0.y && pmy > p1.y)));
                System.Console.WriteLine($"Bezier control points for segment {segmentIndex}: (p0)({p0.x:F2}, {p0.y:F2}), (pm)({pmx:F2}, {pmy:F2}), (p1)({p1.x:F2}, {p1.y:F2})");
                lastSegmentIndex = segmentIndex;
            }

            var pm = (x: pmx, y: pmy);

            double t;
            if (p1.index == p0.index)
            {
                t = 0.0;
            }
            else
            {
                t = (double)(trackIndex - p0.index) / (p1.index - p0.index);
                t = Math.Max(0.0, Math.Min(1.0, t));
            }

            double x = (1 - t) * (1 - t) * p0.x +
                              2 * (1-t) * t * pm.x +
                              t * t * p1.x;

            double y = (1 - t) * (1 - t) * p0.y +
                              2 * (1-t) * t * pm.y +
                              t * t * p1.y;

            return (x, y, lastSegmentIndex);
        }

        private static void CalculateRacingLineProperties(this AccelerationOptimiser optimiser, LineData[] racingLine)
        {
            int n = racingLine.Length;
            double maxAbsCurvature = 0.0;
            for (int i = 0; i < optimiser.Turns.Count; i++)
            {
                var turn = optimiser.Turns[i];
                if (Math.Abs(turn.MaxCurvature) > maxAbsCurvature)
                    maxAbsCurvature = Math.Abs(turn.MaxCurvature);    
            }
            
            // Calculate cumulative distances
            racingLine[0].CumulativeDistance = 0.0;
            for (int i = 1; i < n; i++)
            {
                double dx = racingLine[i].X - racingLine[i - 1].X;
                double dy = racingLine[i].Y - racingLine[i - 1].Y;
                double segmentDistance = Math.Sqrt(dx * dx + dy * dy);
                racingLine[i].CumulativeDistance = racingLine[i - 1].CumulativeDistance + segmentDistance;
            }

            // Calculate tangent directions and curvature
            for (int i = 0; i < n; i++)
            {
                int prevIndex = (int)Math.Max(0, i - 0.5 * optimiser.IndicesPerMeter);
                int nextIndex = (int)Math.Min(n - 1, i + 0.5 * optimiser.IndicesPerMeter);

                double dx = racingLine[nextIndex].X - racingLine[prevIndex].X;
                double dy = racingLine[nextIndex].Y - racingLine[prevIndex].Y;

                if (prevIndex == nextIndex)
                {
                    racingLine[i].TangentDirection = 0.0;
                    racingLine[i].Curvature = 0.0;
                    continue;
                }

                double standardAngle = Math.Atan2(dy, dx);
                racingLine[i].TangentDirection = -1 * standardAngle + Math.PI / 2.0;

                // Normalize to [-π, π] range
                if (racingLine[i].TangentDirection > Math.PI)
                    racingLine[i].TangentDirection -= 2.0 * Math.PI;
                else if (racingLine[i].TangentDirection < -Math.PI)
                    racingLine[i].TangentDirection += 2.0 * Math.PI;

                // Calculate curvature
                if (i > 0 && i < n - 1)
                {
                    double x1 = racingLine[prevIndex].X;
                    double y1 = racingLine[prevIndex].Y;
                    double x2 = racingLine[i].X;
                    double y2 = racingLine[i].Y;
                    double x3 = racingLine[nextIndex].X;
                    double y3 = racingLine[nextIndex].Y;

                    double area = Math.Abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
                    double a = Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
                    double b = Math.Sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
                    double c = Math.Sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));

                    if (a > 1e-10 && b > 1e-10 && c > 1e-10 && area > 1e-10)
                    {
                        double radius = (a * b * c) / (4.0 * area);
                        racingLine[i].Curvature = Math.Min(1.0 / radius, 1.2 * maxAbsCurvature);

                        double v1x = x2 - x1, v1y = y2 - y1;
                        double v2x = x3 - x2, v2y = y3 - y2;
                        double crossProduct = v1x * v2y - v1y * v2x;

                        if (crossProduct < 0)
                            racingLine[i].Curvature = Math.Abs(racingLine[i].Curvature);
                        else
                            racingLine[i].Curvature = -Math.Abs(racingLine[i].Curvature);
                    }
                    else
                    {
                        racingLine[i].Curvature = 0.0;
                    }
                }
                else
                {
                    racingLine[i].Curvature = 0.0;
                }
            }

            Console.WriteLine($"Racing line properties calculated: total length = {racingLine[n - 1].CumulativeDistance:F2}m");
        }

        private static int FindClosestTrackIndex(Marker marker, LineData[] trackCentre)
        {
            // Find the track index that has the closest cumulative distance to the marker
            double targetDistance = marker.CumulativeDistance;
            int closestIndex = 0;
            double closestDistanceDiff = Math.Abs(trackCentre[0].CumulativeDistance - targetDistance);

            for (int i = 1; i < trackCentre.Length; i++)
            {
                double distanceDiff = Math.Abs(trackCentre[i].CumulativeDistance - targetDistance);
                if (distanceDiff < closestDistanceDiff)
                {
                    closestDistanceDiff = distanceDiff;
                    closestIndex = i;
                }
            }

            return closestIndex;
        }
    }
}