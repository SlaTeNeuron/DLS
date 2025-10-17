using System;
using System.Collections.Generic;
using System.Linq;
using DLS.Structures;

namespace DLS.Extensions
{
    /// <summary>
    /// Extension methods for turn detection and analysis
    /// </summary>
    public static class TurnAnalysis
    {
        #region Main Turn Analysis

        public static void DetectAndAnalyzeTurns(this AccelerationOptimiser optimiser)
        {
            var trackCentre = optimiser.TrackCentre;
            if (trackCentre == null || trackCentre.Length == 0)
                throw new InvalidOperationException("Track data not loaded. Call ReadTrackData() first.");

            Console.WriteLine("Analyzing turns in track...");
            var startTime = DateTime.Now;

            var turns = new List<Turn>();
            DetectTurns(trackCentre, turns);

            optimiser.SetTurns(turns);

            var analysisTime = DateTime.Now - startTime;
            Console.WriteLine($"Turn analysis completed in {analysisTime.TotalMilliseconds:F1} ms");
            Console.WriteLine($"Found {turns.Count} turns on track");

            // Print turn summary
            for (int i = 0; i < turns.Count; i++)
            {
                var turn = turns[i];
                Console.WriteLine($"Turn {i + 1}: {turn.Direction} turn, " +
                                $"Length: {turn.TurnLength:F1}m, " +
                                $"Max Curvature: {turn.MaxCurvature:F4}, " +
                                $"Points: {turn.StartIndex}-{turn.EndIndex}");
            }
        }

        #endregion

        #region Turn Detection Logic

        private static void DetectTurns(LineData[] trackCentre, List<Turn> turns)
        {
            const double curvatureThreshold = 0.001;
            const int minTurnLength = 10;

            bool inTurn = false;
            int turnStartIndex = 0;
            double maxAbsCurvature = 0.0;
            int apexIndex = 0;
            bool currentTurnIsRight = false;

            for (int i = 0; i < trackCentre.Length; i++)
            {
                double curvature = trackCentre[i].Curvature;
                double absCurvature = Math.Abs(curvature);
                bool isRightTurn = curvature > 0;

                if (!inTurn && absCurvature > curvatureThreshold)
                {
                    // Start of a new turn
                    inTurn = true;
                    turnStartIndex = i;
                    maxAbsCurvature = absCurvature;
                    apexIndex = i;
                    currentTurnIsRight = isRightTurn;
                }
                else if (inTurn && absCurvature > curvatureThreshold)
                {
                    // Check for direction change (chicane detection)
                    if (isRightTurn != currentTurnIsRight)
                    {
                        // Direction changed - this is a chicane
                        int chicaneTransitionIndex = i;
                        int turnEndIndex = chicaneTransitionIndex;
                        int turnLengthPoints = turnEndIndex - turnStartIndex + 1;

                        if (turnLengthPoints >= minTurnLength)
                        {
                            int apexStart = turnStartIndex, apexEnd = turnStartIndex;
                            bool apexStarted = false;
                            for (int j = turnStartIndex; j <= turnEndIndex; j++)
                            {
                                double pointCurvature = Math.Abs(trackCentre[j].Curvature);
                                if (pointCurvature > 0.9 * maxAbsCurvature)
                                {
                                    if (!apexStarted)
                                    {
                                        apexStart = j;
                                        apexStarted = true;
                                    }
                                    apexEnd = j;
                                }
                            }
                            apexIndex = (apexStart + apexEnd) / 2;
                            
                            AddChicaneTurnToList(turns, trackCentre, turnStartIndex, apexIndex, turnEndIndex, 
                                               maxAbsCurvature, currentTurnIsRight, true, false, chicaneTransitionIndex);
                        }

                        // Start new turn (second part of chicane)
                        turnStartIndex = chicaneTransitionIndex;
                        maxAbsCurvature = absCurvature;
                        apexIndex = i;
                        currentTurnIsRight = isRightTurn;
                    }
                    else
                    {
                        // Continue in same direction turn
                        if (absCurvature > maxAbsCurvature)
                        {
                            maxAbsCurvature = absCurvature;
                        }
                    }
                }
                else if (inTurn && absCurvature <= curvatureThreshold)
                {
                    // End of turn
                    int turnEndIndex = i - 1;
                    int turnLengthPoints = turnEndIndex - turnStartIndex + 1;

                    if (turnLengthPoints >= minTurnLength)
                    {
                        int apexStart = turnStartIndex, apexEnd = turnStartIndex;
                        bool apexStarted = false;
                        for (int j = turnStartIndex; j <= turnEndIndex; j++)
                        {
                            double pointCurvature = Math.Abs(trackCentre[j].Curvature);
                            if (pointCurvature > 0.9 * maxAbsCurvature)
                            {
                                if (!apexStarted)
                                {
                                    apexStart = j;
                                    apexStarted = true;
                                }
                                apexEnd = j;
                            }
                        }
                        apexIndex = (apexStart + apexEnd) / 2;
                        
                        // Check if this is the end of a chicane
                        bool isChicaneEnd = turns.Count > 0 && turns[turns.Count - 1].IsChicaneStart;
                        int chicaneTransitionIndex = isChicaneEnd ? turnStartIndex : -1;
                        
                        if (isChicaneEnd)
                        {
                            AddChicaneTurnToList(turns, trackCentre, turnStartIndex, apexIndex, turnEndIndex, 
                                               maxAbsCurvature, currentTurnIsRight, false, true, chicaneTransitionIndex);
                        }
                        else
                        {
                            AddTurnToList(turns, trackCentre, turnStartIndex, apexIndex, turnEndIndex, 
                                        maxAbsCurvature, currentTurnIsRight);
                        }
                    }

                    inTurn = false;
                }
            }

            // Handle case where track ends in a turn
            if (inTurn)
            {
                int turnEndIndex = trackCentre.Length - 1;
                int turnLengthPoints = turnEndIndex - turnStartIndex + 1;

                if (turnLengthPoints >= minTurnLength)
                {
                    bool isChicaneEnd = turns.Count > 0 && turns[turns.Count - 1].IsChicaneStart;
                    int chicaneTransitionIndex = isChicaneEnd ? turnStartIndex : -1;
                    
                    if (isChicaneEnd)
                    {
                        AddChicaneTurnToList(turns, trackCentre, turnStartIndex, apexIndex, turnEndIndex, 
                                           maxAbsCurvature, currentTurnIsRight, false, true, chicaneTransitionIndex);
                    }
                    else
                    {
                        AddTurnToList(turns, trackCentre, turnStartIndex, apexIndex, turnEndIndex, 
                                    maxAbsCurvature, currentTurnIsRight);
                    }
                }
            }
        }

        private static void AddTurnToList(List<Turn> turns, LineData[] trackCentre, int startIndex, int apexIndex, 
                                        int endIndex, double maxCurvature, bool isRightTurn)
        {
            double turnLength = trackCentre[endIndex].CumulativeDistance - trackCentre[startIndex].CumulativeDistance;
            string direction = isRightTurn ? "Right" : "Left";

            turns.Add(new Turn
            {
                StartIndex = startIndex,
                ApexIndex = apexIndex,
                EndIndex = endIndex,
                MaxCurvature = maxCurvature,
                TurnLength = turnLength,
                Direction = direction,
                EntrySpeed = 0.0,
                ApexSpeed = 0.0,
                ExitSpeed = 0.0,
                IsChicaneStart = false,
                IsChicaneEnd = false,
                ChicaneTransitionIndex = -1
            });
        }

        private static void AddChicaneTurnToList(List<Turn> turns, LineData[] trackCentre, int startIndex, int apexIndex, 
                                               int endIndex, double maxCurvature, bool isRightTurn, 
                                               bool isChicaneStart, bool isChicaneEnd, int chicaneTransitionIndex)
        {
            double turnLength = trackCentre[endIndex].CumulativeDistance - trackCentre[startIndex].CumulativeDistance;
            string direction = isRightTurn ? "Right" : "Left";

            turns.Add(new Turn
            {
                StartIndex = startIndex,
                ApexIndex = apexIndex,
                EndIndex = endIndex,
                MaxCurvature = maxCurvature,
                TurnLength = turnLength,
                Direction = direction,
                EntrySpeed = 0.0,
                ApexSpeed = 0.0,
                ExitSpeed = 0.0,
                IsChicaneStart = isChicaneStart,
                IsChicaneEnd = isChicaneEnd,
                ChicaneTransitionIndex = chicaneTransitionIndex
            });
        }

        #endregion

        private static double CalculateTurnAngle(LineData[] trackCentre, Turn turn)
        {
            if (trackCentre == null || turn.StartIndex >= trackCentre.Length || turn.EndIndex >= trackCentre.Length)
                return 0.0;

            double totalAngle = 0.0;
            for (int i = turn.StartIndex; i < turn.EndIndex; i++)
            {
                int nextIndex = i + 1;
                if (nextIndex < trackCentre.Length)
                {
                    double angle1 = Math.Atan2(trackCentre[i].Y - trackCentre[Math.Max(0, i - 1)].Y, 
                                             trackCentre[i].X - trackCentre[Math.Max(0, i - 1)].X);
                    double angle2 = Math.Atan2(trackCentre[nextIndex].Y - trackCentre[i].Y, 
                                             trackCentre[nextIndex].X - trackCentre[i].X);

                    double angleDiff = angle2 - angle1;
                    if (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
                    if (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

                    totalAngle += angleDiff;
                }
            }

            return Math.Abs(totalAngle);
        }

        #region Marker Generation

        public static void GenerateOptimizationMarkers(this AccelerationOptimiser optimiser)
        {
            var trackCentre = optimiser.TrackCentre;
            var turns = optimiser.Turns;
            
            if (turns == null || turns.Count == 0)
            {
                Console.WriteLine("No turns detected. Generating basic straight-line markers.");
                return;
            }

            var markers = new List<Marker>();

            // Generate turn markers for each turn
            for (int turnIdx = 0; turnIdx < turns.Count; turnIdx++)
            {
                Turn turn = turns[turnIdx];

                double totalCumulativeAngle = CalculateTurnAngle(trackCentre, turn);
                int segmentCount = Math.Max(1, (int)Math.Ceiling(totalCumulativeAngle / (Math.PI / 8)));

                double entryOffset = turn.Direction == "Right" ? -1.0 : 1.0;
                double exitOffset = turn.Direction == "Right" ? -1.0 : 1.0;
                double chicaneTransitionOffset = 0.0;
                int markerCount = 1;

                // Entry marker
                if (!turn.IsChicaneEnd)
                {
                    var (entryX, entryY) = optimiser.ApplyRacingLineOffset(turn.StartIndex, turn.IsChicaneEnd ? chicaneTransitionOffset : entryOffset);
                    markers.Add(new Marker
                    {
                        X = entryX,
                        Y = entryY,
                        CumulativeDistance = trackCentre[turn.StartIndex].CumulativeDistance,
                        TangentDirection = trackCentre[turn.StartIndex].TangentDirection,
                        MarkerNumber = null,
                        Type = turn.IsChicaneEnd ? "Chicane Transition" : "Entry"
                    });
                }

                // Curve weight points for long corners
                if (segmentCount > 2)
                {
                    for (int seg = 1; seg < segmentCount - 1; seg++)
                    {
                        double t = (double)seg / (segmentCount - 1);
                        int targetIndex = turn.StartIndex + (int)(t * (turn.EndIndex - turn.StartIndex));

                        int bestIndex = turn.StartIndex;
                        double maxCurvature = 0.0;
                        int searchStart = Math.Max(turn.StartIndex, targetIndex - 2);
                        int searchEnd = Math.Min(turn.EndIndex, targetIndex + 2);

                        for (int idx = searchStart; idx <= searchEnd; idx++)
                        {
                            double curvature = Math.Abs(trackCentre[idx].Curvature);
                            if (curvature > maxCurvature)
                            {
                                maxCurvature = curvature;
                                bestIndex = idx;
                            }
                        }

                        double segmentOffset;

                        if (bestIndex < turn.ApexIndex)
                        {
                            if (turn.IsChicaneEnd)
                            {
                                segmentOffset = (turn.ApexIndex - bestIndex) / (double)(3 * (turn.ApexIndex - turn.StartIndex)) - 1.0;
                            }
                            else
                            {
                                segmentOffset = 2 * (turn.ApexIndex - bestIndex) / (double)(turn.ApexIndex - turn.StartIndex) - 1.0;
                            }
                            segmentOffset *= turn.Direction == "Right" ? -1.0 : 1.0;
                        }
                        else if (bestIndex > turn.ApexIndex)
                        {
                            if (turn.IsChicaneStart)
                            {
                                segmentOffset = (bestIndex - turn.EndIndex) / (double)(3 * (turn.EndIndex - turn.ApexIndex)) - 2.0 / 3.0;
                                System.Console.WriteLine(segmentOffset + " " + markerCount);
                            }
                            else
                            {
                                segmentOffset = 2 * (bestIndex - turn.ApexIndex) / (double)(turn.EndIndex - turn.ApexIndex) - 1.0;
                            }
                            segmentOffset *= turn.Direction == "Right" ? -1.0 : 1.0;
                        }
                        else
                        {
                            segmentOffset = turn.Direction == "Right" ? 1.0 : -1.0; // Apex point
                        }
                        var (segmentX, segmentY) = optimiser.ApplyRacingLineOffset(bestIndex, segmentOffset);
                        markers.Add(new Marker
                        {
                            X = segmentX,
                            Y = segmentY,
                            CumulativeDistance = trackCentre[bestIndex].CumulativeDistance,
                            TangentDirection = trackCentre[bestIndex].TangentDirection,
                            MarkerNumber = markerCount++,
                            Type = "Segment"
                        });
                    }
                }

                // Exit marker
                if (!turn.IsChicaneStart)
                {
                    var (exitX, exitY) = optimiser.ApplyRacingLineOffset(turn.EndIndex, turn.IsChicaneStart ? chicaneTransitionOffset : exitOffset);
                    markers.Add(new Marker
                    {
                        X = exitX,
                        Y = exitY,
                        CumulativeDistance = trackCentre[turn.EndIndex].CumulativeDistance,
                        TangentDirection = trackCentre[turn.EndIndex].TangentDirection,
                        MarkerNumber = markerCount++,
                        Type = turn.IsChicaneStart ? "Chicane Transition" : "Exit"
                    });
                }
            }

            // Store markers in the optimiser
            optimiser.SetOptimizationMarkers(markers.ToArray());
            Console.WriteLine($"Generated {markers.Count} optimization markers for {turns.Count} turns");
            AddStraightSectionMarkers(optimiser);
        }

        private static void AddStraightSectionMarkers(this AccelerationOptimiser optimiser)
        {
            var trackCentre = optimiser.TrackCentre;
            var turns = optimiser.Turns;
            
            if (turns == null || turns.Count == 0) return;

            var markers = new List<Marker>(optimiser.OptimizationMarkers ?? []);

            // Add markers for straight sections between turns
            for (int i = 0; i <= turns.Count; i++)
            {
                int sectionStart, sectionEnd;
                Turn? upcomingTurn = null;

                if (i == 0)
                {
                    // First section: start of track to first turn
                    sectionStart = 0;
                    sectionEnd = turns[0].StartIndex;
                    upcomingTurn = turns[0];
                }
                else if (i == turns.Count)
                {
                    // Last section: last turn to end of track
                    sectionStart = turns[i - 1].EndIndex;
                    sectionEnd = trackCentre.Length - 1;
                    upcomingTurn = null;
                }
                else
                {
                    // Middle section: between two turns
                    sectionStart = turns[i - 1].EndIndex;
                    sectionEnd = turns[i].StartIndex;
                    upcomingTurn = turns[i];
                }

                double sectionLength = trackCentre[sectionEnd].CumulativeDistance - 
                                     trackCentre[sectionStart].CumulativeDistance;

                // Only add markers for longer straight sections
                if (sectionLength > 50.0 || i == 0 || i == turns.Count)
                {
                    int numMarkers = Math.Max(1, (int)(sectionLength / 100.0));
                    if (i == 0 || i == turns.Count)
                    {
                        //numMarkers++;
                    }
                    for (int j = 1; j <= numMarkers; j++)
                    {
                        int markerCount = 0;
                        double t;
                        if (i == 0)
                        {
                            t = j == 1 ? 0.5 : 0.875;
                        }
                        else if (i == turns.Count)
                        {
                            t = j == 1 ? 0.5 : 0.25;
                        }
                        else
                        {
                            t = (double)j / (numMarkers + 1);
                        }
                        int markerIndex = sectionStart + (int)(t * (sectionEnd - sectionStart));

                        // Calculate offset based on upcoming turn (for racing line positioning)
                        double offset = 0; // Straight sections use center line
                        //if (t < 0.3) // Near the end of straight before turn
                        //{
                            bool isRightTurn = turns[Math.Max(i - 1, 0)].Direction == "Right";
                            offset = j == 1 ? 0.5 : 0.8;
                        offset *= isRightTurn ? -1 : 1; // Position for optimal turn entry
                        //}
                        /*
                        if (t > 0.7)
                        {
                            bool isRightTurn = turns[i].Direction == "Right";
                            offset = j == 1 ? 0.5 : 0.93;
                            offset *= isRightTurn ? -1 : 1; // Position for optimal turn exit
                        }*/
                        var (markerX, markerY) = optimiser.ApplyRacingLineOffset(markerIndex, offset);
                        markers.Add(new Marker
                        {
                            X = markerX,
                            Y = markerY,
                            CumulativeDistance = trackCentre[markerIndex].CumulativeDistance,
                            TangentDirection = trackCentre[markerIndex].TangentDirection,
                            MarkerNumber = markerCount++,
                            Type = "Straight"
                        });
                    }
                }
            }

            optimiser.SetOptimizationMarkers(markers.ToArray());
        }

        #endregion

        #region Racing Line Offset Calculations

        public static (double X, double Y) ApplyRacingLineOffset(this AccelerationOptimiser optimiser, int index, double offset)
        {
            var trackCentre = optimiser.TrackCentre;
            if (index < 0 || index >= trackCentre.Length)
                return (trackCentre[Math.Max(0, Math.Min(index, trackCentre.Length - 1))].X, 
                       trackCentre[Math.Max(0, Math.Min(index, trackCentre.Length - 1))].Y);

            // Calculate the direction perpendicular to the track at this point
            double directionX, directionY;

            if (index == 0)
            {
                // Use direction to next point
                directionX = trackCentre[1].X - trackCentre[0].X;
                directionY = trackCentre[1].Y - trackCentre[0].Y;
            }
            else if (index == trackCentre.Length - 1)
            {
                // Use direction from previous point
                directionX = trackCentre[index].X - trackCentre[index - 1].X;
                directionY = trackCentre[index].Y - trackCentre[index - 1].Y;
            }
            else
            {
                // Use average direction from previous and to next point
                double dx1 = trackCentre[index].X - trackCentre[index - 1].X;
                double dy1 = trackCentre[index].Y - trackCentre[index - 1].Y;
                double dx2 = trackCentre[index + 1].X - trackCentre[index].X;
                double dy2 = trackCentre[index + 1].Y - trackCentre[index].Y;

                directionX = (dx1 + dx2) / 2.0;
                directionY = (dy1 + dy2) / 2.0;
            }

            // Normalize the direction vector
            double length = Math.Sqrt(directionX * directionX + directionY * directionY);
            if (length > 0)
            {
                directionX /= length;
                directionY /= length;
            }

            // Calculate perpendicular vector (rotate 90 degrees counterclockwise)
            double perpX = directionY;
            double perpY = -directionX;

            // Apply offset
            double newX = trackCentre[index].X + offset * perpX;
            double newY = trackCentre[index].Y + offset * perpY;

            return (newX, newY);
        }

        #endregion

        #region Helper Methods

        public static int GetTurnCount(this AccelerationOptimiser optimiser)
        {
            return optimiser.Turns?.Count ?? 0;
        }

        public static int GetOptimizationMarkerCount(this AccelerationOptimiser optimiser)
        {
            return optimiser.OptimizationMarkers?.Length ?? 0;
        }

        private static bool IsChicaneTransitionPoint(int pointIndex, AccelerationOptimiser optimiser)
        {
            var turns = optimiser.Turns;
            if (turns == null) return false;

            foreach (var turn in turns)
            {
                if (turn.ChicaneTransitionIndex == pointIndex)
                    return true;
            }
            return false;
        }

        #endregion
    }
}