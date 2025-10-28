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
            DetectTurns(optimiser, trackCentre, turns);

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

        private static void DetectTurns(this AccelerationOptimiser optimiser, LineData[] trackCentre, List<Turn> turns)
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
                            double angleTurned = 0.0;
                            for (int j = turnEndIndex; j >= turnStartIndex; j--)
                            {
                                if (angleTurned < ChicaneAngle(optimiser, Math.Abs(optimiser.TrackCentre[j].Curvature)))
                                {
                                    apexIndex = j;
                                }
                                angleTurned += Math.Abs(optimiser.TrackCentre[j].TangentDirection - optimiser.TrackCentre[j - 1].TangentDirection);
                                if (angleTurned >= Math.PI / 2)
                                    break;
                            }

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
                            double angleTurned = 0.0;
                            for (int j = turnStartIndex; j <= turnEndIndex; j++)
                            {
                                if (angleTurned < ChicaneAngle(optimiser, Math.Abs(optimiser.TrackCentre[j].Curvature)))
                                {
                                    apexIndex = j;
                                    //System.Console.WriteLine(ChicaneAngle(optimiser, optimiser.TrackCentre[j].Curvature));
                                }
                                angleTurned += Math.Abs(optimiser.TrackCentre[j].TangentDirection - optimiser.TrackCentre[j - 1].TangentDirection);
                                if (angleTurned >= Math.PI / 2)
                                    break;
                            }
                            //System.Console.WriteLine(apexIndex);
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

        #region Marker Generation

        public static void GenerateOptimizationMarkers(this AccelerationOptimiser optimiser)
        {
            var trackCentre = optimiser.TrackCentre;
            var turns = optimiser.Turns;
            int entryMarker, exitMarker;

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

                if (turn.IsChicaneStart)
                {
                    entryMarker = (int)Math.Ceiling(turn.StartIndex * optimiser.MarkersPerMeter / optimiser.IndicesPerMeter);
                    exitMarker = (int)Math.Floor(turn.ApexIndex * optimiser.MarkersPerMeter / optimiser.IndicesPerMeter);
                }
                else if (turn.IsChicaneEnd)
                {
                    entryMarker = (int)Math.Ceiling(turn.ApexIndex * optimiser.MarkersPerMeter / optimiser.IndicesPerMeter);
                    exitMarker = (int)Math.Floor(turn.EndIndex * optimiser.MarkersPerMeter / optimiser.IndicesPerMeter);
                }
                else
                {
                    entryMarker = (int)Math.Ceiling(turn.StartIndex * optimiser.MarkersPerMeter / optimiser.IndicesPerMeter);
                    exitMarker = (int)Math.Floor(turn.EndIndex * optimiser.MarkersPerMeter / optimiser.IndicesPerMeter);
                }
                int entryIndex = (int)(entryMarker * optimiser.IndicesPerMeter / optimiser.MarkersPerMeter);
                int exitIndex = (int)(exitMarker * optimiser.IndicesPerMeter / optimiser.MarkersPerMeter);

                int markerCount = exitMarker - entryMarker + 1;
                //System.Console.WriteLine(markerCount);

                double entryOffset = turn.IsChicaneEnd ? 1.0 - 2 * (turn.EndIndex - entryIndex) / (double)(turn.EndIndex - turn.ApexIndex) : 1.0;
                entryOffset *= turn.Direction == "Right" ? -1.0 : 1.0;

                double exitOffset = turn.IsChicaneStart ? 1.0 - 2 * (exitIndex - turn.StartIndex) / (double)(turn.ApexIndex - turn.StartIndex) : 1.0;
                exitOffset *= turn.Direction == "Right" ? -1.0 : 1.0;

                // Entry marker

                var (entryX, entryY) = optimiser.ApplyRacingLineOffset(entryIndex, entryOffset);
                markers.Add(new Marker
                {
                    X = entryX,
                    Y = entryY,
                    CumulativeDistance = trackCentre[entryIndex].CumulativeDistance,
                    TangentDirection = trackCentre[entryIndex].TangentDirection,
                    MarkerNumber = null,
                    Type = turn.IsChicaneEnd ? "Chicane Transition" : "Entry"
                });

                // Intermediate markers
                if (markerCount > 2)
                {
                    for (int seg = 1; seg < markerCount - 1; seg++)
                    {
                        int bestIndex = (int)((entryMarker + seg) * optimiser.IndicesPerMeter / optimiser.MarkersPerMeter);
                        double segmentOffset;

                        if (bestIndex < turn.ApexIndex)
                        {
                            segmentOffset = 2 * (turn.ApexIndex - bestIndex) / (double)(turn.ApexIndex - turn.StartIndex) - 1.0;
                            segmentOffset *= turn.Direction == "Right" ? -1.0 : 1.0;
                        }
                        else if (bestIndex > turn.ApexIndex)
                        {
                            segmentOffset = 2 * (bestIndex - turn.ApexIndex) / (double)(turn.EndIndex - turn.ApexIndex) - 1.0;
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
                            MarkerNumber = entryMarker + seg,
                            Type = "Corner Segment"
                        });
                    }
                }

                // Exit marker
                var (exitX, exitY) = optimiser.ApplyRacingLineOffset(exitIndex, exitOffset);
                markers.Add(new Marker
                {
                    X = exitX,
                    Y = exitY,
                    CumulativeDistance = trackCentre[exitIndex].CumulativeDistance,
                    TangentDirection = trackCentre[exitIndex].TangentDirection,
                    MarkerNumber = markerCount++,
                    Type = turn.IsChicaneStart ? "Chicane Transition" : "Exit"
                });
            }
            // Store markers in the optimiser
            optimiser.SetOptimizationMarkers(markers.ToArray());
            Console.WriteLine($"Generated {markers.Count} optimization markers for {turns.Count} turns");
            AddStraightSectionMarkers(optimiser);
        }

        private static double CalculateTotalAngleTurned(this AccelerationOptimiser optimiser, Turn turn)
        {
            double angleTurned = 0.0;
            for (int i = turn.StartIndex; i <= turn.EndIndex; i++)
            {
                angleTurned += Math.Abs(optimiser.TrackCentre[i].TangentDirection - optimiser.TrackCentre[i - 1].TangentDirection);
            }
            return angleTurned;
        }

        private static double ChicaneAngle(this AccelerationOptimiser optimiser, double curvature)
        {
            return Math.Acos(1/(1 + (optimiser.trackWidth-optimiser.carWidth) / 2 * curvature)); // straight line between two arcs w radius = (1/curvature)-(trackWidth-carWidth)/2
        }

        private static void AddStraightSectionMarkers(this AccelerationOptimiser optimiser)
        {
            var trackCentre = optimiser.TrackCentre;
            var turns = optimiser.Turns;
            
            if (turns == null || turns.Count == 0) return;

            var markers = new List<Marker>(optimiser.OptimizationMarkers ?? []);
            int originalMarkerCount = markers.Count;

            markers.Add(new Marker
            {
                X = trackCentre[0].X,
                Y = trackCentre[0].Y,
                CumulativeDistance = trackCentre[0].CumulativeDistance,
                TangentDirection = trackCentre[0].TangentDirection,
                Type = "Straight"
            });

            markers.Add(new Marker
            {
                X = trackCentre[trackCentre.Length - 1].X,
                Y = trackCentre[trackCentre.Length - 1].Y,
                CumulativeDistance = trackCentre[trackCentre.Length - 1].CumulativeDistance,
                TangentDirection = trackCentre[trackCentre.Length - 1].TangentDirection,
                Type = "Straight"
            });

            // Add markers for straight sections between turns
            for (int i = 0; i <= turns.Count; i++)
            {
                int sectionStart, sectionEnd;
                double startOffset, endOffset;

                if (i == 0)
                {
                    // First section: start of track to first turn
                    sectionStart = 1;
                    sectionEnd = turns[0].StartIndex;
                    startOffset = 0.0;
                    endOffset = turns[0].Direction == "Right" ? -1.0 : 1.0;
                }
                else if (i == turns.Count)
                {
                    // Last section: last turn to end of track
                    sectionStart = turns[i - 1].EndIndex;
                    sectionEnd = trackCentre.Length - 1;
                    startOffset = turns[i - 1].Direction == "Right" ? -1.0 : 1.0;
                    endOffset = 0.0;
                }
                else if (turns[i].IsChicaneEnd && turns[i - 1].IsChicaneStart)
                {
                    // Chicane section: between two chicanes
                    sectionStart = turns[i - 1].ApexIndex;
                    sectionEnd = turns[i].ApexIndex;
                    startOffset = turns[i - 1].Direction == "Right" ? 1.0 : -1.0;
                    endOffset = turns[i].Direction == "Right" ? 1.0 : -1.0;
                }
                else
                {
                    // Middle section: between two turns
                    sectionStart = turns[i - 1].EndIndex;
                    sectionEnd = turns[i].StartIndex;
                    startOffset = turns[i - 1].Direction == "Right" ? -1.0 : 1.0;
                    endOffset = turns[i].Direction == "Right" ? -1.0 : 1.0;
                }

                double sectionLength = trackCentre[sectionEnd].CumulativeDistance - 
                                     trackCentre[sectionStart].CumulativeDistance;

                // Only add markers for longer straight sections
                if (sectionLength > 5.0 || i == 0 || i == turns.Count)
                {
                    //System.Console.WriteLine($"startOffset: {startOffset}, endOffset: {endOffset}");
                    int firstMarker = (int)Math.Ceiling(sectionStart / optimiser.IndicesPerMeter * optimiser.MarkersPerMeter);
                    int lastMarker = (int)Math.Floor(sectionEnd / optimiser.IndicesPerMeter * optimiser.MarkersPerMeter);
                    int refPoint1 = 0, refPoint2 = 0;
                    int index1 = 10000;
                    int index2 = 0;
                    for (int m = 0; m < originalMarkerCount+2; m++)
                    {
                        int markerIndex = (int)Math.Round(markers[m].CumulativeDistance * optimiser.MarkersPerMeter);
                        if (markerIndex < index1 && markerIndex >= firstMarker - 1)
                        {
                            index1 = markerIndex;
                            refPoint1 = m;
                        }
                        if (markerIndex > index2 && markerIndex <= lastMarker + 1)
                        {
                            index2 = markerIndex;
                            refPoint2 = m;
                        }
                        //System.Console.WriteLine(markerIndex + " at " + m);
                    }
                    //System.Console.WriteLine(firstMarker + " to " + lastMarker);
                    //System.Console.WriteLine(refPoint1 + ", " + refPoint2);
                    int numMarkers = lastMarker - firstMarker + 1;
                    double p1x = markers[refPoint1].X;
                    double p1y = markers[refPoint1].Y;
                    double p2x = markers[refPoint2].X;
                    double p2y = markers[refPoint2].Y;
                    double grad = (p2y - p1y) / (p2x - p1x);
                    double intercept = p1y - grad * p1x;
                    int markerCount = 0;
                    //System.Console.WriteLine(p1x + ", " + p1y + " to " + p2x + ", " + p2y);

                    for (int j = 0; j < numMarkers; j++)
                    {
                        int markerIndex = (int)Math.Floor((firstMarker + j) * optimiser.IndicesPerMeter / optimiser.MarkersPerMeter);
                        double t = (markerIndex - sectionStart) / optimiser.IndicesPerMeter / sectionLength;
                        double markerX = t * (p2x - p1x) + p1x;
                        double markerY = grad * markerX + intercept;
                        //System.Console.WriteLine($"Marker {markerIndex}: offset {offset}");

                        //var (markerX, markerY) = optimiser.ApplyRacingLineOffset(markerIndex, offset);
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