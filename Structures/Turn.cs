namespace DLS.Structures
{
    public struct Turn
    {
        public int StartIndex { get; set; }        // Start point of turn
        public int ApexIndex { get; set; }         // Apex (tightest point) of turn
        public int EndIndex { get; set; }          // End point of turn
        public double MaxCurvature { get; set; }   // Maximum curvature in turn
        public double TurnLength { get; set; }     // Length of turn in meters
        public string Direction { get; set; }      // "Left" or "Right"
        public double EntrySpeed { get; set; }     // Optimal entry speed
        public double ApexSpeed { get; set; }      // Optimal apex speed
        public double ExitSpeed { get; set; }      // Optimal exit speed
        public bool IsChicaneStart { get; set; }   // True if this turn starts a chicane
        public bool IsChicaneEnd { get; set; }     // True if this turn ends a chicane
        public int ChicaneTransitionIndex { get; set; } // Index of shared chicane endpoint (-1 if not part of chicane)
    }
}