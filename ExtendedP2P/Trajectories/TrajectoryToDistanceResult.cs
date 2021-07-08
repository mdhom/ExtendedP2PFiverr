namespace Trajectories
{
    public class TrajectoryToDistanceResult
    {
        public TrajectoryChain TrajectoryChain { get; }

        public TrajectoryToDistanceCalculationStatus CalculationStatus { get; }

        public TrajectoryToDistanceResult(TrajectoryToDistanceCalculationStatus status)
        {
            CalculationStatus = status;
        }

        public TrajectoryToDistanceResult(Trajectory[] ramps)
        {
            CalculationStatus = TrajectoryToDistanceCalculationStatus.Ok;
            TrajectoryChain = new TrajectoryChain(ramps);
        }
    }
}
