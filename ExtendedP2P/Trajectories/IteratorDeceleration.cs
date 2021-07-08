using System;
using System.Collections.Generic;
using System.Text;

namespace Trajectories
{
    public static class IteratorDeceleration
    {
        public const double DistanceReachedThreshold = 0.5;

        public static Trajectory Iterate(MotionParameter motionParameter, double a0, double v0, double distance)
        {
            Trajectory trajectory = Iterator.IterateFromMax(2 * motionParameter.MaximumDeceleration, motionParameter.MaximumDeceleration, (decIterated) =>
            {
                Trajectory trajectory = TrajectoryToVelocity.Calculate(a0, v0, 0, motionParameter.WithDifferentDeceleration(decIterated));
                bool iterateDown = trajectory.Length > distance;
                bool endSearch = Math.Abs(trajectory.Length - distance) < DistanceReachedThreshold;
                return new IteratorStepResult<Trajectory>(iterateDown, endSearch, trajectory);
            });
            return trajectory;
        }
    }
}
