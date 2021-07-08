using System;
using System.Collections.Generic;
using System.Text;

namespace Trajectories
{
    public static class TrajectoryToDistance
    {
        public static TrajectoryToDistanceResult Calculate(double distance, double driveVelocity, double v0, double a0, MotionParameter motionParameter)
        {
            Trajectory rampToDriveVelocity = TrajectoryToVelocity.Calculate(a0, v0, driveVelocity, motionParameter);
            Trajectory rampToZero = TrajectoryToVelocity.Calculate(driveVelocity, 0, motionParameter);

            if (rampToDriveVelocity.Length + rampToZero.Length > distance)
            {
                // does not reach constant velocity in between -> modified ramps needed

                if (driveVelocity < v0)
                {
                    // brake down to zero without constant velocity in between
                    Trajectory result = TrajectoryToVelocity.Calculate(a0, v0, 0, motionParameter.WithDifferentDeceleration(motionParameter.MaximumDeceleration * 2));
                    if (result.Length > distance)
                    {
                        return new TrajectoryToDistanceResult(TrajectoryToDistanceCalculationStatus.Overshooting);
                    }

                    Trajectory trajectory = IteratorDeceleration.Iterate(motionParameter, a0, v0, distance);
                    return new TrajectoryToDistanceResult(new[] { trajectory });
                }
                else
                {
                    // accelerate as much as possible before braking
                    IteratorVelocityResult itResult = IteratorVelocity.Iterate(motionParameter, a0, v0, driveVelocity, distance);
                    Trajectory rampConstant = TrajectoryToVelocity.Calculate(itResult.DriveVelocityIterated, itResult.DriveVelocityIterated, motionParameter);
                    rampConstant.Length = distance - itResult.TrajectoryToDriveVelocityIterated.Length - itResult.TrajectoryToZero.Length;
                    rampConstant.TotalDuration = rampConstant.Length / driveVelocity;

                    return new TrajectoryToDistanceResult(new[] { itResult.TrajectoryToDriveVelocityIterated, rampConstant, itResult.TrajectoryToZero });
                }
            }
            else
            {
                // constant velocity is reached

                // calculate constant trajectory
                Trajectory constantTrajectory = TrajectoryToVelocity.Calculate(driveVelocity, driveVelocity, motionParameter);
                constantTrajectory.Length = distance - rampToDriveVelocity.Length - rampToZero.Length;
                constantTrajectory.TotalDuration = constantTrajectory.Length / driveVelocity;

                return new TrajectoryToDistanceResult(new[] { rampToDriveVelocity, constantTrajectory, rampToZero });
            }
        }
    }
}
