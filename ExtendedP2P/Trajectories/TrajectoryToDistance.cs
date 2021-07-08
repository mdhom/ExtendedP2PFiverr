using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;

[assembly:InternalsVisibleTo("Trajectories.Tests")]

namespace Trajectories
{
    public static class TrajectoryToDistance
    {
        public static TrajectoryToDistanceResult Calculate(double distance, double driveVelocity, double v0, double a0, MotionParameter motionParameter)
        {
            Trajectory trajectoryToDriveVelocity = TrajectoryToVelocity.Calculate(a0, v0, driveVelocity, motionParameter);
            Trajectory trajectoryToZero = TrajectoryToVelocity.Calculate(driveVelocity, 0, motionParameter);

            if (trajectoryToDriveVelocity.Length + trajectoryToZero.Length > distance)
            {
                // does not reach constant velocity in between -> modified ramps needed

                if (driveVelocity < v0)
                {
                    // brake down to zero without constant velocity in between
                    // is target reachable with default deceleration?
                    Trajectory trajectory = TrajectoryToVelocity.Calculate(a0, v0, 0, motionParameter);
                    if (trajectory.Length > distance)
                    {
                        // higher deceleration needed
                        // is it even possible to reach with twice the deceleration
                        trajectory = TrajectoryToVelocity.Calculate(a0, v0, 0, motionParameter.WithDifferentDeceleration(motionParameter.MaximumDeceleration * 2));
                        if (trajectory.Length > distance)
                        {
                            // not possible with even twice the deceleration
                            return new TrajectoryToDistanceResult(TrajectoryToDistanceCalculationStatus.Overshooting);
                        }
                        
                        // iterate to closest possible result
                        trajectory = IteratorDeceleration.Iterate(motionParameter, a0, v0, distance);
                    }
                    else if (trajectory.Length < distance)
                    {
                        // insert constant drive
                        double vAtAZero = TrajectoryToVelocity.v_at_a_zero(motionParameter, a0, v0);
                        double sAtAZero = TrajectoryToVelocity.s_at_a_zero(motionParameter, a0, v0);
                        trajectoryToZero = TrajectoryToVelocity.Calculate(vAtAZero, 0, motionParameter);
                        double totalDistance = sAtAZero + trajectoryToZero.Length;
                        double distanceDifference = distance - totalDistance;
                        
                        // constant drive
                        Trajectory constantTrajectory = TrajectoryToVelocity.Calculate(vAtAZero, vAtAZero, motionParameter);
                        constantTrajectory.Length = distanceDifference;
                        constantTrajectory.TotalDuration = constantTrajectory.Length / vAtAZero;

                        // limit first trajectory to only the part where acceleration is zeroed
                        trajectory.TotalDuration = TrajectoryToVelocity.t_at_a_zero(motionParameter, a0);
                        trajectory.Length = sAtAZero;
                        
                        return new TrajectoryToDistanceResult(new[] { trajectory, constantTrajectory, trajectoryToZero });
                    }
                    
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
                constantTrajectory.Length = distance - trajectoryToDriveVelocity.Length - trajectoryToZero.Length;
                constantTrajectory.TotalDuration = constantTrajectory.Length / driveVelocity;

                return new TrajectoryToDistanceResult(new[] { trajectoryToDriveVelocity, constantTrajectory, trajectoryToZero });
            }
        }
    }
}
