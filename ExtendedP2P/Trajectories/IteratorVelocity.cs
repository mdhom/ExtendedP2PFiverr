using System;
using System.Collections.Generic;
using System.Text;

namespace Trajectories
{
    public static class IteratorVelocity
    {
        public const double ConstantVelocityReachedThreshold = 10.0;

        public static IteratorVelocityResult Iterate(MotionParameter motionParameter, double a0, double v0, double driveVelocity, double distance)
        {
            // accelerate as much as possible before braking
            IteratorVelocityResult result = Iterator.IterateFromMax(v0, driveVelocity, (driveVelocityIterated) =>
            {
                Trajectory rampToDriveVelocity = TrajectoryToVelocity.Calculate(a0, v0, driveVelocityIterated, motionParameter);
                Trajectory rampToZero = TrajectoryToVelocity.Calculate(driveVelocityIterated, 0, motionParameter);

                bool iterateDown = rampToDriveVelocity.Length + rampToZero.Length > distance;
                double resultingConstantVelocityDistance = distance - rampToDriveVelocity.Length - rampToZero.Length;
                bool endSearch = resultingConstantVelocityDistance >= 0.0 && resultingConstantVelocityDistance <= ConstantVelocityReachedThreshold;
                return new IteratorStepResult<IteratorVelocityResult>(iterateDown, endSearch, new IteratorVelocityResult(driveVelocityIterated, rampToDriveVelocity, rampToZero));
            });
            return result;
        }
    }

    public class IteratorVelocityResult
    {
        public double DriveVelocityIterated { get; set; }
        public Trajectory TrajectoryToDriveVelocityIterated { get; set; }
        public Trajectory TrajectoryToZero { get; set; }

        public IteratorVelocityResult(double driveVelocityIterated, Trajectory trajectoryToDriveVelocityIterated, Trajectory trajectoryToZero)
        {
            DriveVelocityIterated = driveVelocityIterated;
            TrajectoryToDriveVelocityIterated = trajectoryToDriveVelocityIterated;
            TrajectoryToZero = trajectoryToZero;
        }
    }
}
