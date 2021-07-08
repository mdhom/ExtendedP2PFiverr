using System;

namespace LiveMotionGenerator
{
    public enum MotionPhase
    {
        Acclerate,
        Drive,
        Brake,
        Stop
    }

    public record MotionStatus(double Jerk, double Acceleration, double Velocity, double Distance, double BrakingDistance, double BrakingVelocity, MotionPhase MotionPhase);
}
