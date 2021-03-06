
namespace Trajectories
{
    public struct MotionParameter
    {
        public double PositiveJerk { get; set; }
        public double NegativeJerk { get; set; }
        public double MaximumAcceleration { get; set; }
        public double MaximumDeceleration { get; set; }

        public MotionParameter(double positiveJerk, double negativeJerk, double maximumAcceleration, double maximumDecceleration)
        {
            PositiveJerk = positiveJerk;
            NegativeJerk = negativeJerk;
            MaximumAcceleration = maximumAcceleration;
            MaximumDeceleration = maximumDecceleration;
        }

        public MotionParameter WithDifferentAcceleration(double acceleration)
            => new MotionParameter(PositiveJerk, NegativeJerk, acceleration, MaximumDeceleration);

        public MotionParameter WithDifferentDeceleration(double deceleration)
            => new MotionParameter(PositiveJerk, NegativeJerk, MaximumAcceleration, deceleration);

        public MotionParameter WithDifferentPositiveJerk(double jerk)
            => new MotionParameter(jerk, NegativeJerk, MaximumAcceleration, MaximumDeceleration);

        public MotionParameter WithDifferentNegativeJerk(double jerk)
            => new MotionParameter(PositiveJerk, jerk, MaximumAcceleration, MaximumDeceleration);
    }
}
