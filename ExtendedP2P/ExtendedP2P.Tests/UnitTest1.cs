using System;
using Xunit;

namespace ExtendedP2P.Tests
{
    public class UnitTest1
    {
        [Fact]
        public void Test1()
        {
            Trajectory trajectory = TrajectoryToVelocity.Calculate(100, 0, 200, new MotionParameter(100, -100, 100, -100));
            trajectory.CalculateStatus(trajectory.TotalDuration, out double j, out double a, out double v, out double s);
        }

        [Theory]
        [InlineData(63, 62.5)]
        [InlineData(50, 50)]
        [InlineData(40, 40.625)]
        public void Iterate(double target, double expectedResult)
        {
            double result = Iterator.IterateFromMax(0, 100, (val) =>
            {
                return new IteratorStepResult<double>(val > target, Math.Abs(val - target) < 2, val);
            });
            Assert.Equal(expectedResult, result, 3);
        }

        [Fact]
        public void Find()
        {
            MotionParameter motionParameter = new MotionParameter(100, -100, 100, -100);
            TrajectoryToDistanceResult ramps = TrajectoryToDistance.Calculate(2000, 300, 100, 200, motionParameter);
            ramps = TrajectoryToDistance.Calculate(2000, 700, 100, 200, motionParameter);
        }
    }
}
