using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xunit;

namespace Trajectories.Tests
{
    public class TrajectoryToVelocityTests
    {
        private MotionParameter _motionParameter = new MotionParameter(100, -100, 100, -100);

        [Fact]
        public void AccelerateFromZero()
        {
            double vTarget = 200.0;
            Trajectory trajectory = TrajectoryToVelocity.Calculate(0, 0, vTarget, _motionParameter);
            trajectory.CalculateStatus(trajectory.TotalDuration, out double j, out double a, out double v, out double s);

            Assert.Equal(_motionParameter.NegativeJerk, j, 1);
            Assert.Equal(0.0, a, 1);
            Assert.Equal(vTarget, v, 1);
            Assert.Equal(300.0, s, 1);
        }

        [Fact]
        public void AccelerateFromVelocity()
        {
            double vStart = 50.0;
            double vTarget = 200.0;
            Trajectory trajectory = TrajectoryToVelocity.Calculate(0, vStart, vTarget, _motionParameter);
            trajectory.CalculateStatus(trajectory.TotalDuration, out double j, out double a, out double v, out double s);

            Assert.Equal(_motionParameter.NegativeJerk, j, 1);
            Assert.Equal(0.0, a, 1);
            Assert.Equal(vTarget, v, 1);
            Assert.Equal(312.5, s, 1);
        }
    }
}
