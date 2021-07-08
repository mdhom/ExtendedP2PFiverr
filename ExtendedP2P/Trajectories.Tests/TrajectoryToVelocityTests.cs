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
        private readonly MotionParameter _motionParameter = new MotionParameter(100, -100, 100, -100);

        [Fact]
        public void AccelerateFromZero()
        {
            const double vTarget = 200.0;
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
            const double vStart = 50.0;
            const double vTarget = 200.0;
            Trajectory trajectory = TrajectoryToVelocity.Calculate(0, vStart, vTarget, _motionParameter);
            trajectory.CalculateStatus(trajectory.TotalDuration, out double j, out double a, out double v, out double s);

            Assert.Equal(_motionParameter.NegativeJerk, j, 1);
            Assert.Equal(0.0, a, 1);
            Assert.Equal(vTarget, v, 1);
            Assert.Equal(312.5, s, 1);
        }

        [Fact]
        public void Calculation()
        {
            const double a0 = 100;
            const double v0 = 100;
            
            double ta0 = TrajectoryToVelocity.t_at_a_zero(_motionParameter, a0);
            Assert.Equal(1.0, ta0, 1);

            double va0 = TrajectoryToVelocity.v_at_a_zero(_motionParameter, a0, v0);
            Assert.Equal(150.0, va0, 1);

            double sa0 = TrajectoryToVelocity.s_at_a_zero(_motionParameter, a0, v0);
            Assert.Equal(133.333, sa0, 1);

            Trajectory trajectory = TrajectoryToVelocity.Calculate(a0, v0, 0, _motionParameter);
            trajectory.CalculateStatus(ta0, out _, out _, out double v, out double s);
            Assert.Equal(va0, v, 1);
            Assert.Equal(sa0, s, 1);
            
            
        }
    }
}
