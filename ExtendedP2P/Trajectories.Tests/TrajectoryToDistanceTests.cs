using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xunit;

namespace Trajectories.Tests
{
    public class TrajectoryToDistanceTests
    {
        [Fact]
        public void Find()
        {
            MotionParameter motionParameter = new MotionParameter(100, -100, 100, -100);
            TrajectoryToDistanceResult ramps = TrajectoryToDistance.Calculate(2000, 300, 100, 200, motionParameter);
            ramps = TrajectoryToDistance.Calculate(2000, 700, 100, 200, motionParameter);
        }
    }
}
