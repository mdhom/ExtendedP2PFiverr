using System;
using Xunit;

namespace Trajectories.Tests
{
    public class IteratorTests
    {
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
    }
}
