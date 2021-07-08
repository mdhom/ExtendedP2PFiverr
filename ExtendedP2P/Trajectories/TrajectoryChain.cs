using System;
using System.Collections.Generic;
using System.Text;

namespace Trajectories
{
    public class TrajectoryChain
    {
        private readonly Trajectory[] _trajectories;

        public double Distance { get; }

        public double TotalDuration { get; }

        public TrajectoryChain(Trajectory[] trajectories)
        {
            _trajectories = trajectories;

            foreach (Trajectory ramp in _trajectories)
            {
                Distance += ramp.Length;
                TotalDuration += ramp.TotalDuration;
            }
        }

        public void GetStatus(double t, out double j, out double a, out double v, out double s)
        {
            if (_trajectories == null)
            {
                j = double.NaN;
                a = double.NaN;
                v = double.NaN;
                s = double.NaN;
                return;
            }

            double tSum = 0.0;
            double sSum = 0.0;
            for (int i = 0; i < _trajectories.Length; i++)
            {
                if (tSum + _trajectories[i].TotalDuration >= t)
                {
                    _trajectories[i].CalculateStatus(t - tSum, out j, out a, out v, out double sRamp);
                    s = sRamp + sSum;
                    return;
                }

                tSum += _trajectories[i].TotalDuration;
                sSum += _trajectories[i].Length;
            }

            GetStatus(tSum, out j, out a, out v, out s);
        }

        public void Validate(double timeStep = 0.001)
        {
            var vLast = _trajectories[0].vFrom;
            var sLast = 0.0;
            for (double t = 0; t <= TotalDuration + 0.1; t += timeStep)
            {
                GetStatus(t, out var j, out var a, out var v, out var s);

                Trajectory.ValidateContinuity(vLast, sLast, a, v, s, timeStep);

                vLast = v;
                sLast = s;
            }
        }
    }
}
