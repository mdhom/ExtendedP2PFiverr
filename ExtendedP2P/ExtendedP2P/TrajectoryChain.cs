using System;
using System.Collections.Generic;
using System.Text;

namespace ExtendedP2P
{
    public class TrajectoryChain
    {
        private readonly Trajectory[] _trajectories;

        public double Distance { get; }

        public double Duration { get; }

        public TrajectoryChain(Trajectory[] trajectories)
        {
            _trajectories = trajectories;

            foreach (Trajectory ramp in _trajectories)
            {
                Distance += ramp.Length;
                Duration += ramp.TotalDuration;
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
    }
}
