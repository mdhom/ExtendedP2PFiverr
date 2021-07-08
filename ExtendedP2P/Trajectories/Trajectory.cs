using System;

namespace Trajectories
{
    public class Trajectory
    {
        public MotionParameter Parameters { get; set; }
        public RampDirection Direction { get; set; }

        public double aFrom { get; set; }
        public double vFrom { get; set; }
        public double vTo { get; set; }

        public double Phase1Duration { get; set; }
        public double Phase1Length { get; set; }
        public double Phase2Duration { get; set; }
        public double Phase2Length { get; set; }
        public double Phase3Duration { get; set; }
        public double Phase3Length { get; set; }

        public double Length { get; set; }
        public double TotalDuration { get; set; }

        public Trajectory()
        {
        }

        public Trajectory(Trajectory result)
        {
            Parameters = result.Parameters;
            Direction = result.Direction;
            aFrom = result.aFrom;
            vFrom = result.vFrom;
            vTo = result.vTo;
            Phase1Duration = result.Phase1Duration;
            Phase2Duration = result.Phase2Duration;
            Phase3Duration = result.Phase3Duration;
            Phase1Length = result.Phase1Length;
            Phase2Length = result.Phase2Length;
            Phase3Length = result.Phase3Length;
            Length = result.Length;
            TotalDuration = result.TotalDuration;
        }

        public void CalculateStatus(double t, out double j, out double a, out double v, out double s)
        {
            if (Direction == RampDirection.Constant)
            {
                j = 0.0;
                a = 0.0;
                v = vFrom;
                s = t * vFrom;
                return;
            }

            double t1 = Phase1Duration;
            double t2 = t1 + Phase2Duration;
            double t3 = t2 + Phase3Duration;

            double jMax = Direction == RampDirection.Accelerate ? Parameters.PositiveJerk : Parameters.NegativeJerk;
            double v0 = vFrom;

            if (t <= t1)
            {
                GetStatus1(this, jMax, v0, t, out j, out a, out v, out s);
            }
            else
            {
                GetStatus1(this, jMax, v0, t1, out _, out double a1, out double v1, out double s1);

                if (t <= t2)
                {
                    GetStatus2(t1, t, a1, v1, s1, out j, out a, out v, out s);
                }
                else
                {
                    GetStatus2(t1, t2, a1, v1, s1, out _, out double a2, out double v2, out double s2);

                    if (t <= t3)
                    {
                        GetStatus3(jMax, t2, t, a2, v2, s2, out j, out a, out v, out s);
                    }
                    else
                    {
                        GetStatus3(jMax, t2, t3, a2, v2, s2, out j, out a, out v, out s);
                    }
                }
            }
        }

        public bool IsReachable(double availableDistance)
        {
            return Length <= availableDistance;
        }

        private static void GetStatus1(Trajectory ramp, double jMax, double v0, double tIn, out double jOut, out double aOut, out double vOut, out double sOut)
        {
            double tPhase = tIn;
            double tPhase2 = tPhase * tPhase;
            double tPhase3 = tPhase2 * tPhase;

            jOut = jMax;
            aOut = jMax * tPhase + ramp.aFrom;
            vOut = v0 + 0.5 * jMax * tPhase2 + ramp.aFrom * tPhase;
            sOut = v0 * tPhase + jMax / 6 * tPhase3 + 0.5 * ramp.aFrom * tPhase2;
        }

        private static void GetStatus2(double t1, double tIn, double a1, double v1, double s1, out double jOut, out double aOut, out double vOut, out double sOut)
        {
            double tPhase = tIn - t1;
            double tPhase2 = tPhase * tPhase;

            jOut = 0;
            aOut = a1;
            vOut = v1 + a1 * tPhase;
            sOut = s1 + v1 * tPhase + 0.5 * a1 * tPhase2;
        }

        private static void GetStatus3(double jMax, double t2, double tIn, double a2, double v2, double s2, out double jOut, out double aOut, out double vOut, out double sOut)
        {
            double tPhase = tIn - t2;
            double tPhase2 = tPhase * tPhase;
            double tPhase3 = tPhase2 * tPhase;

            jOut = -jMax;
            aOut = a2 - jMax * tPhase;
            vOut = v2 + a2 * tPhase + 0.5 * -jMax * tPhase2;
            sOut = s2 + v2 * tPhase + 0.5 * a2 * tPhase2 + -jMax / 6 * tPhase3;
        }

        public void Validate(double timeStep=0.001)
        {
            var vLast = vFrom;
            var sLast = 0.0;
            for (double t = 0; t <= TotalDuration + 0.1; t += timeStep)
            {
                CalculateStatus(t, out var j, out var a, out var v, out var s);

                ValidateContinuity(vLast, sLast, a, v, s, timeStep);

                vLast = v;
                sLast = s;
            }
        }

        public static void ValidateContinuity(double v0, double s0, double a1, double v1, double s1, double timeStep)
        {
            var deltaV = v1 - v0;
            var deltaS = s1 - s0;

            if (deltaS > 0.1 && Math.Abs(deltaS) > Math.Abs(1.5 * timeStep * v1))
            {
                throw new Exception($"Discontinuity in Distance: deltaS={deltaS}, s={s1}, s0={s0}, v1={v1}");
            }
            if (deltaV > 0.1 && Math.Abs(deltaV) > Math.Abs(1.5 * timeStep * a1))
            {
                throw new Exception($"Discontinuity in Velocity: deltaV={deltaV}, v={v1}, v0={v0}, a1={a1}");
            }
        }
    }
}
