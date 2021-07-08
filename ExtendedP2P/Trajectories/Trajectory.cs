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
        
        public bool IsOvershootingAtStart { get; set; } 
        
        public double Phase0Duration { get; set; }
        public double Phase0Length { get; set; }
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
            Phase0Duration = result.Phase0Duration;
            Phase1Duration = result.Phase1Duration;
            Phase2Duration = result.Phase2Duration;
            Phase3Duration = result.Phase3Duration;
            Phase0Length = result.Phase0Length;
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

            double t0 = Phase0Duration;
            double t1 = t0 + Phase1Duration;
            double t2 = t1 + Phase2Duration;
            double t3 = t2 + Phase3Duration;

            double jMax = 0;
            switch (Direction)
            {
                case RampDirection.Accelerate when IsOvershootingAtStart:
                    jMax = Parameters.NegativeJerk;
                    break;
                case RampDirection.Accelerate:
                    jMax = Parameters.PositiveJerk;
                    break;
                case RampDirection.Decelerate when IsOvershootingAtStart:
                    jMax = Parameters.PositiveJerk;
                    break;
                case RampDirection.Decelerate:
                    jMax = Parameters.NegativeJerk;
                    break;
            }
            
            if (t <= t0)
            {
                GetStatus0(this, jMax, aFrom, vFrom, t, out j, out a, out v, out s);
            }
            else
            {
                GetStatus0(this, jMax, aFrom, vFrom, t0, out double j0, out double a0, out double v0, out double s0);
                
                if (t <= t1)
                {
                    GetStatus1(this, jMax, a0, v0, s0, t0, t, out j, out a, out v, out s);
                }
                else
                {
                    GetStatus1(this, jMax, a0, v0, s0, t0, t1, out _, out double a1, out double v1, out double s1);

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
        }

        public bool IsReachable(double availableDistance)
        {
            return Length <= availableDistance;
        }

        private static void GetStatus0(Trajectory ramp, double jMax, double a0, double v0, double tIn, out double jOut,
            out double aOut, out double vOut, out double sOut)
        {
            double tPhase = tIn;
            double tPhase2 = tPhase * tPhase;
            double tPhase3 = tPhase2 * tPhase;
            
            jOut = a0 > 0 ? ramp.Parameters.NegativeJerk : ramp.Parameters.PositiveJerk;
            aOut = a0 + jOut * tPhase;
            vOut = v0 + a0 * tPhase + 0.5 * jOut * tPhase2;
            sOut = v0 * tPhase + 0.5 * a0 * tPhase2 + jOut * tPhase3 / 6.0;
        }
        
        private static void GetStatus1(Trajectory ramp, double jMax, double a0, double v0, double s0, double t0, double tIn, out double jOut, out double aOut, out double vOut, out double sOut)
        {
            double tPhase = tIn - t0;
            double tPhase2 = tPhase * tPhase;
            double tPhase3 = tPhase2 * tPhase;

            jOut = jMax;
            aOut = jMax * tPhase + a0;
            vOut = v0 + 0.5 * jMax * tPhase2 + a0 * tPhase;
            sOut = s0 + v0 * tPhase + jMax / 6 * tPhase3 + 0.5 * a0 * tPhase2;
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
            double vLast = vFrom;
            double sLast = 0.0;
            
            // validate first status
            CalculateStatus(0, out double j, out double a, out double v, out double s);
            if (Math.Abs(a - aFrom) > 0.1)
                throw new Exception($"Initial acceleration differs: a(0)={a}, aFrom={aFrom}");
            if (Math.Abs(v - vFrom) > 0.1)
                throw new Exception($"Initial velocity differs: v(0)={v}, vFrom={vFrom}");
            
            // validate last status
            CalculateStatus(TotalDuration, out j, out a, out v, out s);
            if (Math.Abs(v - vTo) > 0.1)
                throw new Exception($"Target velocity differs: v(total)={v}, vTo={vTo}");
            
            // validate continuity
            for (double t = 0; t <= TotalDuration + 0.1; t += timeStep)
            {
                CalculateStatus(t, out j, out a, out v, out s);

                ValidateContinuity(vLast, sLast, a, v, s, timeStep);

                vLast = v;
                sLast = s;
            }
        }

        public static void ValidateContinuity(double v0, double s0, double a1, double v1, double s1, double timeStep)
        {
            double deltaS = s1 - s0;
            double thresholdS = Math.Abs(1.5 * timeStep * v1);
            if (deltaS > 0.1 && Math.Abs(deltaS) > thresholdS)
            {
                throw new Exception($"Discontinuity in Distance: deltaS={deltaS}, thresholdS={thresholdS}, s={s1}, s0={s0}, v1={v1}");
            }
            
            double deltaV = v1 - v0;
            double thresholdV = Math.Abs(1.5 * timeStep * a1);
            if (deltaV > 0.1 && Math.Abs(deltaV) > thresholdV)
            {
                throw new Exception($"Discontinuity in Velocity: deltaV={deltaV}, thresholdV={thresholdV}, v={v1}, v0={v0}, a1={a1}");
            }
        }
    }
}
