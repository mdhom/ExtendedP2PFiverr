using System;

namespace Trajectories
{
    public class TrajectoryToVelocity
    {
        public double InitialAcceleration { get; }
        public double InitialVelocity { get; }
        public MotionParameter MotionParams { get; }

        public bool Inverted { get; private set; }

        private TrajectoryToVelocity(double initialAcceleration, double initialVelocity, MotionParameter motionParameter)
        {
            InitialAcceleration = initialAcceleration;
            InitialVelocity = initialVelocity;
            MotionParams = motionParameter;
        }

        public static Trajectory Calculate(double initialAcceleration, double initialVelocity, double targetVelocity, MotionParameter motionParameter)
        {
            return new TrajectoryToVelocity(initialAcceleration, initialVelocity, motionParameter).Calculate(targetVelocity);
        }

        public static Trajectory Calculate(double initialVelocity, double targetVelocity, MotionParameter motionParameter)
        {
            return Calculate(0, initialVelocity, targetVelocity, motionParameter);
        }

        public static double CalculateDistanceNeeded(double vFrom, double vTo, MotionParameter motionParameter)
        {
            return Calculate(vFrom, vTo, motionParameter).Length;
        }

        public static double CalculateDistanceNeeded(double aFrom, double vFrom, double vTo, MotionParameter motionParameter)
        {
            return Calculate(aFrom, vFrom, vTo, motionParameter).Length;
        }

        public static double CalculateTimeNeeded(double vFrom, double vTo, MotionParameter motionParameter)
        {
            return Calculate(vFrom, vTo, motionParameter).TotalDuration;
        }

        public static double CalculateTimeNeeded(double aFrom, double vFrom, double vTo, MotionParameter motionParameter)
        {
            return Calculate(aFrom, vFrom, vTo, motionParameter).TotalDuration;
        }

        public static bool IsReachable(double initialVelocity, double targetVelocity, double distanceAvailable, MotionParameter motionParameter)
        {
            return Calculate(initialVelocity, targetVelocity, motionParameter).IsReachable(distanceAvailable);
        }


        #region Calculation

        private RampDirection GetDirection(double targetVelocity)
        {
            if (Math.Abs(InitialVelocity - targetVelocity) < 1e-8)
            {
                return RampDirection.Constant;
            }
            else if (targetVelocity < InitialVelocity)
            {
                return RampDirection.Decelerate;
            }
            else
            {
                return RampDirection.Accelerate;
            }
        }

        private Trajectory Calculate(double targetVelocity)
        {
            Trajectory result = new Trajectory {
                Parameters = MotionParams,
                aFrom = InitialAcceleration,
                vFrom = InitialVelocity,
                vTo = targetVelocity,
                Direction = GetDirection(targetVelocity)
            };

            if (result.Direction == RampDirection.Constant)
            {
                return result;
            }

            double decMax = MotionParams.MaximumDeceleration;
            double jPos = MotionParams.PositiveJerk;
            double jNeg = MotionParams.NegativeJerk;
            if (result.Direction == RampDirection.Accelerate)
            {
                decMax = MotionParams.MaximumAcceleration;
                jPos = MotionParams.NegativeJerk;
                jNeg = MotionParams.PositiveJerk;
            }

            double a0 = InitialAcceleration;
            double v0 = InitialVelocity;
            double t1 = (decMax - a0) / jNeg;
            double t2 = 0.0;
            double t3 = -(decMax / jPos);

            // does profile reach constant a?
            double v_bya0_Ph1 = t1 * a0;
            double v_byjD_Ph1 = 0.5 * jNeg * t1 * t1;
            double v_bya1_Ph3 = t3 * (a0 + jNeg * t1);
            double v_byjA_Ph3 = 0.5 * jPos * t3 * t3;
            double vTotal = v0 + v_bya0_Ph1 + v_byjD_Ph1 + v_bya1_Ph3 + v_byjA_Ph3;

            if (CheckForFlatRampPart(vTotal, targetVelocity, result.Direction))
            {
                // constant a will be reached
                t1 = (decMax - a0) / jNeg;
                t3 = Math.Abs(decMax / jPos);
                double v_Decc = 0.5 * jNeg * (t1 * t1) + a0 * t1;
                double v_Acc = -0.5 * jPos * (t3 * t3);
                t2 = (targetVelocity - (v_Decc + v_Acc + v0)) / decMax;
            }
            else
            {
                // constant a will not be reached
                // => calculate max reachable a
                double jerk;
                double t;
                if (a0 < 0)
                {
                    jerk = MotionParams.PositiveJerk;
                    t = -a0 / jerk;
                }
                else if (a0 > 0)
                {
                    jerk = MotionParams.NegativeJerk;
                    t = -a0 / jerk;
                }
                else
                {
                    t = 0;
                    jerk = 0;
                }

                //3. Bestimmung ob trotz Bremsen, beschleunigt werden muss und umgekehrt!
                // Geschwindigkeit die erreicht werden würde (v_bya0_to0), falls a0 auf 0 gezogen wird. Dies ist das aussschlagebende Kriterium, ob beschleunigt oder gebremst werden muss
                // Bsp.: v0 =200, a0= -112 , vTarget = 190  Nur durch den Abbau von a0 auf 0 wird eine Geschwindigkeit von ca. 187 erreicht --> es muss mathematisch beschleunigt werden, um die Zieglgeschwindigkeit zu erreichen
                double v_bya0_to0 = v0 + t * a0 + 0.5 * jerk * t * t;
                if (v_bya0_to0 > targetVelocity)
                {
                    Inverted = result.Direction == RampDirection.Accelerate;
                }
                else
                {
                    Inverted = result.Direction != RampDirection.Accelerate;
                }

                if (Inverted)  // Beschleuningen falls wir bremsen ---> Bremsen falls beschleunigen
                {
                    double tmp = jNeg;
                    jNeg = jPos;
                    jPos = tmp;
                }

                // 4. Gleichungssystem, um t1,t2 und t3 zu bestimmen
                double a = 0.5 * jNeg - 0.5 * (jNeg * jNeg / jPos);
                double b = a0 - a0 * (jNeg / jPos);
                double c = v0 - targetVelocity - a0 * a0 / (2 * jPos);

                if (MathematicTools.SolveEquation(a, b, c, out double x1, out double x2))
                {
                    CalculateAllTimes(x1, x2, jPos, jNeg, a0, out t1, out t2, out t3);
                }
            }

            double a1 = a0 + jNeg * t1;
            double v1 = v0 + a0 * t1 + 0.5 * jNeg * t1 * t1;
            double s1 = v0 * t1 + 0.5 * a0 * t1 * t1 + 1.0 / 6.0 * jNeg * t1 * t1 * t1;

            double a2 = a1;
            double v2 = v1 + a1 * t2;
            double s2 = v1 * t2 + 0.5 * a1 * t2 * t2;

            double s3 = v2 * t3 + 0.5 * a2 * t3 * t3 + 1.0 / 6.0 * jPos * t3 * t3 * t3;

            result.Length = s1 + s2 + s3;
            result.TotalDuration = t1 + t2 + t3;
            result.Phase1Duration = t1;
            result.Phase1Length = s1;
            result.Phase2Duration = t2;
            result.Phase2Length = s2;
            result.Phase3Duration = t3;
            result.Phase3Length = s3;

            return result;
        }

        private static void CalculateAllTimes(double x1, double x2, double jA, double jD, double a0, out double t1, out double t2, out double t3)
        {
            t1 = 0;
            t2 = 0;
            t3 = 0;
            if (x1 >= 0 && x2 >= 0)
            {
                double t1_1 = x1;
                double t1_2 = x2;
                t2 = 0;
                double t3_1 = (-jD * t1_1 - a0) / jA;
                double t3_2 = (-jD * t1_2 - a0) / jA;
                t1 = t3_1 > 0 ? t1_1 : t1_2;
                t3 = t3_1 > 0 ? t3_1 : t3_2;
            }
            else if (x1 >= 0 || x2 >= 0)
            {
                t1 = x1 > 0 ? x1 : x2;
                t2 = 0;
                t3 = (-jD * t1 - a0) / jA;
            }
        }

        private static bool CheckForFlatRampPart(double vTotal, double vTarget, RampDirection direction)
        {
            if (direction == RampDirection.Decelerate)
            {
                return vTotal > vTarget;
            }
            else
            {
                return vTotal <= vTarget;
            }
        }

        #endregion
    }
}
