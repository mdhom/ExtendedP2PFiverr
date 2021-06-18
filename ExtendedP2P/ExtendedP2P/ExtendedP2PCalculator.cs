﻿using System;
using System.Collections.Generic;
using System.Numerics;

namespace ExtendedP2P
{
    internal class ExtendedP2PCalculator
    {
        public double InitialAcceleration { get; }
        public double InitialVelocity { get; }
        public MotionParameter MotionParameter { get; }

        public bool Inverted { get; private set; }

        private ExtendedP2PCalculator(double initialAcceleration, double initialVelocity, MotionParameter motionParameter)
        {
            InitialAcceleration = initialAcceleration;
            InitialVelocity = initialVelocity;
            MotionParameter = motionParameter;
        }

        public static ExtendedP2PCalculatorResult Calculate(double initialAcceleration, double initialVelocity, double targetVelocity, MotionParameter motionParameter)
            => new ExtendedP2PCalculator(initialAcceleration, initialVelocity, motionParameter).Calculate(targetVelocity);

        public static ExtendedP2PCalculatorResult Calculate(double initialVelocity, double targetVelocity, MotionParameter motionParameter)
            => Calculate(0, initialVelocity, targetVelocity, motionParameter);

        public static double CalculateDistanceNeeded(double vFrom, double vTo, MotionParameter motionParameter)
            => Calculate(vFrom, vTo, motionParameter).Length;

        public static double CalculateDistanceNeeded(double aFrom, double vFrom, double vTo, MotionParameter motionParameter)
            => Calculate(aFrom, vFrom, vTo, motionParameter).Length;

        public static double CalculateTimeNeeded(double vFrom, double vTo, MotionParameter motionParameter)
            => Calculate(vFrom, vTo, motionParameter).TotalDuration;

        public static double CalculateTimeNeeded(double aFrom, double vFrom, double vTo, MotionParameter motionParameter)
            => Calculate(aFrom, vFrom, vTo, motionParameter).TotalDuration;

        public static bool IsReachable(double initialVelocity, double targetVelocity, double distanceAvailable, MotionParameter motionParameter)
            => Calculate(initialVelocity, targetVelocity, motionParameter).IsReachable(distanceAvailable);

        public static void CalculateStatus(ExtendedP2PCalculatorResult ramp, double t, out double j, out double a, out double v, out double s)
        {
            var t1 = ramp.Phase1Duration;
            var t2 = t1 + ramp.Phase2Duration;
            var t3 = t2 + ramp.Phase3Duration;

            var jMax = ramp.Direction == RampDirection.Accelerate ? ramp.Parameters.PositiveJerk : ramp.Parameters.NegativeJerk;
            var v0 = ramp.vFrom;

            if (t <= t1)
            {
                GetStatus1(ramp, jMax, v0, t, out j, out a, out v, out s);
            }
            else
            {
                GetStatus1(ramp, jMax, v0, t1, out _, out var a1, out var v1, out var s1);

                if (t <= t2)
                {
                    GetStatus2(t1, t, a1, v1, s1, out j, out a, out v, out s);
                }
                else
                {
                    GetStatus2(t1, t2, a1, v1, s1, out _, out var a2, out var v2, out var s2);

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

        private static void GetStatus1(ExtendedP2PCalculatorResult ramp, double jMax, double v0, double tIn, out double jOut, out double aOut, out double vOut, out double sOut)
        {
            var tPhase = tIn;
            var tPhase2 = tPhase * tPhase;
            var tPhase3 = tPhase2 * tPhase;

            jOut = jMax;
            aOut = jMax * tPhase + ramp.aFrom;
            vOut = v0 + 0.5 * jMax * tPhase2 + ramp.aFrom * tPhase;
            sOut = v0 * tPhase + jMax / 6 * tPhase3 + 0.5 * ramp.aFrom * tPhase2;
        }

        private static void GetStatus2(double t1, double tIn, double a1, double v1, double s1, out double jOut, out double aOut, out double vOut, out double sOut)
        {
            var tPhase = tIn - t1;
            var tPhase2 = tPhase * tPhase;

            jOut = 0;
            aOut = a1;
            vOut = v1 + a1 * tPhase;
            sOut = s1 + v1 * tPhase + 0.5 * a1 * tPhase2;
        }

        private static void GetStatus3(double jMax, double t2, double tIn, double a2, double v2, double s2, out double jOut, out double aOut, out double vOut, out double sOut)
        {
            var tPhase = tIn - t2;
            var tPhase2 = tPhase * tPhase;
            var tPhase3 = tPhase2 * tPhase;

            jOut = -jMax;
            aOut = a2 - jMax * tPhase;
            vOut = v2 + a2 * tPhase + 0.5 * -jMax * tPhase2;
            sOut = s2 + v2 * tPhase + 0.5 * a2 * tPhase2 + -jMax / 6 * tPhase3;
        }

        private static List<Complex> SolveCubic(double a, double b, double c, double d)
        {
            const int NRoots = 3;
            var SquareRootof3 = Math.Sqrt(3);
            // the 3 cubic roots of 1
            var CubicUnity = new List<Complex>(NRoots)
                        { new Complex(1, 0), new Complex(-0.5, -SquareRootof3 / 2.0), new Complex(-0.5, SquareRootof3 / 2.0) };
            // intermediate calculations
            var DELTA = 18 * a * b * c * d - 4 * b * b * b * d + b * b * c * c - 4 * a * c * c * c - 27 * a * a * d * d;
            var DELTA0 = b * b - 3 * a * c;
            var DELTA1 = 2 * b * b * b - 9 * a * b * c + 27 * a * a * d;
            Complex DELTA2 = -27 * a * a * DELTA;
            var C = Complex.Pow((DELTA1 + Complex.Pow(DELTA2, 0.5)) / 2, 1 / 3.0); //Phew...
            var R = new List<Complex>(NRoots);
            for (var i = 0; i < NRoots; i++)
            {
                var M = CubicUnity[i] * C;
                var Root = -1.0 / (3 * a) * (b + M + DELTA0 / M);
                R.Add(Root);
            }
            return R;
        }

        private static double SolveCubicEquation(double A, double B, double C, double D)
        {
            var p = (9 * A * C - 3 * B * B) / (9 * A * A);
            var q = (2 * B * B * B - 9 * A * B * C + 27 * A * A * D) / (27 * A * A * A);
            var diskriminante = (27 * A * A * D * D + 4 * B * B * B * D - 18 * A * B * C * D + 4 * A * C * C * C - B * B * C * C) / (108 * A * A * A * A);
            var d1Mats = double.NaN;
            if (true) // p < 0
            {
                var h1 = Math.Sqrt(-p * 4 / 3);
                var h2 = -q / 2 * Math.Sqrt(-27.0 / (p * p * p));
                if (Math.Abs(h2) > 1)
                {
                    // ERROR!
                }
                var h3 = 1.0 / 3 * Math.Acos(h2);
                var h4 = B / (3 * A);
                if (diskriminante < 0)
                {
                    var x2 = -h1 * Math.Cos(h3 + Math.PI / 3) - h4;
                    var x1 = h1 * Math.Cos(h3) - h4;
                    var x3 = -h1 * Math.Cos(h3 - Math.PI / 3) - h4;

                    if (Math.Abs(x1) < Math.Abs(x2) && Math.Abs(x1) < Math.Abs(x3))
                    {
                        d1Mats = x1;
                    }
                    else if (Math.Abs(x2) < Math.Abs(x1) && Math.Abs(x2) < Math.Abs(x3))
                    {
                        d1Mats = x2;
                    }
                    else
                    {
                        d1Mats = x3;
                    }
                }
                else if (diskriminante == 0 && p == 0)
                {
                    d1Mats = -B / (3 * A);
                }
                else if (diskriminante == 0 && p != 0)
                {
                    var x1 = (B * B * B - 4 * A * B * C + 9 * A * A * D) / (3 * A * A * C - A * B * B);
                    var x2 = (A * B * C - 9 * A * A * D) / (6 * A * A * C - 2 * A * B * B);
                    if (Math.Abs(x1) < Math.Abs(x2))
                    {
                        d1Mats = x1;
                    }
                    else
                    {
                        d1Mats = x2;
                    }
                }
                else if (diskriminante > 0)
                {
                    var u = -(q / 2) + Math.Sqrt(diskriminante);
                    if (u >= 0)
                    {
                        u = MathematicTools.GetCubeRoot(u);
                    }
                    else
                    {
                        u = -MathematicTools.GetCubeRoot(-u);
                    }
                    var v = -(q / 2) - Math.Sqrt(diskriminante);
                    if (v >= 0)
                    {
                        v = MathematicTools.GetCubeRoot(v);
                    }
                    else
                    {
                        v = -MathematicTools.GetCubeRoot(-v);
                    }

                    d1Mats = u + v - B / (3 * A);
                }
            }

            return d1Mats;
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

        private ExtendedP2PCalculatorResult Calculate(double targetVelocity)
        {
            var result = new ExtendedP2PCalculatorResult
            {
                Parameters = MotionParameter,
                aFrom = InitialAcceleration,
                vFrom = InitialVelocity,
                vTo = targetVelocity,
                Direction = GetDirection(targetVelocity)
            };

            if (result.Direction == RampDirection.Constant)
            {
                return result;
            }

            var decMax = MotionParameter.MaximumDecceleration;
            var jPos = MotionParameter.PositiveJerk;
            var jNeg = MotionParameter.NegativeJerk;
            if (result.Direction == RampDirection.Accelerate)
            {
                decMax = MotionParameter.MaximumAcceleration;
                jPos = MotionParameter.NegativeJerk;
                jNeg = MotionParameter.PositiveJerk;
            }

            var a0 = InitialAcceleration;
            var v0 = InitialVelocity;
            var t1 = (decMax - a0) / jNeg;
            var t2 = 0.0;
            var t3 = -(decMax / jPos);

            // does profile reach constant a?
            var v_bya0_Ph1 = t1 * a0;
            var v_byjD_Ph1 = 0.5 * jNeg * t1 * t1;
            var v_bya1_Ph3 = t3 * (a0 + jNeg * t1);
            var v_byjA_Ph3 = 0.5 * jPos * t3 * t3;
            var vTotal = v0 + v_bya0_Ph1 + v_byjD_Ph1 + v_bya1_Ph3 + v_byjA_Ph3;

            if (CheckForFlatRampPart(vTotal, targetVelocity, result.Direction))
            {
                // constant a will be reached
                t1 = (decMax - a0) / jNeg;
                t3 = Math.Abs(decMax / jPos);
                var v_Decc = 0.5 * jNeg * (t1 * t1) + a0 * t1;
                var v_Acc = -0.5 * jPos * (t3 * t3);
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
                    jerk = MotionParameter.PositiveJerk;
                    t = -a0 / jerk;
                }
                else if (a0 > 0)
                {
                    jerk = MotionParameter.NegativeJerk;
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
                var v_bya0_to0 = v0 + t * a0 + 0.5 * jerk * t * t;
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
                    var tmp = jNeg;
                    jNeg = jPos;
                    jPos = tmp;
                }

                // 4. Gleichungssystem, um t1,t2 und t3 zu bestimmen
                var a = 0.5 * jNeg - 0.5 * (jNeg * jNeg / jPos);
                var b = a0 - a0 * (jNeg / jPos);
                var c = v0 - targetVelocity - a0 * a0 / (2 * jPos);

                if (MathematicTools.SolveEquation(a, b, c, out var x1, out var x2))
                {
                    CalculateAllTimes(x1, x2, jPos, jNeg, a0, out t1, out t2, out t3);
                }
            }

            var a1 = a0 + jNeg * t1;
            var v1 = v0 + a0 * t1 + 0.5 * jNeg * t1 * t1;
            var s1 = v0 * t1 + 0.5 * a0 * t1 * t1 + 1.0 / 6.0 * jNeg * t1 * t1 * t1;

            var a2 = a1;
            var v2 = v1 + a1 * t2;
            var s2 = v1 * t2 + 0.5 * a1 * t2 * t2;

            var s3 = v2 * t3 + 0.5 * a2 * t3 * t3 + 1.0 / 6.0 * jPos * t3 * t3 * t3;

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
                var t1_1 = x1;
                var t1_2 = x2;
                t2 = 0;
                var t3_1 = (-jD * t1_1 - a0) / jA;
                var t3_2 = (-jD * t1_2 - a0) / jA;
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