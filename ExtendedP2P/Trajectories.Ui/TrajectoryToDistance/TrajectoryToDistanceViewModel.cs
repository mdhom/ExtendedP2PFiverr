using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;

namespace Trajectories.Ui
{
    public class TrajectoryToDistanceViewModel : Servus.Core.NotifyPropertyChangedBase
    {
        private double _targetDistance = 15000;
        public double TargetDistance
        {
            get => _targetDistance;
            set => ChangeProperty(value, ref _targetDistance, Update);
        }

        private double _velocityMax = 1500;
        public double VelocityMax
        {
            get => _velocityMax;
            set => ChangeProperty(value, ref _velocityMax, Update);
        }

        private double _jerkMax = 2000;
        public double JerkMax
        {
            get => _jerkMax;
            set => ChangeProperty(value, ref _jerkMax, Update);
        }

        private double _accelerationMax = 500;
        public double AccelerationMax
        {
            get => _accelerationMax;
            set => ChangeProperty(value, ref _accelerationMax, Update);
        }

        private double _acceleration0 = 500;
        public double Acceleration0
        {
            get => _acceleration0;
            set => ChangeProperty(value, ref _acceleration0, Update);
        }

        private double _velocity0 = 500;
        public double Velocity0
        {
            get => _velocity0;
            set => ChangeProperty(value, ref _velocity0, Update);
        }

        public TrajectoryToDistanceCalculationStatus Status { get; private set; }
        public string CalculationTime { get; private set; }
        public double ResultDuration { get; private set; }
        public double ResultDistance { get; private set; }
        public double ResultDistanceDifference { get; private set; }
        public double ResultMaxReachedVelocity { get; private set; }
        public int ResultTrajectoryInstanceCase { get; private set; }

        public List<DataPoint> DataJ { get; private set; }
        public List<DataPoint> DataA { get; private set; }
        public List<DataPoint> DataV { get; private set; }
        public List<DataPoint> DataS { get; private set; }
        public List<DataPoint> DataBrakingDistance { get; private set; }

        public PlotModel ModelJerk { get; }
        public PlotModel ModelAcceleration { get; }
        public PlotModel ModelVelocity { get; }
        public PlotModel ModelDistance { get; }

        public TrajectoryToDistanceViewModel()
        {
            ModelJerk = new PlotModel();
            LinearAxis axisJerk = new LinearAxis() { Position = AxisPosition.Left, Title="Jerk" };
            ModelJerk.Axes.Add(axisJerk);

            ModelAcceleration = new PlotModel();
            LinearAxis axisAcceleration = new LinearAxis() { Position = AxisPosition.Left, Title = "Acceleration" };
            ModelAcceleration.Axes.Add(axisAcceleration);

            ModelVelocity = new PlotModel();
            LinearAxis axisVelocity = new LinearAxis() { Position = AxisPosition.Left, Title = "Velocity" };
            ModelVelocity.Axes.Add(axisVelocity);

            ModelDistance = new PlotModel();
            LinearAxis axisDistance = new LinearAxis() { Position = AxisPosition.Left, Title = "Distance" };
            ModelDistance.Axes.Add(axisDistance);

            Update();
        }

        private void Update()
        {
            DataJ = new List<DataPoint>();
            DataA = new List<DataPoint>();
            DataV = new List<DataPoint>();
            DataS = new List<DataPoint>();
            DataBrakingDistance = new List<DataPoint>();
            ResultDuration = 0;
            ResultTrajectoryInstanceCase = 0;
            ResultDistance = 0;
            ResultMaxReachedVelocity = 0;

            try
            {
                MotionParameter motionParameter = new MotionParameter(JerkMax, -JerkMax, AccelerationMax, -AccelerationMax);
                DateTime started = DateTime.Now;
                TrajectoryToDistanceResult result = TrajectoryToDistance.Calculate(TargetDistance, VelocityMax, Velocity0, Acceleration0, motionParameter);
                DateTime completed = DateTime.Now;
                CalculationTime = $"{(completed - started).TotalMilliseconds:0.000} ms";
                Status = result.CalculationStatus;
                if (Status == TrajectoryToDistanceCalculationStatus.Ok)
                {
                    ResultDistance = result.TrajectoryChain.Distance;
                    ResultDistanceDifference = TargetDistance - ResultDistance;
                    ResultDuration = result.TrajectoryChain.Duration;
                    for (double t = 0; t <= ResultDuration + 0.002; t += 0.001)
                    {
                        result.TrajectoryChain.GetStatus(t, out double j, out double a, out double v, out double s);
                        DataJ.Add(new DataPoint(t, j));
                        DataA.Add(new DataPoint(t, a));
                        DataV.Add(new DataPoint(t, v));
                        DataS.Add(new DataPoint(t, s));
                        //DataBrakingDistance.Add(new DataPoint(t, calc.GetBrakingDistance(t)));
                    }

                    //ResultTrajectoryInstanceCase = calc.TrajectoryInstanceCase;
                    //ResultMaxReachedVelocity = calc.CalculateMaximumReachedVelocity();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Exception: {ex.Message}\r\n{ex.StackTrace}");
            }

            ModelJerk.Series.Clear();
            ModelJerk.Series.Add(new LineSeries() { ItemsSource = DataJ, Color = OxyColors.Red });
            ModelJerk.InvalidatePlot(true);

            ModelAcceleration.Series.Clear();
            ModelAcceleration.Series.Add(new LineSeries() { ItemsSource = DataA, Color = OxyColors.Green });
            ModelAcceleration.InvalidatePlot(true);

            ModelVelocity.Series.Clear();
            ModelVelocity.Series.Add(new LineSeries() { ItemsSource = DataV, Color = OxyColors.Blue });
            ModelVelocity.InvalidatePlot(true);

            ModelDistance.Series.Clear();
            ModelDistance.Series.Add(new LineSeries() { ItemsSource = DataS, Color = OxyColors.Orange });
            ModelDistance.InvalidatePlot(true);

            OnPropertyChanged(nameof(DataJ));
            OnPropertyChanged(nameof(DataA));
            OnPropertyChanged(nameof(DataV));
            OnPropertyChanged(nameof(DataS));
            OnPropertyChanged(nameof(DataBrakingDistance));
            OnPropertyChanged(nameof(CalculationTime));
            OnPropertyChanged(nameof(ResultDuration));
            OnPropertyChanged(nameof(ResultTrajectoryInstanceCase));
            OnPropertyChanged(nameof(ResultDistance));
            OnPropertyChanged(nameof(ResultDistanceDifference));
            OnPropertyChanged(nameof(ResultMaxReachedVelocity));
            OnPropertyChanged(nameof(Status));
        }
    }
}
