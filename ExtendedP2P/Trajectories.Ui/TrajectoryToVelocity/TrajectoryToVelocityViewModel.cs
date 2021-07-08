using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Input;
using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;
using Servus.Core.Ui;

namespace Trajectories.Ui
{
    public class TrajectoryToVelocityViewModel : Servus.Core.NotifyPropertyChangedBase
    {
        private double _velocityMax = 200;
        public double VelocityMax
        {
            get => _velocityMax;
            set => ChangeProperty(value, ref _velocityMax, UpdateFromProperty);
        }

        private double _jerkMax = 100;
        public double JerkMax
        {
            get => _jerkMax;
            set => ChangeProperty(value, ref _jerkMax, UpdateFromProperty);
        }

        private double _accelerationMax = 100;
        public double AccelerationMax
        {
            get => _accelerationMax;
            set => ChangeProperty(value, ref _accelerationMax, UpdateFromProperty);
        }

        private double _acceleration0 = 0;
        public double Acceleration0
        {
            get => _acceleration0;
            set => ChangeProperty(value, ref _acceleration0, UpdateFromProperty);
        }

        private double _velocity0 = 0;
        public double Velocity0
        {
            get => _velocity0;
            set => ChangeProperty(value, ref _velocity0, UpdateFromProperty);
        }

        public string CalculationTime { get; private set; }
        public double ResultDuration { get; private set; }
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

        public ICommand ToggleRandomCommand { get; }

        private bool _randomRunning;
        public bool RandomRunning
        {
            get => _randomRunning;
            set => ChangeProperty(value, ref _randomRunning);
        }

        private int _randomCount;
        public int RandomCount
        {
            get => _randomCount;
            set => ChangeProperty(value, ref _randomCount);
        }

        public TrajectoryToVelocityViewModel()
        {
            ModelJerk = new PlotModel();
            LinearAxis axisJerk = new LinearAxis() { Position = AxisPosition.Left, Title = "Jerk" };
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

            ToggleRandomCommand = new RelayCommand(() =>
            {
                if (RandomRunning)
                {
                    RandomRunning = false;
                }
                else
                {
                    RandomRunning = true;
                    Task.Run(RunRandom);
                }
            });

            Update();
        }

        private Task RunRandom()
        {
            Random random = new Random((int)DateTime.Now.Ticks);
            const int batchSize = 1000;
            while (RandomRunning)
            {
                _ = Parallel.For(0, batchSize, (i) =>
                  {
                      CalculateRandom(random);
                  });
                RandomCount += batchSize;
            }

            return Task.CompletedTask;
        }

        private static void CalculateRandom(Random random)
        {
            double jerkMax = RandomInRange(random, 5, 1000);
            double accelerationMax = RandomInRange(random, 5, 1000);
            double velocityMax = RandomInRange(random, 5, 1000);
            double acceleration0 = RandomInRange(random, -accelerationMax, accelerationMax);
            double velocity0 = RandomInRange(random, 0, 1000);

            MotionParameter motionParameter = new MotionParameter(jerkMax, -jerkMax, accelerationMax, -accelerationMax);
            Trajectory trajectory = TrajectoryToVelocity.Calculate(acceleration0, velocity0, velocityMax, motionParameter);
            trajectory.Validate();
        }

        private static double RandomInRange(Random random, double min, double max)
        {
            return min + random.NextDouble() * (max - min);
        }

        private void UpdateFromProperty()
        {
            Update();
        }

        private bool Update()
        {
            DataJ = new List<DataPoint>();
            DataA = new List<DataPoint>();
            DataV = new List<DataPoint>();
            DataS = new List<DataPoint>();
            DataBrakingDistance = new List<DataPoint>();
            ResultDuration = 0;
            ResultTrajectoryInstanceCase = 0;
            ResultMaxReachedVelocity = 0;

            bool success = true;
            try
            {
                MotionParameter motionParameter = new MotionParameter(JerkMax, -JerkMax, AccelerationMax, -AccelerationMax);

                DateTime started = DateTime.Now;
                Trajectory trajectory = TrajectoryToVelocity.Calculate(Acceleration0, Velocity0, VelocityMax, motionParameter);
                DateTime completed = DateTime.Now;
                CalculationTime = $"{(completed - started).TotalMilliseconds:0.000} ms";

                ResultDuration = trajectory.TotalDuration;
                for (double t = 0; t <= ResultDuration + 0.002; t += 0.001)
                {
                    trajectory.CalculateStatus(t, out double j, out double a, out double v, out double s);
                    DataJ.Add(new DataPoint(t, j));
                    DataA.Add(new DataPoint(t, a));
                    DataV.Add(new DataPoint(t, v));
                    DataS.Add(new DataPoint(t, s));
                }

                trajectory.Validate();
            }
            catch (Exception ex)
            {
                success = false;
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
            OnPropertyChanged(nameof(ResultMaxReachedVelocity));

            return success;
        }
    }
}
