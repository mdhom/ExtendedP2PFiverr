using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;
using OxyPlot;
using OxyPlot.Annotations;
using OxyPlot.Axes;
using OxyPlot.Series;
using Servus.Core.Ui;

namespace Trajectories.Ui
{
    public class TrajectoryToDistanceViewModel : Servus.Core.NotifyPropertyChangedBase
    {
        private double _targetDistance = 15000;
        public double TargetDistance
        {
            get => _targetDistance;
            set => ChangeProperty(value, ref _targetDistance, UpdateFromProperty);
        }

        private double _velocityMax = 1500;
        public double VelocityMax
        {
            get => _velocityMax;
            set => ChangeProperty(value, ref _velocityMax, UpdateFromProperty);
        }

        private double _jerkMax = 2000;
        public double JerkMax
        {
            get => _jerkMax;
            set => ChangeProperty(value, ref _jerkMax, UpdateFromProperty);
        }

        private double _accelerationMax = 500;
        public double AccelerationMax
        {
            get => _accelerationMax;
            set => ChangeProperty(value, ref _accelerationMax, UpdateFromProperty);
        }

        private double _acceleration0 = 500;
        public double Acceleration0
        {
            get => _acceleration0;
            set => ChangeProperty(value, ref _acceleration0, UpdateFromProperty);
        }

        private double _velocity0 = 500;
        public double Velocity0
        {
            get => _velocity0;
            set => ChangeProperty(value, ref _velocity0, UpdateFromProperty);
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

        public ICommand RecalcCommand { get; }
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

        private bool _updatingBatch;

        public TrajectoryToDistanceViewModel()
        {
                
            ModelJerk = new PlotModel();
            LinearAxis axisJerk = new LinearAxis() { Position = AxisPosition.Left, Title="Jerk" };
            ModelJerk.Axes.Add(axisJerk);
            AddHorizontalZeroLine(ModelJerk);

            ModelAcceleration = new PlotModel();
            LinearAxis axisAcceleration = new LinearAxis() { Position = AxisPosition.Left, Title = "Acceleration" };
            ModelAcceleration.Axes.Add(axisAcceleration);
            AddHorizontalZeroLine(ModelAcceleration);

            ModelVelocity = new PlotModel();
            LinearAxis axisVelocity = new LinearAxis() { Position = AxisPosition.Left, Title = "Velocity" };
            ModelVelocity.Axes.Add(axisVelocity);
            AddHorizontalZeroLine(ModelVelocity);

            ModelDistance = new PlotModel();
            LinearAxis axisDistance = new LinearAxis() { Position = AxisPosition.Left, Title = "Distance" };
            AddHorizontalZeroLine(ModelDistance);

            ToggleRandomCommand = new RelayCommand(() =>
            {
                Random random = new Random((int)DateTime.Now.Ticks);
                CalculateRandom(random);
            });

            RecalcCommand = new RelayCommand(Update);
            
            Update();
        }

        private static void AddHorizontalZeroLine(PlotModel model)
        {
            LineAnnotation lineAnnotation = new LineAnnotation() {
                StrokeThickness = 1,
                Color = OxyColors.Gray,
                Type = LineAnnotationType.Horizontal,
                Text = "0",
                Y = 0
            };
            model.Annotations.Add(lineAnnotation);
        }
        
        private Task RunRandom()
        {
            Random random = new Random((int)DateTime.Now.Ticks);
            const int batchSize = 1;
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

        private void CalculateRandom(Random random)
        {
            _updatingBatch = true;
            
            JerkMax = RandomInRange(random, 100, 5000);
            AccelerationMax = JerkMax / RandomInRange(random, 1, 10);
            VelocityMax = RandomInRange(random, 100, 1000);

            double distance = RandomInRange(random, 100, 10000);
            TrajectoryToDistanceResult result = TrajectoryToDistance.Calculate(distance, VelocityMax, 0, 0,
                new MotionParameter(JerkMax, -JerkMax, AccelerationMax, -AccelerationMax));
            Console.WriteLine($"Source Trajectory: distance={distance}, duration={result.TrajectoryChain.TotalDuration:.2} JerkMax={JerkMax}, AccelerationMax={AccelerationMax}, VelocityMax={VelocityMax}");
            
            double t = RandomInRange(random, 0, result.TrajectoryChain.TotalDuration);
            result.TrajectoryChain.GetStatus(t, out _, out double a, out double v, out _);
            Console.WriteLine($"New Trajectory: t={t}, a={a}, v={v}");
            
            Acceleration0 = a;
            Velocity0 = v;
            
            _updatingBatch = false;
            
            Update();
        }

        private static double RandomInRange(Random random, double min, double max)
        {
            return min + random.NextDouble() * (max - min);
        }

        private void UpdateFromProperty()
        {
            if (_updatingBatch)
                return;
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
                    ResultDistanceDifference = Math.Round(TargetDistance - ResultDistance, 3);
                    ResultDuration = result.TrajectoryChain.TotalDuration;
                    for (double t = 0; t <= ResultDuration + 0.002; t += 0.001)
                    {
                        result.TrajectoryChain.GetStatus(t, out double j, out double a, out double v, out double s);
                        DataJ.Add(new DataPoint(t, j));
                        DataA.Add(new DataPoint(t, a));
                        DataV.Add(new DataPoint(t, v));
                        DataS.Add(new DataPoint(t, s));
                        //DataBrakingDistance.Add(new DataPoint(t, calc.GetBrakingDistance(t)));
                    }
                    
                    result.TrajectoryChain.Validate();
                    
                    //ResultTrajectoryInstanceCase = calc.TrajectoryInstanceCase;
                    //ResultMaxReachedVelocity = calc.CalculateMaximumReachedVelocity();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"JerkMax={JerkMax}");
                Console.WriteLine($"AccelerationMax={AccelerationMax}");
                Console.WriteLine($"TargetDistance={TargetDistance}");
                Console.WriteLine($"VelocityMax={VelocityMax}");
                Console.WriteLine($"Velocity0={Velocity0}");
                Console.WriteLine($"Acceleration0={Acceleration0}");
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
