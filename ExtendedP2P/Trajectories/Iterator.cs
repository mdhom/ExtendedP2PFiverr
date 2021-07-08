using System;

namespace Trajectories
{
    public static class Iterator
    {
        public static T IterateFromMax<T>(double valMin, double valMax, Func<double, IteratorStepResult<T>> iteratorFunction)
        {
            int stepCount = 0;
            while (true)
            {
                double val = (valMin + valMax) / 2;

                IteratorStepResult<T> stepResult = iteratorFunction(val);

                if (stepResult.EndSearch)
                {
                    return stepResult.Result;
                }

                if (stepResult.IterateDown)
                {
                    valMax = val;
                }
                else
                {
                    valMin = val;
                }
                
                stepCount++;
                if (stepCount > 400)
                {
                    throw new Exception("No result found");
                }
            }
        }
    }

    public class IteratorStepResult<T>
    {
        public bool IterateDown { get; }
        public bool EndSearch { get; }
        public T Result { get; }

        public IteratorStepResult(bool iterateDown, bool endSearch, T result)
        {
            IterateDown = iterateDown;
            EndSearch = endSearch;
            Result = result;
        }
    }
}
