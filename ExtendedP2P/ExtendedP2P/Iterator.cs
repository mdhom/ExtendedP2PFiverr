using System;
using System.Collections.Generic;
using System.Text;

namespace ExtendedP2P
{
    public static class Iterator
    {
        public static T IterateFromMax<T>(double valMin, double valMax, Func<double, IteratorStepResult<T>> validator)
        {
            double val = (valMin + valMax) / 2;
            IteratorStepResult<T> stepResult = validator(val);
            if (stepResult.EndSearch)
            {
                return stepResult.Result;
            }

            if (stepResult.IterateDown)
            {
                return IterateFromMax(valMin, val, validator);
            }
            else
            {
                return IterateFromMax(val, valMax, validator);
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
