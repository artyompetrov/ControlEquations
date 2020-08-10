using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations.Tests
{
    public class TestValueSource : ValueSource
    {
        public override bool GetBool()
        {
            throw new NotImplementedException();
        }

        public override double GetDouble()
        {
            return (double)_value;
        }

        private double? _value;
        public TestValueSource(double value)
        {
            _value = value;
        }
    }
}
