using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public abstract class ValueSource
    {
        public abstract double GetDouble();
        public abstract bool GetBool();
    }
}
