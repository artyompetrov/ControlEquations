using System;
using System.Collections.Generic;
using System.Dynamic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public abstract class ControlEquation
    {
        public abstract double CalculateError();

        public abstract double SolveForArgument(EquationArgument argument);

        protected List<EquationArgument> _arguments = new List<EquationArgument>();

        public IReadOnlyCollection<EquationArgument> Arguments => _arguments.AsReadOnly();        
    }
}
