using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public class LinearCombination
    {
        public ControlEquation BaseEquation { get; private set; }

        public IReadOnlyCollection<EquationArgument> Arguments
        {
            get 
            {
                throw new NotImplementedException();
            }
        }

        public double CalculateError()
        {
            throw new NotImplementedException();
        }


    }
}
