using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public class NodeBalanceEquation<T> : ControlEquation where T : PowerFlow
    {
        public override double CalculateError()
        {
            double error = 0.0;
            foreach (var argument in _arguments)
            {
                error += argument.Value;
            }
            return error;
        }

        public override double SolveForArgument(EquationArgument argument)
        {
            double value = 0.0;
            foreach (var equationArgument in _arguments)
            {
                if (equationArgument != argument) value -= equationArgument.Value;
            }
            return value;
        }

        public NodeBalanceEquation(params T[] powerFlowArguments)
        {
            if (powerFlowArguments.Length < 2) throw new ArgumentException("NodeBalanceEquation should receive at least two arguments");
            
            _arguments.AddRange(powerFlowArguments);

            foreach (var argument in powerFlowArguments)
            {
                argument.AddReferenceToControlEquation(this);
            }
        }
    }
}
