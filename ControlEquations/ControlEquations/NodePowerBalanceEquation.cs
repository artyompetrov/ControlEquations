using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public class NodePowerBalanceEquation<T> : ControlEquation where T : Power
    {
        public override double Error
        { 
            get
            {
                double error = 0.0;
                foreach (var argument in _arguments)
                {
                    error += argument.Value;
                }
                return error;
            }
        }

        public override RearrangedControlEquation GetRearrangedEquation(EquationArgument subject)
        {
            if (subject == null) throw new ArgumentException("Subject cannot be equal to null");

            var matches = 0;
            var arguments = new List<EquationArgument>();

            foreach (var argument in _arguments)
            {
                if (argument == subject)
                {
                    matches++;
                    continue;
                }

                arguments.Add(argument);
            }

            if (matches != 1) throw new ArgumentException("The argument that is supposed to be the subject wasn't found among ControlEquation arguments");


            double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
            {                
                double subjectValue = 0.0;
                foreach (var argument in equationArguments)
                {
                    subjectValue -= argument.Value;
                }
                return subjectValue;
            };

            return new RearrangedControlEquation(subject, arguments, null, calcSubjectValue, this);
        }


        public NodePowerBalanceEquation(params T[] powerArguments) : this((ICollection<T>)powerArguments) { }
        public NodePowerBalanceEquation(ICollection<T> powerArguments)
        {
            if (powerArguments.Count < 2) throw new ArgumentException("NodeBalanceEquation should receive at least two arguments");

            foreach (var argument in powerArguments)
            {
                AddToArguments(argument);
            }            
        }
    }
}
