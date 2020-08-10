using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public class RearrangedControlEquation
    {
        private Func<List<EquationArgument>, List<Constant>, double> _calcSubjectValue;

        private readonly List<EquationArgument> _arguments = new List<EquationArgument>();

        private ReadOnlyCollection<EquationArgument> _argumentsReadOnly;

        public ReadOnlyCollection<EquationArgument> Arguments
        {
            get
            {
                if (_argumentsReadOnly == null)
                {
                    _argumentsReadOnly = _arguments.AsReadOnly();
                }                
                return _argumentsReadOnly;
            }
            
        }

        public ControlEquation ParentControlEquation { get; private set; }

        private readonly List<Constant> _constants = new List<Constant>();

        public EquationArgument Subject { get; private set; } 

        public double SubjectValue 
        {
            get 
            {
                return _calcSubjectValue.Invoke(_arguments, _constants);
            }
        }        

        internal RearrangedControlEquation(EquationArgument subject, List<EquationArgument> arguments,
            List<Constant> constants, Func<List<EquationArgument>, List<Constant>, double> calcSubjectValue, ControlEquation parentControlEquation)
        {
            _arguments = arguments;
            _constants = constants;
            Subject = subject;
            _calcSubjectValue = calcSubjectValue;
            ParentControlEquation = parentControlEquation;
        }
    }
}
