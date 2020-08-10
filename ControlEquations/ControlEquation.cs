using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Dynamic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public abstract class ControlEquation
    {
        static public double NearZeroMarginPowerFlow = 5;
        static public double NearZeroMarginVoltage = 5;

        protected bool IsCloseToZero(double value, double nearZeroMargin)
        {
            return Math.Abs(value) < nearZeroMargin;
        }

        public abstract double Error { get; }

        public abstract RearrangedControlEquation GetRearrangedEquation(EquationArgument subject);


        protected readonly List<Constant> _constants = new List<Constant>();
        private ReadOnlyCollection<Constant> _constantsRadOnly;
        public ReadOnlyCollection<Constant> Constants
        {
            get
            {
                if (_constantsRadOnly == null)
                {
                    _constantsRadOnly = _constants.AsReadOnly();
                }

                return _constantsRadOnly;
            }
        }

        protected void AddToConstants(Constant constant)
        {
            constant.AddReferenceToControlEquation(this);
            _constants.Add(constant);
        }

        protected readonly List<EquationArgument> _arguments = new List<EquationArgument>();
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
        protected void AddToArguments(EquationArgument argument)
        {
            argument.AddReferenceToControlEquation(this);
            _arguments.Add(argument);
        }
                

           
    }
}
