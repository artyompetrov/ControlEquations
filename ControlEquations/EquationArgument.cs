using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public abstract class EquationArgument
    {
        public ValueSource ValueSource { get; set; }

        private List<ControlEquation> _controlEquations = new List<ControlEquation>();
        public IReadOnlyCollection<ControlEquation> ControlEquations => _controlEquations.AsReadOnly();

        internal void AddReferenceToControlEquation(ControlEquation controlEquation)
        {
            if (_controlEquations.Contains(controlEquation)) throw new InvalidOperationException("Link which you are trying to add is already in control equation list");
            _controlEquations.Add(controlEquation);
        }

        public bool HasValueSource => ValueSource != null;

        // логику присвоения аргументам значений возможно придется пересмотреть
        public double Value
        {
            get
            {
                return ValueSource.GetDouble();
            }
        }
    }
}
