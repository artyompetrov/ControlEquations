﻿using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public abstract class EquationArgument
    {
        public ValueSource ValueSource { get; set; }

        private readonly List<ControlEquation> _controlEquations = new List<ControlEquation>();

        private ReadOnlyCollection<ControlEquation> _controlEquationsReadOnly; 
        public ReadOnlyCollection<ControlEquation> ControlEquations
        {
            get
            {
                if (_controlEquationsReadOnly == null)
                {
                    _controlEquationsReadOnly = _controlEquations.AsReadOnly();
                }
                return _controlEquationsReadOnly;
            }
        }

        internal void AddReferenceToControlEquation(ControlEquation controlEquation)
        {
            if (_controlEquations.Contains(controlEquation)) throw new InvalidOperationException("Link what you are trying to add is already in control equation list");
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
