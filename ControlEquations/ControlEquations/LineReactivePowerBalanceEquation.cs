﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public class LineReactivePowerBalanceEquation : ControlEquation
    {

        public Voltage Ui { get; private set; }
        public ActivePower Pij { get; private set; }
        public ReactivePower Qij { get; private set; }
        public ReactivePower Qji { get; private set; }
        public Constant X { get; private set; }

        public LineReactivePowerBalanceEquation(Voltage Ui, ActivePower Pij, ReactivePower Qij, ReactivePower Qji, Constant x)
        {
            this.Ui = Ui;
            this.Pij = Pij;
            this.Qij = Qij;
            this.Qji = Qji;
            this.X = x;

            AddToArguments(Ui);
            AddToArguments(Pij);
            AddToArguments(Qij);
            AddToArguments(Qji);
            AddToConstants(x);
        }

        public override double Error
        {
            get
            {
                return Qij.Value + Qji.Value - X.Value * (Math.Pow(Pij.Value, 2) + Math.Pow(Qij.Value, 2)) / Math.Pow(Ui.Value, 2);
            }
        }

        public override RearrangedControlEquation GetRearrangedEquation(EquationArgument subject)
        {
            if(subject == null) throw new ArgumentException("Subject cannot be equal to null");

            if (subject == Ui)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Qij = equationArguments[0].Value;
                    var Qji = equationArguments[1].Value;
                    var Pij = equationArguments[2].Value;

                    var X = equationConstants[0].Value;

                    var qijPlusQji = Qij + Qji;

                    if (IsCloseToZero(qijPlusQji, NearZeroMarginPowerFlow)) return double.NaN;

                    var res = Math.Sqrt(X * (Math.Pow(Pij, 2) + Math.Pow(Qij, 2)) / (qijPlusQji));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Qij, Qji, Pij };
                var constants = new List<Constant>() { X };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Ui = equationArguments[0].Value;
                    var Pij = equationArguments[1].Value;
                    var Qji = equationArguments[2].Value;

                    var X = equationConstants[0].Value;

                    if (IsCloseToZero(Ui, NearZeroMarginVoltage)) return double.NaN;

                    var D = 1 - 4 * X * (X * Math.Pow(Pij, 2) / Math.Pow(Ui, 2) - Qji) / Math.Pow(Ui, 2);

                    if (D <= 0) return double.NaN; // D should be bigger that a=1

                    var result = (1 - Math.Sqrt(D)) / (2 * X / Math.Pow(Ui, 2));

                    return result;

                }

                var arguments = new List<EquationArgument>() { Ui, Pij, Qji };
                var constants = new List<Constant>() { X };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qji)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Ui = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Pij = equationArguments[2].Value;

                    var X = equationConstants[0].Value;

                    if (IsCloseToZero(Ui, NearZeroMarginVoltage)) return double.NaN;

                    var res = X * (Math.Pow(Pij, 2) + Math.Pow(Qij, 2)) / Math.Pow(Ui, 2) - Pij;

                    return res;

                }

                var arguments = new List<EquationArgument>() { Ui, Qij, Pij };
                var constants = new List<Constant>() { X };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Pij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Qij = equationArguments[0].Value;
                    var Ui = equationArguments[1].Value;
                    var Qji = equationArguments[2].Value;

                    var X = equationConstants[0].Value;

                    var qijPlusQji = Qij + Qji;

                    var res = Math.Sqrt(qijPlusQji * Math.Pow(Ui, 2) / X - Math.Pow(Qij, 2));

                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Ui, Qji };
                var constants = new List<Constant>() { X };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            throw new ArgumentException("The argument that is supposed to be the subject wasn't found among ControlEquation arguments");
        }
    }
}
