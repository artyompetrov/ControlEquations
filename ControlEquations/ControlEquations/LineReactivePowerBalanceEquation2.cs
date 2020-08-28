using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations.ControlEquations
{
    class LineReactivePowerBalanceEquation2 : ControlEquation
    {
        public Voltage Ui { get; private set; }
        public ActivePower Pij { get; private set; }
        public ActivePower Pji { get; private set; }
        public ReactivePower Qij { get; private set; }
        public ReactivePower Qji { get; private set; }
        public Constant X { get; private set; }
        public Constant R { get; private set; }

        public LineReactivePowerBalanceEquation2(Voltage Ui, ActivePower Pij, ActivePower Pji, ReactivePower Qij, ReactivePower Qji, Constant x, Constant r)
        {
            this.Ui = Ui;
            this.Pij = Pij;
            this.Pji = Pji;
            this.Qij = Qij;
            this.Qji = Qji;
            this.X = x;
            this.R = r;

            AddToArguments(Ui);
            AddToArguments(Pij);
            AddToArguments(Pji);
            AddToArguments(Qij);
            AddToArguments(Qji);
            AddToConstants(x);
            AddToConstants(r);
        }

        public override double Error
        {
            get
            {
                return Qij.Value + Qji.Value + R.Value * (Pij.Value - Pji.Value)/X.Value - (Math.Pow(R.Value,2) + Math.Pow(X.Value, 2))*(Math.Pow(Pij.Value, 2) + Math.Pow(Qij.Value, 2)) / (Math.Pow(Ui.Value,2) * X.Value);
            }
        }

        public override RearrangedControlEquation GetRearrangedEquation(EquationArgument subject)
        {
            if (subject == null) throw new ArgumentException("Subject cannot be equal to null");

            if (subject == Ui)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Qij = equationArguments[0].Value;
                    var Qji = equationArguments[1].Value;
                    var Pij = equationArguments[2].Value;
                    var Pji = equationArguments[3].Value;

                    var X = equationConstants[0].Value;
                    var R = equationConstants[1].Value;

                    var qijPlusQji = Qij + Qji;
                    var pijPlusPji = Pij + Pji;

                    if (IsCloseToZero((qijPlusQji * X + pijPlusPji * R), NearZeroMarginPowerFlow)) return double.NaN;

                    var res = Math.Sqrt(((Math.Pow(R, 2) + Math.Pow(X, 2)) * (Math.Pow(Pij, 2) + Math.Pow(Qij, 2)) / (qijPlusQji * X + pijPlusPji * R)));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Qij, Qji, Pij, Pji };
                var constants = new List<Constant>() { X, R };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Ui = equationArguments[0].Value;
                    var Pij = equationArguments[1].Value;
                    var Qji = equationArguments[2].Value;
                    var Pji = equationArguments[3].Value;

                    var X = equationConstants[0].Value;
                    var R = equationConstants[1].Value;

                    var pijPlusPji = Pij + Pji;

                    if (IsCloseToZero(Ui, NearZeroMarginVoltage)) return double.NaN;

                    var D = 1 - 4 * R *(Math.Pow(R, 2) + Math.Pow(X, 2)) / (Math.Pow(Ui,2) * Math.Pow(X, 2)) * (Math.Pow(Pij, 2) * (Math.Pow(R, 2) + Math.Pow(X, 2)) / (Math.Pow(Ui, 2) * Math.Pow(X, 2)) - (pijPlusPji / X) * R - Qji);

                    if (D <= 0) return double.NaN; // D should be bigger that a=1

                    var result = (- 1 - Math.Sqrt(D)) / (2 * (Math.Pow(R, 2) + Math.Pow(X, 2)) / (Math.Pow(Ui, 2) * Math.Pow(X, 2)));

                    return result;

                }

                var arguments = new List<EquationArgument>() { Ui, Pij, Qji, Pji };
                var constants = new List<Constant>() { X, R };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qji)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Ui = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Pij = equationArguments[2].Value;
                    var Pji = equationArguments[3].Value;

                    var X = equationConstants[0].Value;
                    var R = equationConstants[1].Value;

                    var pijPlusPji = Pij + Pji;

                    if (IsCloseToZero(Ui, NearZeroMarginVoltage)) return double.NaN;

                    var res = - Qij - pijPlusPji * R / X + (Math.Pow(R, 2) + Math.Pow(X, 2)) * ((Math.Pow(Pij, 2) + Math.Pow(Qij, 2)) / Math.Pow(Ui,2) * X);

                    return res;

                }

                var arguments = new List<EquationArgument>() { Ui, Qij, Pij, Pji };
                var constants = new List<Constant>() { X, R };
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
