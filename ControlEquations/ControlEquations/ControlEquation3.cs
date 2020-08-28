using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations.ControlEquations
{
    class ControlEquation3 : ControlEquation
    {
        public Voltage Ui { get; private set; }
        public Voltage Uj { get; private set; }
        public ActivePower Pij { get; private set; }
        public ReactivePower Qij { get; private set; }
        public ReactivePower Qji { get; private set; }
        public Constant R { get; private set; }
        public Constant X { get; private set; }
        public Constant B { get; private set; }

        public ControlEquation3(Voltage Ui, Voltage Uj, ActivePower Pij, ReactivePower Qij, ReactivePower Qji, Constant r, Constant x, Constant b)
        {
            this.Ui = Ui;
            this.Uj = Uj;
            this.Pij = Pij;
            this.Qij = Qij;
            this.Qji = Qji;
            this.R = r;
            this.R = x;
            this.B = b;

            AddToArguments(Ui);
            AddToArguments(Uj);
            AddToArguments(Pij);
            AddToArguments(Qij);
            AddToArguments(Qji);
            AddToConstants(r);
            AddToConstants(x);
            AddToConstants(b);
        }

        public override double Error
        {
            get
            {
                return (Math.Pow(Ui.Value, 2) * Qij.Value + Math.Pow(Ui.Value, 4) * B.Value / 2 - Qji.Value * Math.Pow(Ui.Value, 2) - Math.Pow(Ui.Value, 2) * Math.Pow(Uj.Value, 2) * B.Value / 2 - Math.Pow(Pij.Value, 2) * X.Value - Math.Pow(Qij.Value, 2) * X.Value - 2 * Qij.Value * X.Value * Math.Pow(Ui.Value, 2) * B.Value / 2 - Math.Pow(Ui.Value, 4) * Math.Pow(B.Value, 2) * X.Value / 4);
            }
        }

        public override RearrangedControlEquation GetRearrangedEquation(EquationArgument subject)
        {
            if (subject == null) throw new ArgumentException("Subject cannot be equal to null");

            if (subject == Ui)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Pij = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Qji = equationArguments[2].Value;
                    var Uj = equationArguments[3].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var D = Math.Pow((Qij - Qji - Math.Pow(Uj, 2) * B / 2 - 2 * Qij * X * B / 2), 2) - 4 * (B / 2 - Math.Pow(B, 2) * X / 4) * (-Math.Pow(Pij, 2) * X - Math.Pow(Qij, 2) * X);

                    if (D <= 0) return double.NaN;

                    var res = Math.Sqrt(((-(Qij - Qji - Math.Pow(Uj, 2) * B / 2 - 2 * Qij * X * B / 2)) - Math.Sqrt(D)) / (2 * (B / 2 - Math.Pow(B, 2) * X / 4)));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Qij,Qji, Uj };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Uj)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Pij = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Qji = equationArguments[2].Value;
                    var Ui = equationArguments[3].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = Math.Sqrt((Math.Pow(Ui, 2) * Qij + Math.Pow(Ui, 4) * B / 2 - Qji * Math.Pow(Ui, 2) - Math.Pow(Pij, 2) * X - Math.Pow(Qij, 2) * X - 2 * Qij * X * Math.Pow(Ui, 2) * B / 2 - Math.Pow(Ui, 4) * Math.Pow(B, 2) * X / 4) / (Math.Pow(Ui, 2) * B / 2));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Qij, Qji, Ui };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Pij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Uj = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Qji = equationArguments[2].Value;
                    var Ui = equationArguments[3].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = Math.Sqrt((Math.Pow(Ui, 2) * Qij + Math.Pow(Ui, 4) * B / 2 - Qji * Math.Pow(Ui, 2) - Math.Pow(Ui, 2) * Math.Pow(Uj, 2) * B / 2 - Math.Pow(Qij, 2) * X - 2 * Qij * X * Math.Pow(Ui, 2) * B / 2 - Math.Pow(Ui, 4) * Math.Pow(B, 2) * X / 4) / X);
                    return res;
                }

                var arguments = new List<EquationArgument>() { Uj, Qij, Qji, Ui };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Pij = equationArguments[0].Value;
                    var Ui = equationArguments[1].Value;
                    var Qji = equationArguments[2].Value;
                    var Uj = equationArguments[3].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var D = Math.Pow((Math.Pow(Ui, 2) - 2 * X * Math.Pow(Ui, 2) * B / 2), 2) - 4 * (- X) * (Math.Pow(Ui, 4) * B / 2 - Qji * Math.Pow(Ui, 2) - Math.Pow(Ui, 2) * Math.Pow(Uj, 2) * B / 2 - Math.Pow(Pij, 2) * X - Math.Pow(Ui, 4) * Math.Pow(B, 2) * X / 4);

                    if (D <= 0) return double.NaN;

                    var res = (-(Math.Pow(Ui, 2) - 2 * X * Math.Pow(Ui, 2) * B / 2) - Math.Sqrt(D)) / (- 2 * X);
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Ui, Qji, Uj };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qji)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Pij = equationArguments[0].Value;
                    var Ui = equationArguments[1].Value;
                    var Qij = equationArguments[2].Value;
                    var Uj = equationArguments[3].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = (Math.Pow(Ui, 2) * Qij + Math.Pow(Ui, 4) * B / 2 - Math.Pow(Ui, 2) * Math.Pow(Uj, 2) * B / 2 - Math.Pow(Pij, 2) * X - Math.Pow(Qij, 2) * X - 2 * Qij * X * Math.Pow(Ui, 2) * B / 2 - Math.Pow(Ui, 4) * Math.Pow(B, 2) * X / 4) / Math.Pow(Ui, 2);
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Ui, Qij, Uj };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            throw new ArgumentException("The argument that is supposed to be the subject wasn't found among ControlEquation arguments");
        }
    }
}
