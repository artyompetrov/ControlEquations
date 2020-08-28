using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations.ControlEquations
{
    class ControlEquation13 : ControlEquation
    {
        public Voltage Ui { get; private set; }
        public Voltage Uj { get; private set; }
        public ActivePower Pij { get; private set; }
        public ReactivePower Qij { get; private set; }
        public ActivePower Pji { get; private set; }
        public Constant R { get; private set; }
        public Constant X { get; private set; }
        public Constant B { get; private set; }

        public ControlEquation13(Voltage Ui, Voltage Uj, ActivePower Pij, ReactivePower Qij, ActivePower Pji, Constant r, Constant x, Constant b)
        {
            this.Ui = Ui;
            this.Uj = Uj;
            this.Pij = Pij;
            this.Qij = Qij;
            this.Pji = Pji;
            this.R = r;
            this.R = x;
            this.B = b;

            AddToArguments(Ui);
            AddToArguments(Uj);
            AddToArguments(Pij);
            AddToArguments(Qij);
            AddToArguments(Pji);
            AddToConstants(r);
            AddToConstants(x);
            AddToConstants(b);
        }

        public override double Error
        {
            get
            {
                return (Qij.Value + Math.Pow(Ui.Value, 2) * B.Value / 2 + (-(Math.Pow(X.Value,2) + Math.Pow(R.Value, 2)) * Pij.Value + (Math.Pow(X.Value, 2) + Math.Pow(R.Value, 2)) * Pji.Value) / (2 * R.Value * X.Value) - (Math.Pow(Ui.Value, 2) - Math.Pow(Uj.Value, 2)) / (2 * X.Value));
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
                    var Uj = equationArguments[2].Value;
                    var Pji = equationArguments[3].Value;


                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = Math.Sqrt((-Qij - (-(Math.Pow(X, 2) + Math.Pow(R, 2)) * Pij + (Math.Pow(X, 2) + Math.Pow(R, 2)) * Pji) / (2 * R * X) - Math.Pow(Uj, 2) / (2 * X)) / (B / 2 - 1 / (2 * X)));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Qij, Uj, Pji };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Uj)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Pij = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Ui = equationArguments[2].Value;
                    var Pji = equationArguments[3].Value;


                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = Math.Sqrt((Qij + Math.Pow(Ui, 2) * B / 2 + (-(Math.Pow(X, 2) + Math.Pow(R, 2)) * Pij + (Math.Pow(X, 2) + Math.Pow(R, 2)) * Pji) / (2 * R * X) - Math.Pow(Ui, 2) / (2 * X)) * 2 * X);
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Qij, Ui, Pji };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Pij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Uj = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Ui = equationArguments[2].Value;
                    var Pji = equationArguments[3].Value;


                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = (Qij + Math.Pow(Ui, 2) * B / 2 + ((Math.Pow(X, 2) + Math.Pow(R, 2)) * Pji) / (2 * R * X) - (Math.Pow(Ui, 2) - Math.Pow(Uj, 2)) / (2 * X)) * 2 * R * X / (Math.Pow(X, 2) + Math.Pow(R, 2));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Uj, Qij, Ui, Pji };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Pji)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Uj = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Ui = equationArguments[2].Value;
                    var Pij = equationArguments[3].Value;


                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = -(Qij + Math.Pow(Ui, 2) * B / 2 + (-(Math.Pow(X, 2) + Math.Pow(R, 2)) * Pij) / (2 * R * X) - (Math.Pow(Ui, 2) - Math.Pow(Uj, 2)) / (2 * X)) * 2 * R * X / (Math.Pow(X, 2) + Math.Pow(R, 2));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Uj, Qij, Ui, Pij };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Uj = equationArguments[0].Value;
                    var Pji = equationArguments[1].Value;
                    var Ui = equationArguments[2].Value;
                    var Pij = equationArguments[3].Value;


                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = -(Math.Pow(Ui, 2) * B / 2 + (-(Math.Pow(X, 2) + Math.Pow(R, 2)) * Pij + (Math.Pow(X, 2) + Math.Pow(R, 2)) * Pji) / (2 * R * X) - (Math.Pow(Ui, 2) - Math.Pow(Uj, 2)) / (2 * X));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Uj, Pji, Ui, Pij };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            throw new ArgumentException("The argument that is supposed to be the subject wasn't found among ControlEquation arguments");
        }

    }
}
