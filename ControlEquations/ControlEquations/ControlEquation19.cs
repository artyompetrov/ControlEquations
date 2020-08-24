using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations.ControlEquations
{
    class ControlEquation19 : ControlEquation
    {
        public Voltage Ui { get; private set; }
        public Voltage Uj { get; private set; }
        public ActivePower Pij { get; private set; }
        public ReactivePower Qij { get; private set; }
        public ActivePower Pji { get; private set; }
        public ReactivePower Qji { get; private set; }
        public Constant R { get; private set; }
        public Constant X { get; private set; }
        public Constant B { get; private set; }

        public ControlEquation19(Voltage Ui, Voltage Uj, ActivePower Pij, ReactivePower Qij, ActivePower Pji, ReactivePower Qji, Constant r, Constant x, Constant b)
        {
            this.Ui = Ui;
            this.Uj = Uj;
            this.Pij = Pij;
            this.Qij = Qij;
            this.Pji = Pji;
            this.Qji = Qji;
            this.R = r;
            this.X = x;
            this.B = b;

            AddToArguments(Ui);
            AddToArguments(Uj);
            AddToArguments(Pij);
            AddToArguments(Qij);
            AddToArguments(Pji);
            AddToArguments(Qji);
            AddToConstants(r);
            AddToConstants(x);
            AddToConstants(b);
        }

        public override double Error
        {
            get
            {
                return (Qij.Value + Math.Pow(Ui.Value, 2)* B.Value / 2 + Qji.Value + Math.Pow(Uj.Value, 2) * B.Value / 2 + 1 / X.Value * (- Math.Pow(Ui.Value, 2) + Math.Pow(Uj.Value, 2) + R.Value * (Pij.Value + Pji.Value)));
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
                    var Pji = equationArguments[2].Value;
                    var Qji = equationArguments[3].Value;
                    var Uj = equationArguments[4].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = Math.Sqrt((- Qij - Qji - Math.Pow(Uj, 2) * B / 2 - 1 / X * (Math.Pow(Uj, 2) + R * (Pij + Pji))) / (B / 2 - 1 / X));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Qij, Pji, Qji, Uj };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Uj)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Pij = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Pji = equationArguments[2].Value;
                    var Qji = equationArguments[3].Value;
                    var Ui = equationArguments[4].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = Math.Sqrt((-Qij - Qji - Math.Pow(Ui, 2) * B / 2 - 1 / X * (Math.Pow(Ui, 2) + R * (Pij + Pji))) / (B / 2 - 1 / X));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Qij, Pji, Qji, Ui };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Pij = equationArguments[0].Value;
                    var Uj = equationArguments[1].Value;
                    var Pji = equationArguments[2].Value;
                    var Qji = equationArguments[3].Value;
                    var Ui = equationArguments[4].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = - (Math.Pow(Ui, 2) * B / 2 + Qji + Math.Pow(Uj, 2) * B / 2 + 1 / X * (-Math.Pow(Ui, 2) + Math.Pow(Uj, 2) + R * (Pij + Pji)));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Uj, Pji, Qji, Ui };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qji)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Pij = equationArguments[0].Value;
                    var Uj = equationArguments[1].Value;
                    var Pji = equationArguments[2].Value;
                    var Qij = equationArguments[3].Value;
                    var Ui = equationArguments[4].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = -(Math.Pow(Ui, 2) * B / 2 + Qij + Math.Pow(Uj, 2) * B / 2 + 1 / X * (-Math.Pow(Ui, 2) + Math.Pow(Uj, 2) + R * (Pij + Pji)));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Uj, Pji, Qij, Ui };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Pij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Qji = equationArguments[0].Value;
                    var Uj = equationArguments[1].Value;
                    var Pji = equationArguments[2].Value;
                    var Qij = equationArguments[3].Value;
                    var Ui = equationArguments[4].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = -(Qij + Math.Pow(Ui, 2) * B / 2 + Qji + Math.Pow(Uj, 2) * X + Math.Pow(Ui, 2) - Math.Pow(Uj, 2)) / R - Pji;
                    return res;
                }

                var arguments = new List<EquationArgument>() { Qji, Uj, Pji, Qij, Ui };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Pji)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Qji = equationArguments[0].Value;
                    var Uj = equationArguments[1].Value;
                    var Pij = equationArguments[2].Value;
                    var Qij = equationArguments[3].Value;
                    var Ui = equationArguments[4].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = -(Qij + Math.Pow(Ui, 2) * B / 2 + Qji + Math.Pow(Uj, 2) * X + Math.Pow(Ui, 2) - Math.Pow(Uj, 2)) / R - Pij;
                    return res;
                }

                var arguments = new List<EquationArgument>() { Qji, Uj, Pij, Qij, Ui };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            throw new ArgumentException("The argument that is supposed to be the subject wasn't found among ControlEquation arguments");
        }
    }
}
