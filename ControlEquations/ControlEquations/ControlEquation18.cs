using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations.ControlEquations
{
    class ControlEquation18 : ControlEquation
    {
        public Voltage Uj { get; private set; }
        public ActivePower Pij { get; private set; }
        public ReactivePower Qij { get; private set; }
        public ActivePower Pji { get; private set; }
        public ReactivePower Qji { get; private set; }
        public Constant R { get; private set; }
        public Constant X { get; private set; }
        public Constant B { get; private set; }

        public ControlEquation18(Voltage Uj, ActivePower Pij, ReactivePower Qij, ActivePower Pji, ReactivePower Qji, Constant r, Constant x, Constant b)
        {
            this.Uj = Uj;
            this.Pij = Pij;
            this.Qij = Qij;
            this.Pji = Pji;
            this.Qji = Qji;
            this.R = r;
            this.X = x;
            this.B = b;

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
                return (Qij.Value - 1 / (1 - X.Value * B.Value) * (Qji.Value + Math.Pow(Uj.Value, 2) ) + 1 / R.Value * (-(X.Value- B.Value / 2 * (Math.Pow(X.Value, 2) - Math.Pow(R. Value, 2))) * Pij.Value + (X.Value - B.Value / 2 * (Math.Pow(X.Value, 2) + Math.Pow(R.Value, 2))) * Pji.Value));
            }
        }

        public override RearrangedControlEquation GetRearrangedEquation(EquationArgument subject)
        {
            if (subject == null) throw new ArgumentException("Subject cannot be equal to null");

            if (subject == Uj)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Pij = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Pji = equationArguments[2].Value;
                    var Qji = equationArguments[3].Value;

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = Math.Sqrt((Qij - 1 / (1 - X * B) * Qji + 1 / R * (-(X -B / 2 * (Math.Pow(X, 2) - Math.Pow(R, 2))) * Pij + (X - B / 2 * (Math.Pow(X, 2) + Math.Pow(R, 2))) * Pji)) * 2 *(1 - X * B) / B);
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Qij, Pji, Qji };
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

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = 1 / (1 - X * B) * (Qji + Math.Pow(Uj, 2)) - 1 / R * (-(X - B / 2 * (Math.Pow(X, 2) - Math.Pow(R, 2))) * Pij + (X - B / 2 * (Math.Pow(X, 2) + Math.Pow(R, 2))) * Pji);
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Uj, Pji, Qji };
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

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = (Qij - 1 / (1 - X * B) * Math.Pow(Uj, 2) * B /  2 + 1 / R * (-(X - B / 2 * (Math.Pow(X, 2) - Math.Pow(R, 2))) * Pij + (X - B / 2 * (Math.Pow(X, 2) + Math.Pow(R, 2))) * Pji)) * (1 - X * B);
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Uj, Pji, Qij };
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

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = (Qij - 1 / (1 - X * B) * (Qji + Math.Pow(Uj, 2) * B / 2) + 1 / R * Pji * (X - B / 2 * (Math.Pow(X, 2) + Math.Pow(R, 2)))) / (1 / R * (X - B / 2 * (Math.Pow(X, 2) - Math.Pow(R, 2))));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Qji, Uj, Pji, Qij };
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

                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = - (Qij - 1 / (1 - X * B) * (Qji + Math.Pow(Uj, 2) * B / 2) - 1 / R * Pij * (X - B / 2 * (Math.Pow(X, 2) - Math.Pow(R, 2)))) / (1 / R * (X - B / 2 * (Math.Pow(X, 2) + Math.Pow(R, 2))));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Qji, Uj, Pij, Qij };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            throw new ArgumentException("The argument that is supposed to be the subject wasn't found among ControlEquation arguments");
        }
    }
}
