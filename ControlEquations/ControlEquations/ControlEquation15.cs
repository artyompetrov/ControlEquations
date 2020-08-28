using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations.ControlEquations
{
    class ControlEquation15 : ControlEquation
    {
        public Voltage Ui { get; private set; }
        public Voltage Uj { get; private set; }
        public ActivePower Pij { get; private set; }
        public ReactivePower Qij { get; private set; }
        public ReactivePower Qji { get; private set; }
        public Constant R { get; private set; }
        public Constant X { get; private set; }
        public Constant B { get; private set; }


        public ControlEquation15(Voltage Ui, Voltage Uj, ActivePower Pij, ReactivePower Qij, ReactivePower Qji, Constant r, Constant x, Constant b)
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
                return (((Math.Pow(X.Value, 2) - Math.Pow(R.Value, 2)) * (Qij.Value + Math.Pow(Ui.Value, 2) * B.Value / 2) + (Math.Pow(X.Value, 2) + Math.Pow(R.Value, 2)) * (Qji.Value + Math.Pow(Uj.Value, 2) * B.Value / 2)) / (2 * R.Value * X.Value) + Pij.Value - (Math.Pow(Ui.Value, 2) - Math.Pow(Uj.Value, 2)) / 2 / R.Value);
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
                    var Qji = equationArguments[3].Value;


                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = Math.Sqrt((-(Math.Pow(X, 2) - Math.Pow(R, 2)) * Qij / (2 * R * X) - (Math.Pow(X, 2) + Math.Pow(R, 2)) * Qji / (2 * R * X) - (Math.Pow(X, 2) + Math.Pow(R, 2)) * B * Math.Pow(Uj, 2)/ (4 * R * X) - Pij - Math.Pow(Uj, 2) / (2 * R)) / ((Math.Pow(X, 2) - Math.Pow(R, 2)) * B / (4 * R * X) - 1 / ( 2 * R)));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Qij, Uj, Qji };
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
                    var Qji = equationArguments[3].Value;


                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = Math.Sqrt((-(Math.Pow(X, 2) - Math.Pow(R, 2)) * Qij / (2 * R * X) + (Math.Pow(X, 2) + Math.Pow(R, 2)) * Qji / (2 * R * X) + (Math.Pow(X, 2) - Math.Pow(R, 2)) * B * Math.Pow(Ui, 2) / (4 * R * X) + Pij - Math.Pow(Ui, 2) / (2 * R)) / ((Math.Pow(X, 2) + Math.Pow(R, 2)) * B / (4 * R * X) + 1 / (2 * R)));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Qij, Ui, Qji };
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
                    var Qji = equationArguments[3].Value;


                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = (Math.Pow(Ui, 2) - Math.Pow(Uj, 2)) / 2 / R - ((Math.Pow(X, 2) - Math.Pow(R, 2)) * (Qij + Math.Pow(Ui, 2) * B / 2) + (Math.Pow(X, 2) + Math.Pow(R, 2)) * (Qji + Math.Pow(Uj, 2) * B / 2)) / (2 * R * X);
                    return res;
                }

                var arguments = new List<EquationArgument>() { Uj, Qij, Ui, Qji };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Uj = equationArguments[0].Value;
                    var Pij = equationArguments[1].Value;
                    var Ui = equationArguments[2].Value;
                    var Qji = equationArguments[3].Value;


                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = (- Pij + (Math.Pow(Ui, 2) - Math.Pow(Uj, 2)) / 2 / R / X - (Math.Pow(X, 2) + Math.Pow(R, 2)) * (Qji + Math.Pow(Uj, 2) * B / 2)) / (Math.Pow(X, 2) - Math.Pow(R, 2)) - Math.Pow(Ui, 2) * B / 2;
                    return res;
                }

                var arguments = new List<EquationArgument>() { Uj, Pij, Ui, Qji };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qji)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Uj = equationArguments[0].Value;
                    var Pij = equationArguments[1].Value;
                    var Ui = equationArguments[2].Value;
                    var Qij = equationArguments[3].Value;


                    var R = equationConstants[0].Value;
                    var X = equationConstants[1].Value;
                    var B = equationConstants[2].Value;

                    var res = (-Pij + (Math.Pow(Ui, 2) - Math.Pow(Uj, 2)) / 2 / R / X - (Math.Pow(X, 2) - Math.Pow(R, 2)) * (Qij + Math.Pow(Ui, 2) * B / 2)) / (Math.Pow(X, 2) + Math.Pow(R, 2)) - Math.Pow(Uj, 2) * B / 2;
                    return res;
                }

                var arguments = new List<EquationArgument>() { Uj, Pij, Ui, Qij };
                var constants = new List<Constant>() { R, X, B };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            throw new ArgumentException("The argument that is supposed to be the subject wasn't found among ControlEquation arguments");
        }


    }
}
