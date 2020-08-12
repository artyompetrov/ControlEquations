using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ControlEquations
{
    public class LineActivePowerBalanceEquation : ControlEquation
    {

        public Voltage Ui { get; private set; }
        public ActivePower Pij { get; private set; }
        public ReactivePower Qij { get; private set; }
        public ActivePower Pji { get; private set; }
        public Constant R { get; private set; }

        public LineActivePowerBalanceEquation(Voltage Ui, ActivePower Pij,  ReactivePower Qij, ActivePower Pji, Constant r)
        {
            this.Ui = Ui;
            this.Pij = Pij;
            this.Qij = Qij;
            this.Pji = Pji;
            this.R = r;

            AddToArguments(Ui);
            AddToArguments(Pij);
            AddToArguments(Qij);
            AddToArguments(Pji);
            AddToConstants(r);
        }

        public override double Error
        { 
            get
            {
                return Pij.Value + Pji.Value - R.Value * (Math.Pow(Pij.Value, 2) + Math.Pow(Qij.Value, 2)) / Math.Pow(Ui.Value, 2);
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

                    var R = equationConstants[0].Value;

                    var pijPlusPji = Pij + Pji;

                    if (IsCloseToZero(pijPlusPji, NearZeroMarginPowerFlow)) return double.NaN;
                    
                    var res = Math.Sqrt(R * (Math.Pow(Pij, 2) + Math.Pow(Qij, 2)) / (pijPlusPji));
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Qij, Pji };
                var constants = new List<Constant>() { R };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Pij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Ui = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Pji = equationArguments[2].Value;

                    var R = equationConstants[0].Value;

                    if (IsCloseToZero(Ui, NearZeroMarginVoltage)) return double.NaN;

                    var D = 1 - 4 * R * (R * Math.Pow(Qij, 2) / Math.Pow(Ui, 2) - Pji) / Math.Pow(Ui, 2);

                    if (D <= 0) return double.NaN; // D should be bigger that a=1

                    var result = (1 - Math.Sqrt(D)) / (2 * R / Math.Pow(Ui, 2));

                    return result;
                    
                }

                var arguments = new List<EquationArgument>() { Ui, Qij, Pji };
                var constants = new List<Constant>() { R };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Pji)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Ui = equationArguments[0].Value;
                    var Qij = equationArguments[1].Value;
                    var Pij = equationArguments[2].Value;

                    var R = equationConstants[0].Value;

                   if (IsCloseToZero(Ui, NearZeroMarginVoltage)) return double.NaN;

                    var res = R * (Math.Pow(Pij, 2) + Math.Pow(Qij, 2))/ Math.Pow(Ui, 2) - Pij;

                    return res;

                }

                var arguments = new List<EquationArgument>() { Ui, Qij, Pij };
                var constants = new List<Constant>() { R };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            if (subject == Qij)
            {
                double calcSubjectValue(List<EquationArgument> equationArguments, List<Constant> equationConstants)
                {
                    var Pij = equationArguments[0].Value;
                    var Ui = equationArguments[1].Value;
                    var Pji = equationArguments[2].Value;

                    var R = equationConstants[0].Value;

                    var pijPlusPji = Pij + Pji;

                    var res = Math.Sqrt(pijPlusPji*Math.Pow(Ui,2)/R-Math.Pow(Pij,2));
                    
                    return res;
                }

                var arguments = new List<EquationArgument>() { Pij, Ui, Pji };
                var constants = new List<Constant>() { R };
                return new RearrangedControlEquation(subject, arguments, constants, calcSubjectValue, this);
            }

            throw new ArgumentException("The argument that is supposed to be the subject wasn't found among ControlEquation arguments");
        }
    }
}
