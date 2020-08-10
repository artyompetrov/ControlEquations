using System;
using System.Collections.Generic;
using System.Text;
using Xunit;
using FluentAssertions;

namespace ControlEquations.Tests
{
    public class ControlEquationTestsParent
    {

        public T Argument<T>(double value) where T : EquationArgument, new()
        {
            var valueSource = new TestValueSource(value);

            var argument = new T();

            argument.ValueSource = valueSource;

            return argument; 
        }

        public void RearrangeEquationTest(ControlEquation equation, bool skipNullEquations = false, bool skipNaN = false,  double defaultRangeWidth=0.1, 
            Dictionary<EquationArgument, double> specificRangeWidth = null, Dictionary<EquationArgument, bool> specificSkipNaN = null)
        {
                        
            foreach (var argument in equation.Arguments)
            {
                var rearrangedEquation = equation.GetRearrangedEquation(argument);

                if (skipNullEquations && rearrangedEquation == null) continue;

                var halfOfRangeWidth = defaultRangeWidth / 2;

                if (specificRangeWidth != null)
                {
                    if (specificRangeWidth.ContainsKey(argument))
                    {
                        halfOfRangeWidth = specificRangeWidth[argument] / 2;
                    }
                }

                if (specificSkipNaN != null)
                {
                    if (specificSkipNaN.ContainsKey(argument))
                    {
                        skipNaN = specificSkipNaN[argument];
                    }
                }

                var subjectValue = rearrangedEquation.SubjectValue;

                if (skipNaN && double.IsNaN(subjectValue)) continue;

                subjectValue.Should().BeInRange(argument.Value - halfOfRangeWidth, argument.Value + halfOfRangeWidth,
                    $" it is the argument {argument.GetType()} value");
            }
        }

    }
}
