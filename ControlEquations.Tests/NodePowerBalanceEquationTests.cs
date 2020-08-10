using System;
using System.Collections.Generic;
using Xunit;

namespace ControlEquations.Tests
{
    public class NodePowerBalanceEquationTests : ControlEquationTestsParent
    {
        private Random _random = new Random(0);
        public (List<T>, double) GenerateArgumentSet<T>(bool balanced, int n = 20, int min =-10000, int max = 10000) where T : Power, new()
        {
            double error = 0.0;

            var arguments = new List<T>();

            for (int i = 0; i < n; i++)
            {
                var val = _random.NextDouble() * _random.Next(min, max);

                error += val;

                var argument = Argument<T>(val);

                arguments.Add(argument);
            }

            if (balanced)
            {
                var argument = Argument<T>(-error);

                arguments.Add(argument);

                return (arguments, 0.0);
            }
            else
            {
                return (arguments, error);
            }
        }


        [Fact]
        public void ActivePowerBalanceCalculatesCorrectlyBalanced()
        {
            var (equationArguments, error) = GenerateArgumentSet<ActivePower>(balanced: true);

            var controlEquation = new NodePowerBalanceEquation<ActivePower>(equationArguments);

            RearrangeEquationTest(controlEquation);
        }

        [Fact]
        public void ReactivePowerBalanceCalculatesCorrectlyBalanced()
        {
            var (equationArguments, error) = GenerateArgumentSet<ReactivePower>(balanced: true);

            var controlEquation = new NodePowerBalanceEquation<ReactivePower>(equationArguments);

            RearrangeEquationTest(controlEquation);
        }

        [Fact]
        public void ActivePowerBalanceCalculatesCorrectlyBalanced_AllArgumentsAreZeros()
        {
            var (equationArguments, error) = GenerateArgumentSet<ActivePower>(balanced: true, min: 0, max:0) ;

            var controlEquation = new NodePowerBalanceEquation<ActivePower>(equationArguments);

            Assert.Equal(0.0, controlEquation.Error, 1);

            RearrangeEquationTest(controlEquation);
        }

        [Fact]
        public void ReactivePowerBalanceCalculatesCorrectlyBalanced_AllArgumentsAreZeros()
        {
            var (equationArguments, error) = GenerateArgumentSet<ReactivePower>(balanced: true, min: 0, max: 0);

            var controlEquation = new NodePowerBalanceEquation<ReactivePower>(equationArguments);

            Assert.Equal(0.0, controlEquation.Error, 1);

            RearrangeEquationTest(controlEquation);
        }

        [Fact]
        public void ActivePowerBalanceCalculatesCorrectlyUnbalanced()
        {
            var (equationArguments, error) = GenerateArgumentSet<ActivePower>(balanced: false);

            var controlEquation = new NodePowerBalanceEquation<ActivePower>(equationArguments);

            Assert.Equal(error, controlEquation.Error, 10);

        }

        [Fact]
        public void ReactivePowerBalanceCalculatesCorrectlyUnbalanced()
        {
            var (equationArguments, error) = GenerateArgumentSet<ReactivePower>(balanced: false);

            var controlEquation = new NodePowerBalanceEquation<ReactivePower>(equationArguments);

            Assert.Equal(error, controlEquation.Error, 10);

        }
    }
}
