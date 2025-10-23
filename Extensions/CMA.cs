using DLS.Structures;
using System;
using System.Collections.Generic;
using System.Linq;

namespace DLS.Extensions
{
    public class CMAOptimizationResult
    {
        public double[] BestSolution { get; set; } = null!;
        public double BestFitness { get; set; }
        public int Evaluations { get; set; }
        public List<double> FitnessHistory { get; set; } = new List<double>();
    }
    public class CMAESOptimizer
    {
        private readonly int _dimension;
        private readonly int _lambda;
        private readonly int _mu;
        private readonly double[] _weights;
        private readonly double _muW;
        private readonly double _cs;
        private readonly double _cc;
        private readonly double _c1;
        private readonly double _cmu;
        private readonly double _damps;
        private readonly double _chiN;

        private double[] _mean;
        private double _sigma;
        private double[,] _C;
        private double[] _pc;
        private double[] _ps;
        private double[,] _B;
        private double[] _D;
        private int _countEval;
        private int _eigenEval;
        private readonly Random _random;

        public CMAESOptimizer(int dimension, double[]? initialMean = null, double initialSigma = 1.0, int? populationSize = null, Random? random = null)
        {
            _dimension = dimension;
            _lambda = populationSize ?? 4 + (int)(3 * Math.Log(dimension));
            _mu = _lambda / 2;
            _random = random ?? new Random();

            // Initialize weights for recombination
            _weights = new double[_mu];
            for (int i = 0; i < _mu; i++)
            {
                _weights[i] = Math.Log(_mu + 0.5) - Math.Log(i + 1);
            }
            
            double sumWeights = _weights.Sum();
            for (int i = 0; i < _mu; i++)
            {
                _weights[i] /= sumWeights;
            }
            
            _muW = 1.0 / _weights.Select(w => w * w).Sum();

            // Strategy parameter setting: adaptation
            _cs = (_muW + 2) / (_dimension + _muW + 5);
            _cc = (4 + _muW / _dimension) / (_dimension + 4 + 2 * _muW / _dimension);
            _c1 = 2 / (Math.Pow(_dimension + 1.3, 2) + _muW);
            _cmu = Math.Min(1 - _c1, 2 * (_muW - 2 + 1 / _muW) / (Math.Pow(_dimension + 2, 2) + _muW));
            _damps = 1 + 2 * Math.Max(0, Math.Sqrt((_muW - 1) / (_dimension + 1)) - 1) + _cs;
            _chiN = Math.Sqrt(_dimension) * (1 - 1.0 / (4 * _dimension) + 1.0 / (21 * _dimension * _dimension));

            // Initialize dynamic strategy parameters and constants
            _mean = initialMean?.ToArray() ?? Enumerable.Repeat(0.0, _dimension).ToArray();
            _sigma = initialSigma;
            _pc = new double[_dimension];
            _ps = new double[_dimension];
            
            // Initialize covariance matrix to identity
            _C = new double[_dimension, _dimension];
            _B = new double[_dimension, _dimension];
            _D = new double[_dimension];
            
            for (int i = 0; i < _dimension; i++)
            {
                _C[i, i] = 1.0;
                _B[i, i] = 1.0;
                _D[i] = 1.0;
            }
            
            _countEval = 0;
            _eigenEval = 0;
        }

        public CMAOptimizationResult Optimize(Func<double[], double> objectiveFunction, int maxEvaluations = 10000, double targetFitness = double.NegativeInfinity, bool minimization = true)
        {
            var bestSolution = new double[_dimension];
            var bestFitness = minimization ? double.MaxValue : double.MinValue;
            var fitnessHistory = new List<double>();
            
            while (_countEval < maxEvaluations)
            {
                // Generate population
                var population = GeneratePopulation();
                var fitness = new double[_lambda];
                
                // Evaluate population
                for (int i = 0; i < _lambda; i++)
                {
                    fitness[i] = objectiveFunction(population[i]);
                    _countEval++;
                    
                    // Track best solution
                    bool isBetter = minimization ? fitness[i] < bestFitness : fitness[i] > bestFitness;
                    if (isBetter)
                    {
                        bestFitness = fitness[i];
                        Array.Copy(population[i], bestSolution, _dimension);
                    }
                }
                
                fitnessHistory.Add(bestFitness);
                
                // Check termination criteria
                if ((minimization && bestFitness <= targetFitness) || (!minimization && bestFitness >= targetFitness))
                {
                    break;
                }
                
                // Sort population by fitness
                var indices = Enumerable.Range(0, _lambda)
                    .OrderBy(i => minimization ? fitness[i] : -fitness[i])
                    .ToArray();
                
                // Update distribution parameters
                UpdateDistribution(population, indices);
            }
            
            return new CMAOptimizationResult
            {
                BestSolution = bestSolution,
                BestFitness = bestFitness,
                Evaluations = _countEval,
                FitnessHistory = fitnessHistory
            };
        }

        private double[][] GeneratePopulation()
        {
            // Update eigensystem if necessary
            if (_countEval - _eigenEval > _lambda / (_c1 + _cmu) / _dimension / 10)
            {
                _eigenEval = _countEval;
                UpdateEigensystem();
            }
            
            var population = new double[_lambda][];
            
            for (int i = 0; i < _lambda; i++)
            {
                population[i] = new double[_dimension];
                
                // Generate random vector
                var z = new double[_dimension];
                for (int j = 0; j < _dimension; j++)
                {
                    z[j] = SampleNormal();
                }
                
                // Transform to distribution space
                var scaledZ = VectorElementwiseMultiply(_D, z);
                var y = MatrixVectorMultiply(_B, scaledZ);
                
                for (int j = 0; j < _dimension; j++)
                {
                    population[i][j] = _mean[j] + _sigma * y[j];
                }
            }
            
            return population;
        }

        private void UpdateDistribution(double[][] population, int[] indices)
        {
            // Compute weighted mean of selected population
            var oldMean = (double[])_mean.Clone();
            Array.Fill(_mean, 0.0);
            
            for (int i = 0; i < _mu; i++)
            {
                for (int j = 0; j < _dimension; j++)
                {
                    _mean[j] += _weights[i] * population[indices[i]][j];
                }
            }
            
            // Cumulation: update evolution paths
            var meanShift = VectorSubtract(_mean, oldMean);
            var scaledShift = VectorScalarMultiply(meanShift, 1.0 / _sigma);
            
            // Path for sigma
            var Binv_meanShift = MatrixVectorMultiply(TransposeMatrix(_B), scaledShift);
            for (int i = 0; i < _dimension; i++)
            {
                Binv_meanShift[i] /= _D[i];
            }
            
            for (int i = 0; i < _dimension; i++)
            {
                _ps[i] = (1 - _cs) * _ps[i] + Math.Sqrt(_cs * (2 - _cs) * _muW) * Binv_meanShift[i];
            }
            
            // Path for covariance matrix
            var hsig = VectorNorm(_ps) / Math.Sqrt(1 - Math.Pow(1 - _cs, 2 * _countEval / _lambda)) / _chiN < 1.4 + 2.0 / (_dimension + 1);
            
            for (int i = 0; i < _dimension; i++)
            {
                _pc[i] = (1 - _cc) * _pc[i] + (hsig ? 1.0 : 0.0) * Math.Sqrt(_cc * (2 - _cc) * _muW) * scaledShift[i];
            }
            
            // Adapt covariance matrix C
            var artmp = new double[_dimension, _mu];
            for (int i = 0; i < _mu; i++)
            {
                var diff = VectorSubtract(population[indices[i]], oldMean);
                var scaledDiff = VectorScalarMultiply(diff, 1.0 / _sigma);
                for (int j = 0; j < _dimension; j++)
                {
                    artmp[j, i] = scaledDiff[j];
                }
            }
            
            // Update C
            for (int i = 0; i < _dimension; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    _C[i, j] = (1 - _c1 - _cmu) * _C[i, j] + _c1 * _pc[i] * _pc[j];
                    
                    for (int k = 0; k < _mu; k++)
                    {
                        _C[i, j] += _cmu * _weights[k] * artmp[i, k] * artmp[j, k];
                    }
                    
                    if (i != j)
                        _C[j, i] = _C[i, j];
                }
            }
            
            // Adapt step size sigma
            _sigma *= Math.Exp((_cs / _damps) * (VectorNorm(_ps) / _chiN - 1));
        }

        private void UpdateEigensystem()
        {
            // Simple eigendecomposition - in practice, you'd use a more robust method
            // For now, we'll use a simplified approach
            for (int i = 0; i < _dimension; i++)
            {
                _D[i] = Math.Sqrt(_C[i, i]);
                for (int j = 0; j < _dimension; j++)
                {
                    _B[i, j] = (i == j) ? 1.0 : 0.0;
                }
            }
        }

        private double SampleNormal()
        {
            // Box-Muller transform
            static double NextGaussian(Random random)
            {
                double u1 = 1.0 - random.NextDouble();
                double u2 = 1.0 - random.NextDouble();
                return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
            }
            
            return NextGaussian(_random);
        }

        // Utility methods for vector/matrix operations
        private static double[] VectorScalarMultiply(double[] vector, double scalar)
        {
            return vector.Select(v => v * scalar).ToArray();
        }

        private static double[] VectorElementwiseMultiply(double[] a, double[] b)
        {
            return a.Zip(b, (x, y) => x * y).ToArray();
        }

        private static double[] VectorSubtract(double[] a, double[] b)
        {
            return a.Zip(b, (x, y) => x - y).ToArray();
        }

        private static double VectorNorm(double[] vector)
        {
            return Math.Sqrt(vector.Sum(v => v * v));
        }

        private static double[] MatrixVectorMultiply(double[,] matrix, double[] vector)
        {
            int rows = matrix.GetLength(0);
            int cols = matrix.GetLength(1);
            var result = new double[rows];
            
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    result[i] += matrix[i, j] * vector[j];
                }
            }
            
            return result;
        }

        private static double[,] TransposeMatrix(double[,] matrix)
        {
            int rows = matrix.GetLength(0);
            int cols = matrix.GetLength(1);
            var result = new double[cols, rows];
            
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    result[j, i] = matrix[i, j];
                }
            }
            
            return result;
        }

        public int Evaluations => _countEval;
        public double[] CurrentMean => (double[])_mean.Clone();
        public double CurrentSigma => _sigma;
    }

    public static class CMA
    {
        public static CMAOptimizationResult Optimize(Func<double[], double> objectiveFunction, 
            int dimension, 
            double[]? initialMean = null, 
            double initialSigma = 1.0, 
            int maxEvaluations = 10000,
            bool minimization = true)
        {
            var optimizer = new CMAESOptimizer(dimension, initialMean, initialSigma);
            return optimizer.Optimize(objectiveFunction, maxEvaluations, minimization: minimization);
        }
    }
    
    /*
     * Example usage:
     * 
     * // Define your blackbox function - this example finds the minimum of a sphere function
     * Func<double[], double> sphereFunction = parameters => {
     *     return parameters.Sum(x => x * x); // Simple sphere function: minimize sum of squares
     * };
     * 
     * // Optimize with CMA-ES
     * int dimensions = 5; // Optimize 5 parameters
     * var result = CMA.Optimize(sphereFunction, dimensions, maxEvaluations: 1000);
     * 
     * Console.WriteLine($"Best solution: [{string.Join(", ", result.BestSolution)}]");
     * Console.WriteLine($"Best fitness: {result.BestFitness}");
     * Console.WriteLine($"Evaluations used: {result.Evaluations}");
     * 
     * // Or use the CMAESOptimizer class directly for more control:
     * var optimizer = new CMAESOptimizer(
     *     dimension: 3, 
     *     initialMean: new double[] { 1, 2, 3 }, 
     *     initialSigma: 0.5
     * );
     * 
     * var detailedResult = optimizer.Optimize(sphereFunction, maxEvaluations: 2000);
     */
}