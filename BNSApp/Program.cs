using System;

namespace BNSApp
{
    class Program
    {
        private const string InputDirectory = "./input_data/input_data_original";
        private const string OutputDirectory = "./TestOutput";
        
        private static readonly MotionContainer TrainData = new MotionContainer($"{InputDirectory}/train/train.csv");
        private static readonly MotionContainer EasyData = new MotionContainer($"{InputDirectory}/test/test_easy.csv", 5);
        private static readonly MotionContainer NormalData = new MotionContainer($"{InputDirectory}/test/test_normal.csv", 15);
        private static readonly MotionContainer HardData = new MotionContainer($"{InputDirectory}/test/test_hard.csv", 45);

        private static void SolveOneTestFile(MotionContainer testData, MotionContainer trainData, ISolver solver,
            string outputFileName)
        {
            foreach (var motionData in testData.MotionList)
            {
                solver.Solve(motionData, trainData);
                Console.WriteLine($"{motionData.MotionId} Finish {DateTime.Now:yyyy/MM/dd HH:mm:ss}");
            }

            testData.SaveData(outputFileName);
        }
        
        static void Main(string[] args)
        {
            var solver = new TwoFrameInterpolation();
            Console.WriteLine($"Start: {DateTime.Now:yyyy/MM/dd HH:mm:ss}");
            var testData = EasyData;
            var testOutputFileName = Utils.GetOutputFileName(testData, OutputDirectory);
            SolveOneTestFile(testData, TrainData, solver, testOutputFileName);
            Console.Write($"Finish {DateTime.Now:yyyy/MM/dd HH:mm:ss}");
        }
    }
}