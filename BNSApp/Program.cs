using System;

namespace BNSApp
{
    class Program
    {
        private const string InputDirectory = "./input_data/input_data_original";
        private const string OutputDirectory = "./TestOutput";

        private static void RunAll(ISolver solver, MotionContainer trainData,
            MotionContainer easyTest, MotionContainer normalTest, MotionContainer hardTest)
        {
            SolveOneTestFile(easyTest, trainData, solver);
            SolveOneTestFile(normalTest, trainData, solver);
            SolveOneTestFile(hardTest, trainData, solver);
        }

        private static void SolveOneTestFile(MotionContainer testData, MotionContainer trainData, ISolver solver)
        {
            var outputFileName = Utils.GetOutputFileName(testData, OutputDirectory, solver.GetName());
            foreach (var testMotion in testData.MotionList)
            {
                SolveOneMotion(testMotion, trainData, solver);
            }

            testData.SaveData(outputFileName);
        }

        private static void SolveOneMotion(MotionData testMotion, MotionContainer trainData, ISolver solver)
        {
            solver.Solve(testMotion, trainData);
            Console.WriteLine($"{testMotion.MotionId} Finish {DateTime.Now:yyyy/MM/dd HH:mm:ss}");
        }

        private static void SolveOneMotionById(MotionContainer testData, int motionId, MotionContainer trainData,
            ISolver solver)
        {
            var testMotion = testData.GetMotionById(motionId);
            var testOutputFileName = Utils.GetOutputFileName(testData, OutputDirectory, solver.GetName());
            SolveOneMotion(testMotion, trainData, solver);
            testData.SaveData(testOutputFileName);
        }

        static void Main(string[] args)
        {
            var trainData = new MotionContainer($"{InputDirectory}/train/train.csv");
            var easyData = new MotionContainer($"{InputDirectory}/test/test_easy.csv", 5);
            var normalData = new MotionContainer($"{InputDirectory}/test/test_normal.csv", 15);
            var hardData = new MotionContainer($"{InputDirectory}/test/test_hard.csv", 45);
            
            var solver = new TwoFrameInterpolation();
            var startTime = DateTime.Now;
            Console.WriteLine($"Start: {startTime:yyyy/MM/dd HH:mm:ss}");

            // 特定のモーションだけ動かすならここ
            if (true)
            {
                var testData = normalData;
                var id = 122;
                SolveOneMotionById(testData, id, trainData, solver);
            }

            // １ファイルまとめて処理するならここ
            if (false)
            {
                var testData = normalData;
                SolveOneTestFile(testData, trainData, solver);
            }

            // 全部まとめて処理するならここ
            if (false)
            {
                RunAll(solver, trainData, easyData, normalData, hardData);
            }

            var finishTime = DateTime.Now;
            Console.WriteLine($"Finish {finishTime:yyyy/MM/dd HH:mm:ss}");
            Console.WriteLine($"Total Running Time: {finishTime-startTime}");
        }
    }
}