using System;
using BNSApp.Solver;

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
        
        public enum RunType
        {
            OneMotion,
            OneFile,
            All
        }

        static void Main(string[] args)
        {
            var trainData = new MotionContainer($"{InputDirectory}/train/train.csv");
            var easyData = new MotionContainer($"{InputDirectory}/test/test_easy.csv", 5);
            var normalData = new MotionContainer($"{InputDirectory}/test/test_normal.csv", 15);
            var hardData = new MotionContainer($"{InputDirectory}/test/test_hard.csv", 45);
            var startTime = DateTime.Now;
            Console.WriteLine($"Start: {startTime:yyyy/MM/dd HH:mm:ss}");
            
            ISolver solver = new HistoricalBDMM();
            var type = RunType.OneFile;
            var testData = normalData;
            switch (type)
            {
                case RunType.OneMotion: // 特定のモーションだけを動かす場合
                    var id = 98;
                    SolveOneMotionById(testData, id, trainData, solver);
                    break;
                case RunType.OneFile: // １ファイルまとめて処理するならここ
                    SolveOneTestFile(testData, trainData, solver);
                    break;
                case RunType.All: // 全部まとめて処理するならここ
                    RunAll(solver, trainData, easyData, normalData, hardData);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
            
            var finishTime = DateTime.Now;
            Console.WriteLine($"Finish {finishTime:yyyy/MM/dd HH:mm:ss}");
            Console.WriteLine($"Total Running Time: {finishTime-startTime}");
        }
    }
}