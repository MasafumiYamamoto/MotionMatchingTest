using System;
using System.Collections.Generic;
using System.Numerics;

namespace BNSApp
{
    class Program
    {
        static void LinearInterpolation(MotionData motionData, MotionContainer trainData)
        {
            for (int loop = 0; loop < motionData.Loop; loop++)
            {
                var prevMeasuredId = loop * motionData.SkipFrames + 1;
                var nextMeasuredId = (loop + 1) * motionData.SkipFrames + 1;
                var forwardList = new List<List<Vector3>>();
                var backwardList = new List<List<Vector3>>();
                for (int i = 1; i < motionData.SkipFrames; i++)
                {
                    var currentPos = motionData.PosList[prevMeasuredId - 2 + i];
                    var prevPos = motionData.PosList[prevMeasuredId - 1];
                    var nextPos = motionData.PosList[nextMeasuredId - 1];
                    var delta = nextPos.Minus(currentPos);
                    delta = delta.Slash(motionData.SkipFrames - i);
                    var newPos = currentPos.Add(delta);
                    motionData.PosList[prevMeasuredId - 1 + i] = newPos;
                }
            }
        }


        static void Main(string[] args)
        {
            var inputDirectory = "./input_data/input_data_original/";
            var trainData = new MotionContainer($"{inputDirectory}train/train.csv");
            var easyData = new MotionContainer($"{inputDirectory}test/test_easy.csv", 5);
            var normalData = new MotionContainer($"{inputDirectory}test/test_normal.csv", 15);
            var hardData = new MotionContainer($"{inputDirectory}test/test_hard.csv", 45);
            NNInterpolation.Solve(hardData.MotionList[0], trainData);
            hardData.SaveData("./hardOutTest.csv");
            Console.WriteLine("Hello World!");
        }
    }
}