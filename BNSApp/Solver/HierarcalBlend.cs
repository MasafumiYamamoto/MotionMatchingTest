using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace BNSApp.Solver
{
    /// <summary>
    /// 階層的に間を埋めていく
    /// 45->15->5と補間フレーム数を狭めていく
    /// </summary>
    public class HierarchicalBlend : ISolver
    {
        public string GetName()
        {
            return "HNN";
        }
        public void Solve(MotionData testData, MotionContainer trainData)
        {
            for (int loop = 0; loop < testData.Loop; loop++)
            {
                var prevMeasuredId = loop * testData.SkipFrames + 1;
                var nextMeasuredId = (loop + 1) * testData.SkipFrames + 1;
                var forwardList = new List<Pose>();
                var backwardList = new List<Pose>();

                // 双方向から補間計算していく
                for (int i = 1; i < testData.SkipFrames / testData.Delta ; i++)
                {
                    var prevPos = testData.PosList[prevMeasuredId - 1];
                    var nextPos = testData.PosList[nextMeasuredId - 1];
                    var prevPrevPos = testData.PosList[prevMeasuredId - 1];
                    var nextNextPos = testData.PosList[nextMeasuredId - 1];

                    // 補間２フレーム目以降は計算済みの値を利用する
                    if (i > 1)
                    {
                        prevPos = forwardList[^1];
                        nextPos = backwardList[^1];
                        prevPrevPos = forwardList[^1];
                        nextNextPos = backwardList[^1];
                        if (i > 2)
                        {
                            prevPrevPos = forwardList[^2];
                            nextNextPos = backwardList[^2];
                        }
                    }
                    
                    var forwardNearestPos = Interp(trainData, prevPos, prevPrevPos, 1, testData.MotionId, testData.Delta);
                    var backwardNearestPos = Interp(trainData, nextPos, nextNextPos, -1, testData.MotionId, testData.Delta);

                    // 前後方向から求めた値を溜めておく
                    forwardList.Add(forwardNearestPos);
                    backwardList.Add(backwardNearestPos);
                    
                    // 線形補間で求めた値とブレンド
                    // 線形補間で利用する姿勢は前後両方から計算する必要がある点に注意
                    // var prevMeasuredPos = testData.PosList[prevMeasuredId - 1];
                    // var nextMeasuredPos = testData.PosList[nextMeasuredId - 1];
                    // var forwardLinearPos = Merge.LinearBlend(prevMeasuredPos, nextMeasuredPos,
                    //     (float)(i - 1) / (testData.SkipFrames - 2));
                    // var backwardLinearPos = Merge.LinearBlend(prevMeasuredPos, nextMeasuredPos,
                    //     (float)(testData.SkipFrames - i - 1) / (testData.SkipFrames - 2));
                    // // 価が大きいほど線形補間で求めた値を採用するようになる
                    // var linearBlendRatio = Distribution.Sample(i, testData.SkipFrames) +
                    //                        Distribution.Sample(testData.SkipFrames - i, testData.SkipFrames);
                    // var forwardBlendPos = Merge.LinearBlend(forwardNearestPos, forwardLinearPos, linearBlendRatio);
                    // var backwardBlendPos = Merge.LinearBlend(backwardNearestPos, backwardLinearPos, linearBlendRatio);
                    //
                    // forwardList.Add(forwardBlendPos);
                    // backwardList.Add(backwardBlendPos);
                }

                for (int i = 1; i < testData.SkipFrames / testData.Delta ; i++)
                {
                    var estimatedPos = Merge.LinearBlend(forwardList[i - 1], backwardList[^i],
                        (float)(i - 1) / (testData.SkipFrames / testData.Delta - 2));
                    testData.PosList[prevMeasuredId - 1 + i*testData.Delta] = estimatedPos;
                }
            }
        }

        Pose Interp(MotionContainer trainData, Pose currentPos, Pose pastPos, int direction, int motionId, int delta)
        {
            SearchBestPoseTwoFrame(trainData, currentPos, pastPos, direction, motionId, delta, 
                out var nearestCurrentPose, out var nearestFuturePose);
            var deltaPos = Utils.ComputeDelta(nearestCurrentPose, nearestFuturePose);
            return currentPos.Add(deltaPos);
        }

        void SearchBestPoseTwoFrame(MotionContainer trainData, Pose currentPos, Pose pastPos, int direction,
            int motionId, int delta, out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var testMotionType = Utils.GetMotionType(motionId);
            var minCost = float.MaxValue;
            var nearestMotionType = Utils.MotionType.Back;
            nearestCurrentPose = new Pose();
            nearestFuturePose = new Pose();
            var deltaWidth = delta * direction;
            foreach (var trainMotion in trainData.MotionList)
            {
                for (int i = 0; i < trainMotion.Length; i++)
                {
                    if (i - deltaWidth < 0 || trainMotion.Length <= i - deltaWidth)
                    {
                        continue;
                    }

                    if (i + deltaWidth < 0 || trainMotion.Length <= i + deltaWidth)
                    {
                        continue;
                    }

                    var currentTrainPose = trainMotion.PosList[i];
                    var trainMotionType = Utils.GetMotionType(trainMotion.MotionId);
                    var pastTrainPose = trainMotion.PosList[i - deltaWidth];
                    var cost = Cost.CalcRootCost(currentTrainPose, currentPos) *
                               Cost.CalcMotionTypeCost(testMotionType, trainMotionType);
                    cost += Cost.CalcRootCost(pastTrainPose, pastPos) *
                            Cost.CalcMotionTypeCost(testMotionType, trainMotionType);

                    if (cost < minCost)
                    {
                        minCost = cost;
                        nearestCurrentPose = currentTrainPose;
                        nearestFuturePose = trainMotion.PosList[i + deltaWidth];
                        nearestMotionType = Utils.GetMotionType(trainMotion.MotionId);
                    }
                }
            }

            // Console.WriteLine($"{Utils.GetMotionType(motionId)}, {nearestMotionType}, {minCost}");
        }
    }
}