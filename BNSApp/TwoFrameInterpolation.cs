using System;
using System.Collections.Generic;

namespace BNSApp
{
    /// <summary>
    /// 2フレーム分の情報を利用して補間を行う
    /// </summary>
    public class TwoFrameInterpolation : ISolver
    {
        public string GetName()
        {
            return "TwoNN";
        }
        public void Solve(MotionData motionData, MotionContainer trainData)
        {
            for (int loop = 0; loop < motionData.Loop; loop++)
            {
                var prevMeasuredId = loop * motionData.SkipFrames + 1;
                var nextMeasuredId = (loop + 1) * motionData.SkipFrames + 1;
                var forwardList = new List<Pose>();
                var backwardList = new List<Pose>();

                // 双方向から補間計算していく
                for (int i = 1; i < motionData.SkipFrames; i++)
                {
                    var prevPos = motionData.PosList[prevMeasuredId - 1];
                    var nextPos = motionData.PosList[nextMeasuredId - 1];
                    var prevPrevPos = motionData.PosList[prevMeasuredId - 1];
                    var nextNextPos = motionData.PosList[nextMeasuredId - 1];

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

                    var forwardPos = Interp(trainData, prevPos, prevPrevPos, 1, motionData.MotionId);
                    var backwardPos = Interp(trainData, nextPos, nextNextPos, -1, motionData.MotionId);
                    forwardList.Add(forwardPos);
                    backwardList.Add(backwardPos);
                }

                for (int i = 1; i < motionData.SkipFrames; i++)
                {
                    var pos = Merge.LinearBlend(forwardList[i - 1], backwardList[^i],
                        (float)(i - 1) / (motionData.SkipFrames - 2));
                    motionData.PosList[prevMeasuredId - 1 + i] = pos;
                }
            }
        }

        Pose Interp(MotionContainer trainData, Pose currentPos, Pose pastPos, int direction, int motionId)
        {
            SearchBestPoseTwoFrame(trainData, currentPos, pastPos, direction, motionId, 
                out var nearestCurrentPose, out var nearestFuturePose);
            var deltaPos = Utils.ComputeDelta(nearestCurrentPose, nearestFuturePose);
            return currentPos.Add(deltaPos);
        }

        void SearchBestPoseTwoFrame(MotionContainer trainData, Pose currentPos, Pose pastPos, int direction,
            int motionId, out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var testMotionType = Utils.GetMotionType(motionId);
            var minCost = float.MaxValue;
            var nearestMotionType = Utils.MotionType.Back;
            nearestCurrentPose = new Pose();
            nearestFuturePose = new Pose();
            foreach (var trainMotion in trainData.MotionList)
            {
                for (int i = 0; i < trainMotion.Length; i++)
                {
                    if (i - direction < 0 || trainMotion.Length <= i - direction)
                    {
                        continue;
                    }

                    if (i + direction < 0 || trainMotion.Length <= i + direction)
                    {
                        continue;
                    }

                    var currentTrainPose = trainMotion.PosList[i];
                    var trainMotionType = Utils.GetMotionType(trainMotion.MotionId);
                    var pastTrainPose = trainMotion.PosList[i - direction];
                    var cost = Cost.CalcRootCost(currentTrainPose, currentPos) *
                               Cost.CalcMotionTypeCost(testMotionType, trainMotionType);
                    cost += 0.5f * Cost.CalcRootCost(pastTrainPose, pastPos) *
                            Cost.CalcMotionTypeCost(testMotionType, trainMotionType);

                    if (cost < minCost)
                    {
                        minCost = cost;
                        nearestCurrentPose = currentTrainPose;
                        nearestFuturePose = trainMotion.PosList[i + direction];
                        nearestMotionType = Utils.GetMotionType(trainMotion.MotionId);
                    }
                }
            }

            Console.WriteLine($"{minCost}, {Utils.GetMotionType(motionId)}, {nearestMotionType}");
        }
    }
}