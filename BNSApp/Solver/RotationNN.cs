using System.Collections.Generic;

namespace BNSApp.Solver
{
    /// <summary>
    /// ボーンの位置情報ではなく回転を類似度計算で利用する場合
    /// </summary>
    public class RotationNN : ISolver
    {
        public string GetName()
        {
            return "RotationNN";
        }

        /// <summary>
        /// ひとまず現在のフレームだけ利用してみる
        /// </summary>
        /// <param name="motionData"></param>
        /// <param name="trainData"></param>
        public void Solve(MotionData motionData, MotionContainer trainData)
        {
            for (var loop = 0; loop < motionData.Loop; loop++)
            {
                var prevMeasuredId = loop * motionData.SkipFrames + 1;
                var nextMeasuredId = (loop + 1) * motionData.SkipFrames + 1;
                var forwardList = new List<Pose>();
                var backwardList = new List<Pose>();

                // 双方向から補間計算していく
                for (var i = 1; i < motionData.SkipFrames; i++)
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

                for (var i = 1; i < motionData.SkipFrames; i++)
                {
                    var estimatedPos = Merge.LinearBlend(forwardList[i - 1], backwardList[^i],
                        (float)(i - 1) / (motionData.SkipFrames - 2));
                    estimatedPos = forwardList[i - 1];
                    motionData.PosList[prevMeasuredId - 1 + i] = estimatedPos;
                }
            }
        }

        private static Pose Interp(MotionContainer trainData, Pose currentPos, Pose pastPos, int direction, int motionId)
        {
            SearchBestPoseTwoFrame(trainData, currentPos, pastPos, direction, motionId, 
                out var nearestCurrentPose, out var nearestFuturePose);
            var deltaPos = Utils.ComputeDelta(nearestCurrentPose, nearestFuturePose);
            return currentPos.Add(deltaPos);
        }

        private static void SearchBestPoseTwoFrame(MotionContainer trainData, Pose currentPos, Pose pastPos, int direction,
            int motionId, out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var testMotionType = Utils.GetMotionType(motionId);
            var minCost = float.MaxValue;
            var nearestMotionType = Utils.MotionType.Back;
            nearestCurrentPose = new Pose();
            nearestFuturePose = new Pose();
            foreach (var trainMotion in trainData.MotionList)
            {
                for (var i = 0; i < trainMotion.Length; i++)
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
                    var cost = Cost.CalcRotationCost(currentTrainPose, currentPos);// *
                                // Cost.CalcMotionTypeCost(testMotionType, trainMotionType);
                    // cost += Cost.CalcRootCost(pastTrainPose, pastPos) *
                    //          Cost.CalcMotionTypeCost(testMotionType, trainMotionType);

                    if (cost < minCost)
                    {
                        minCost = cost;
                        nearestCurrentPose = currentTrainPose;
                        nearestFuturePose = trainMotion.PosList[i + direction];
                        nearestMotionType = Utils.GetMotionType(trainMotion.MotionId);
                    }
                }
            }
        }
    }
}