using System;
using System.Diagnostics;
using BNSApp.Skelton;

namespace BNSApp.Solver
{
    /// <summary>
    /// 前方向からのMotionMatchingをメインで使いつつ、次の計測地点との誤差をいい感じに考慮していくMotionMatching
    /// </summary>
    public class ForwardMM : ISolver
    {
        public string GetName()
        {
            return "ForwardMM";
        }

        public void Solve(MotionData testData, MotionContainer trainContainer)
        {
            // １フレーム目はどうしようもないのでそのまま最近傍探索
            testData.PosList[1] = Utils.FindNextFrame(trainContainer, testData.PosList[0], 1, testData.MotionId);
            testData.PosList[2] = Utils.FindNextFrame(trainContainer, testData.PosList[1], 1, testData.MotionId);

            // これまでのLoop単位ではなく、全体を1本で計算していく
            for (var i = 2; i < testData.Length - 1; i++)
            {
                var prevMeasuredIndex = (i / testData.SkipFrames) * testData.SkipFrames;
                var nextMeasuredIndex = (i / testData.SkipFrames + 1) * testData.SkipFrames;

                if (i + 1 == nextMeasuredIndex)
                {
                    continue;
                }

                var pastPose = testData.PosList[i - 1];
                var pastPose2 = testData.PosList[i - 2];
                var currentPose = testData.PosList[i];
                var measuredFuturePose = testData.PosList[nextMeasuredIndex];
                var nextPose = FindNextPose(trainContainer,
                    pastPose2, pastPose, currentPose, measuredFuturePose,
                    testData.MotionId, i, nextMeasuredIndex);
                testData.PosList[i + 1] = nextPose;
            }
        }

        private Pose FindNextPose(MotionContainer trainContainer,
            Pose pastPose2, Pose pastPose, Pose currentPose, Pose futurePose,
            int motionId, int currentFrameIndex, int futureFrameIndex)
        {
            SearchBestPose(trainContainer,
                pastPose2, pastPose, currentPose, futurePose,
                motionId, currentFrameIndex, futureFrameIndex,
                out var nearestCurrentPose, out var nearestFuturePose);
            var poseDelta = Utils.ComputeDelta(nearestCurrentPose, nearestFuturePose);
            return currentPose.Add(poseDelta);
        }

        private void SearchBestPose(MotionContainer trainData,
            Pose pastPose2, Pose pastPose, Pose currentPose, Pose futurePose,
            int motionId, int currentFrameIndex, int futureFrameIndex,
            out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var testMotionType = Utils.GetMotionType(motionId);
            var minCost = float.MaxValue;
            nearestCurrentPose = new Pose();
            nearestFuturePose = new Pose();
            var minMotionId = 99;

            foreach (var trainMotion in trainData.MotionList)
            {
                if (trainMotion.MotionId is 95 or 112)
                {
                    // モーションが特殊すぎるのでいったん外してみる
                    continue;
                }
                var trainMotionType = Utils.GetMotionType(trainMotion.MotionId);
                for (var i = 2; i < trainMotion.Length - 1; i++)
                {
                    var futureFrameDelta = futureFrameIndex - currentFrameIndex;
                    // 将来を考えて使える候補に絞る
                    if (i + futureFrameDelta >= trainMotion.Length)
                    {
                        continue;
                    }

                    var currentTrainPose = trainMotion.PosList[i];
                    var pastTrainPose = trainMotion.PosList[i - 1];
                    var pastTrainPose2 = trainMotion.PosList[i - 2];
                    var futureTrainPose = trainMotion.PosList[i + futureFrameDelta];

                    var totalCost = 0f;
                    var cost = Cost.CalcRootCost(currentTrainPose, currentPose);
                    cost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                    totalCost += cost;

                    var pastCost = Cost.CalcRootCost(pastTrainPose, pastPose);
                    pastCost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                    totalCost += pastCost;

                    var pastCost2 = Cost.CalcRootCost(pastTrainPose2, pastPose2);
                    pastCost2 *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                    totalCost += pastCost2;

                    // このまま動いた時の姿勢のエラー
                    var deltaPose = Utils.ComputeDelta(currentTrainPose, futureTrainPose);
                    var tmpFuturePose = currentPose.Add(deltaPose);
                    var futureCost = Cost.CalcAbsolutePosCost(tmpFuturePose, futurePose);
                    futureCost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                    futureCost *= Cost.CalcFrameCost(currentFrameIndex, futureFrameIndex);

                    totalCost += futureCost;

                    if (totalCost >= minCost)
                    {
                        continue;
                    }

                    minCost = totalCost;
                    nearestCurrentPose = currentTrainPose;
                    nearestFuturePose = trainMotion.PosList[i + 1];
                    minMotionId = trainMotion.MotionId;
                }
            }
            
            // Console.WriteLine($"MinMotionId {minMotionId}");
        }
    }
}