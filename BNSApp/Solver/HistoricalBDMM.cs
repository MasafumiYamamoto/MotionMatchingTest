using System;
using System.Collections.Generic;

namespace BNSApp.Solver
{
    /// <summary>
    /// 履歴情報を多く利用するBDMM
    /// </summary>
    public class HistoricalBDMM : ISolver
    {
        public string GetName()
        {
            return "HBDMM";
        }

        public void Solve(MotionData testData, MotionContainer trainContainer)
        {
            var historySize = 5;

            // 順方向の結果を保存する変数
            var forwardList = new List<Pose>();
            // 合計５フレーム分履歴情報として利用したいのでデータを別途計算しておく
            forwardList.Add(testData.PosList[0]);
            // １フレーム目はどうしようもないので最近傍探索
            forwardList.Add(Utils.FindNextFrame(trainContainer, forwardList[^1], 1, testData.MotionId));

            // 順方向に補間を埋めていく
            for (var i = 1; i < testData.Length - 1; i++)
            {
                var nextMeasuredIndex = (i / testData.SkipFrames + 1) * testData.SkipFrames;

                if (i + 1 == nextMeasuredIndex)
                {
                    // 既に計測済みの値がある場合はそちらを採用
                    forwardList.Add(testData.PosList[i + 1]);
                    continue;
                }

                // 補間対象ならいつも通り計算
                var currentPose = forwardList[^1];
                var pastPoseList = new List<Pose>();
                for (var k = 0; k < MathF.Min(i, historySize - 1); k++)
                {
                    pastPoseList.Add(forwardList[^(k + 2)]);
                }

                var measuredFuturePose = testData.PosList[nextMeasuredIndex];
                var nextPose = FindNextPose(trainContainer,
                    pastPoseList, currentPose, measuredFuturePose,
                    1, testData.MotionId, i, nextMeasuredIndex, testData.SkipFrames);
                forwardList.Add(nextPose);
            }

            // 逆方向に結果を保存する変数
            var backwardList = new List<Pose>();

            // 合計５フレーム分履歴情報として利用したい
            backwardList.Add(testData.PosList[^1]);
            backwardList.Add(Utils.FindNextFrame(trainContainer, backwardList[^1], -1, testData.MotionId));

            // 逆方向に補間を埋めていく
            for (var i = testData.Length - 2; i > 0; i--)
            {
                var nextMeasuredIndex = (i / testData.SkipFrames) * testData.SkipFrames;

                if (i - 1 == nextMeasuredIndex)
                {
                    // 既に計測済みの値がある場合はそちらを採用
                    backwardList.Add(testData.PosList[nextMeasuredIndex]);
                    continue;
                }

                // 補間対象ならいつも通り計算
                var currentPose = backwardList[^1];
                var pastPoseList = new List<Pose>();
                for (var k = 0; k < MathF.Min(testData.Length - 1 - i, historySize - 1); k++)
                {
                    pastPoseList.Add(backwardList[^(k + 2)]);
                }

                var measuredFuturePose = testData.PosList[nextMeasuredIndex];
                var nextPose = FindNextPose(trainContainer,
                    pastPoseList, currentPose, measuredFuturePose,
                    -1, testData.MotionId, i, nextMeasuredIndex, testData.SkipFrames);
                backwardList.Add(nextPose);
            }

            // 双方向から計算した結果をマージ
            for (var i = 0; i < testData.Length; i++)
            {
                float ratio = i % testData.SkipFrames;
                ratio /= testData.SkipFrames;
                var estimatedPos = Merge.SmoothStepBlend(forwardList[i], backwardList[^(i + 1)], ratio);
                testData.PosList[i] = estimatedPos;
            }
        }

        private Pose FindNextPose(MotionContainer trainContainer, List<Pose> pastPoseList, Pose currentPose,
            Pose futurePose,
            int direction, int motionId, int currentFrameIndex, int futureFrameIndex, int skipFrames)
        {
            SearchBestPose(trainContainer, pastPoseList, currentPose, futurePose,
                direction, motionId, currentFrameIndex, futureFrameIndex, skipFrames,
                out var nearestCurrentPose, out var nearestFuturePose);
            var poseDelta = Utils.ComputeDelta(nearestCurrentPose, nearestFuturePose);
            return currentPose.Add(poseDelta);
        }

        private void SearchBestPose(MotionContainer trainData,
            List<Pose> pastPoseList, Pose currentPose, Pose measuredFuturePose,
            int direction, int motionId, int currentFrameIndex, int measuredFutureFrameIndex, int skipFrames,
            out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var testMotionType = Utils.GetMotionType(motionId);
            var minCost = float.MaxValue;
            nearestCurrentPose = new Pose();
            nearestFuturePose = new Pose();
            var minMotionId = 000;

            foreach (var trainMotion in trainData.MotionList)
            {
                if (trainMotion.MotionId is 95 or 112)
                {
                    // モーションが特殊すぎるのでいったん外してみる
                    continue;
                }

                var trainMotionType = Utils.GetMotionType(trainMotion.MotionId);
                for (var i = pastPoseList.Count; i < trainMotion.Length - pastPoseList.Count; i++)
                {
                    var futureFrameDelta = measuredFutureFrameIndex - currentFrameIndex;
                    if (i + futureFrameDelta >= trainMotion.Length || i + futureFrameDelta < 0)
                    {
                        // 将来を考えて使えない候補は利用しない
                        continue;
                    }

                    // 最終的なコスト
                    var totalCost = 0f;

                    var currentTrainPose = trainMotion.PosList[i];
                    var cost = Cost.CalcRootCost(currentTrainPose, currentPose);
                    cost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                    totalCost += cost;

                    for (var k = 0; k < pastPoseList.Count; k++)
                    {
                        var pastTrainPose = trainMotion.PosList[i - direction * (k + 1)];
                        var pastCost = Cost.CalcRootCost(pastTrainPose, pastPoseList[k]);
                        pastCost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                        pastCost *= Cost.CalcPastWeight(k, pastPoseList.Count);
                        totalCost += pastCost;
                    }

                    // このまま動いた時の姿勢のエラーを計算
                    var futureTrainPose = trainMotion.PosList[i + futureFrameDelta];
                    var deltaPose = Utils.ComputeDelta(currentTrainPose, futureTrainPose);
                    var tmpFuturePose = currentPose.Add(deltaPose);
                    var futureCost = Cost.CalcAbsolutePosCost(tmpFuturePose, measuredFuturePose);
                    futureCost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                    futureCost *= Cost.CalcFrameCost(currentFrameIndex, measuredFutureFrameIndex, skipFrames);
                    totalCost += futureCost;

                    if (totalCost >= minCost)
                    {
                        continue;
                    }

                    minCost = totalCost;
                    nearestCurrentPose = currentTrainPose;
                    nearestFuturePose = trainMotion.PosList[i + direction];
                    minMotionId = trainMotion.MotionId;
                }
            }
            // Console.WriteLine($"{direction} {currentFrameIndex} {minMotionId}");
        }
    }
}