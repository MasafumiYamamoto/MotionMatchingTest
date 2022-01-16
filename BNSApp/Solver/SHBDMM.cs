using System;
using System.Collections.Generic;
using BNSApp.Skelton;

namespace BNSApp.Solver
{
    /// <summary>
    /// ボーンを分割して行う履歴情報を多く利用するBDMM
    /// </summary>
    public class SplitHistoricalBDMM : ISolver
    {
        public string GetName()
        {
            return "SplitHistoricalBDMM";
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
            SearchBestLowerPose(trainContainer, pastPoseList, currentPose, futurePose,
                direction, motionId, currentFrameIndex, futureFrameIndex, skipFrames,
                out var nearestCurrentLowerPose, out var nearestFutureLowerPose);
            SearchBestUpperPose(trainContainer, pastPoseList, currentPose, futurePose,
                direction, motionId, currentFrameIndex, futureFrameIndex, skipFrames,
                out var nearestCurrentUpperPose, out var nearestFutureUpperPose);
            
            var lowerPoseDelta = Utils.ComputeDelta(nearestCurrentLowerPose, nearestFutureLowerPose);
            var upperPoseDelta = Utils.ComputeDelta(nearestCurrentUpperPose, nearestFutureUpperPose);
            return Utils.ApplySplitBodyDelta(currentPose, lowerPoseDelta, upperPoseDelta);
        }

        private void SearchBestLowerPose(MotionContainer trainData,
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
                    var cost = Cost.CalcPartPoseCost(currentTrainPose, currentPose, SkeltonData.GetLowerBoneIndexList());
                    cost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                    totalCost += cost;

                    for (var k = 0; k < pastPoseList.Count; k++)
                    {
                        var pastTrainPose = trainMotion.PosList[i - direction * (k + 1)];
                        var pastCost = Cost.CalcPartPoseCost(pastTrainPose, pastPoseList[k], SkeltonData.GetLowerBoneIndexList());
                        pastCost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                        pastCost *= Cost.CalcPastWeight(k, pastPoseList.Count);
                        totalCost += pastCost;
                    }

                    // このまま動いた時の姿勢のエラーを計算
                    var futureTrainPose = trainMotion.PosList[i + futureFrameDelta];
                    var deltaPose = Utils.ComputeDelta(currentTrainPose, futureTrainPose);
                    var tmpFuturePose = currentPose.Add(deltaPose);
                    // 下半身はそのままワールド座標での位置情報を利用する
                    var futureCost = Cost.CalcAbsolutePartPosCost(tmpFuturePose, measuredFuturePose, SkeltonData.GetLowerBoneIndexList());
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

        /// <summary>
        /// 上半身だけを利用して最もコストの良い次の姿勢を検索する
        /// </summary>
        /// <param name="trainData"></param>
        /// <param name="pastPoseList"></param>
        /// <param name="currentPose"></param>
        /// <param name="measuredFuturePose"></param>
        /// <param name="direction"></param>
        /// <param name="motionId"></param>
        /// <param name="currentFrameIndex"></param>
        /// <param name="measuredFutureFrameIndex"></param>
        /// <param name="skipFrames"></param>
        /// <param name="nearestCurrentPose"></param>
        /// <param name="nearestFuturePose"></param>
        private void SearchBestUpperPose(MotionContainer trainData,
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
                    var cost = Cost.CalcPartPoseCost(currentTrainPose, currentPose, SkeltonData.GetUpperBoneIndexList());
                    cost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                    totalCost += cost;

                    for (var k = 0; k < pastPoseList.Count; k++)
                    {
                        var pastTrainPose = trainMotion.PosList[i - direction * (k + 1)];
                        var pastCost = Cost.CalcPartPoseCost(pastTrainPose, pastPoseList[k], SkeltonData.GetUpperBoneIndexList());
                        pastCost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                        pastCost *= Cost.CalcPastWeight(k, pastPoseList.Count);
                        totalCost += pastCost;
                    }

                    // このまま動いた時の姿勢のエラーを計算
                    var futureTrainPose = trainMotion.PosList[i + futureFrameDelta];
                    var deltaPose = Utils.ComputeDelta(currentTrainPose, futureTrainPose);
                    var tmpFuturePose = currentPose.Add(deltaPose);
                    // 下半身は自動でそろっていくので上半身は腰からの相対位置を計算する
                    var futureCost = Cost.CalcPartPoseCost(tmpFuturePose, measuredFuturePose, SkeltonData.GetUpperBoneIndexList());
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