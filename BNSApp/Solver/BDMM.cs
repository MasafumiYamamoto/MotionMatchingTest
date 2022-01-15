using System.Collections.Generic;

namespace BNSApp.Solver
{
    /// <summary>
    /// BiDirectionalMotionMatching
    /// </summary>
    public class BDMM : ISolver
    {
        public string GetName()
        {
            return "BDMM";
        }

        public void Solve(MotionData testData, MotionContainer trainContainer)
        {
            // 順方向に計算した結果
            var forwardList = new List<Pose>();

            forwardList.Add(testData.PosList[0]);
            // １フレーム目はどうしようもないので最近傍探索
            forwardList.Add(Utils.FindNextFrame(trainContainer, testData.PosList[0], 1, testData.MotionId));

            // 順方向
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
                var pastPose = forwardList[^2];
                var currentPose = forwardList[^1];
                var measuredFuturePose = testData.PosList[nextMeasuredIndex];
                var nextPose = FindNextPose(trainContainer,
                    pastPose, currentPose, measuredFuturePose,
                    1, testData.MotionId, i, nextMeasuredIndex, testData.SkipFrames);
                forwardList.Add(nextPose);
            }

            // 逆方向に計算した結果
            var backwardList = new List<Pose>();
            backwardList.Add(testData.PosList[^1]);
            // 同じく１フレーム目は最近傍探索
            backwardList.Add(Utils.FindNextFrame(trainContainer, testData.PosList[^1], -1, testData.MotionId));

            // 逆方向
            for (var i = testData.Length - 2; i > 0; i--)
            {
                var nextMeasuredIndex = (i / testData.SkipFrames) * testData.SkipFrames;

                if (i - 1 == nextMeasuredIndex)
                {
                    // 既に計測済みの値がある場合はそちらを採用
                    backwardList.Add(testData.PosList[i - 1]);
                    continue;
                }

                // 補間対象ならいつも通り計算
                var pastPose = backwardList[^2];
                var currentPose = backwardList[^1];
                var measuredFuturePose = testData.PosList[nextMeasuredIndex];
                var nextPose = FindNextPose(trainContainer,
                    pastPose, currentPose, measuredFuturePose,
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

        private Pose FindNextPose(MotionContainer trainContainer, Pose pastPose, Pose currentPose, Pose futurePose,
            int direction, int motionId, int currentFrameIndex, int futureFrameIndex, int skipFrames)
        {
            SearchBestPose(trainContainer, pastPose, currentPose, futurePose,
                direction, motionId, currentFrameIndex, futureFrameIndex, skipFrames,
                out var nearestCurrentPose, out var nearestFuturePose);
            var poseDelta = Utils.ComputeDelta(nearestCurrentPose, nearestFuturePose);
            return currentPose.Add(poseDelta);
        }

        private void SearchBestPose(MotionContainer trainData,
            Pose pastPose, Pose currentPose, Pose measuredFuturePose,
            int direction, int motionId, int currentFrameIndex, int measuredFutureFrameIndex, int skipFrames,
            out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var testMotionType = Utils.GetMotionType(motionId);
            var minCost = float.MaxValue;
            nearestCurrentPose = new Pose();
            nearestFuturePose = new Pose();

            foreach (var trainMotion in trainData.MotionList)
            {
                if (trainMotion.MotionId is 95 or 112)
                {
                    // モーションが特殊すぎるのでいったん外してみる
                    continue;
                }
                var trainMotionType = Utils.GetMotionType(trainMotion.MotionId);
                for (var i = 1; i < trainMotion.Length - 1; i++)
                {
                    var futureFrameDelta = measuredFutureFrameIndex - currentFrameIndex;
                    if (i + futureFrameDelta >= trainMotion.Length || i + futureFrameDelta < 0)
                    {
                        // 将来を考えて使えない候補は利用しない
                        continue;
                    }

                    var currentTrainPose = trainMotion.PosList[i];
                    var pastTrainPose = trainMotion.PosList[i - direction];
                    var futureTrainPose = trainMotion.PosList[i + futureFrameDelta];

                    var totalCost = 0f;
                    var cost = Cost.CalcRootCost(currentTrainPose, currentPose);
                    cost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                    totalCost += cost;

                    var pastCost = Cost.CalcRootCost(pastTrainPose, pastPose);
                    pastCost *= Cost.CalcMotionTypeCost(trainMotionType, testMotionType);
                    totalCost += pastCost;

                    // このまま動いた時の姿勢のエラー
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
                }
            }
        }
    }
}