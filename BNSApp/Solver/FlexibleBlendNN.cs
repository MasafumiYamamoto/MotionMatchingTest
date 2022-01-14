using System.Collections.Generic;

namespace BNSApp.Solver
{
    /// <summary>
    /// １フレだけでなく複数フレームの幅を考慮したブレンド
    /// </summary>
    public class FlexibleBlendNN : ISolver
    {
        public string GetName()
        {
            return "FlexibleNN";
        }

        public void Solve(MotionData testData, MotionContainer trainData)
        {
            for (var loop = 0; loop < testData.Loop; loop++)
            {
                var prevMeasuredId = loop * testData.SkipFrames + 1;
                var nextMeasuredId = (loop + 1) * testData.SkipFrames + 1;
                var forwardList = new List<Pose>();
                var backwardList = new List<Pose>();

                // 双方向から補間計算していく
                for (var i = 1; i < testData.SkipFrames; i++)
                {
                    var prevPos = testData.PosList[prevMeasuredId - 1];
                    var prevPastPos = testData.PosList[prevMeasuredId - 1];
                    var nextPos = testData.PosList[nextMeasuredId - 1];
                    var nextPastPos = testData.PosList[nextMeasuredId - 1];

                    var forwardPos = new Pose();
                    var backwardPos = new Pose();

                    switch (i)
                    {
                        case 1:
                            forwardPos =InterpFirstFrame(trainData, prevPos, 1, testData.MotionId);
                            backwardPos = InterpFirstFrame(trainData, nextPos, -1, testData.MotionId);
                            break;
                        // 補間２フレーム目以降は計算済みの値を利用する
                        case > 1:
                        {
                            prevPos = forwardList[^1];
                            nextPos = backwardList[^1];
                            if (i > 2)
                            {
                                prevPastPos = forwardList[^2];
                                nextPastPos = backwardList[^2];
                            }
                            forwardPos = Interp(trainData, prevPos, prevPastPos, testData.MotionId);
                            backwardPos = Interp(trainData, nextPos, nextPastPos, testData.MotionId);

                            break;
                        }
                    }

                    forwardList.Add(forwardPos);
                    backwardList.Add(backwardPos);
                }

                for (var i = 1; i < testData.SkipFrames; i++)
                {
                    var estimatedPos = Merge.LinearBlend(forwardList[i - 1], backwardList[^i],
                        (float)(i - 1) / (testData.SkipFrames - 2));
                    testData.PosList[prevMeasuredId - 1 + i] = estimatedPos;
                }
            }
        }

        Pose InterpFirstFrame(MotionContainer trainData, Pose currentPos, int direction, int motionId)
        {
            SearchBestPose(trainData, currentPos, direction, motionId,
                out var nearestCurrentPose, out var nearestFuturePose);
            var deltaPos = Utils.ComputeDelta(nearestCurrentPose, nearestFuturePose);
            return currentPos.Add(deltaPos);
        }

        Pose Interp(MotionContainer trainData, Pose currentPos, Pose pastPos, int motionId)
        {
            SearchBestPoseTwoFrame(trainData, currentPos, pastPos, motionId,
                out var nearestCurrentPose, out var nearestFuturePose);
            var deltaPos = Utils.ComputeDelta(nearestCurrentPose, nearestFuturePose);
            return currentPos.Add(deltaPos);
        }

        void SearchBestPoseTwoFrame(MotionContainer trainData, Pose currentPos, Pose pastPos,
            int motionId, out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var testMotionType = Utils.GetMotionType(motionId);
            var minCost = float.MaxValue;
            var nearestMotionType = Utils.MotionType.Back;
            nearestCurrentPose = new Pose();
            nearestFuturePose = new Pose();
            for (var direction = -3; direction <= 3; direction++)
            {
                if (direction == 0)
                {
                    continue;
                }

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
                        var cost = Cost.CalcRootCost(currentTrainPose, currentPos) *
                                   Cost.CalcMotionTypeCost(testMotionType, trainMotionType);
                        cost += Cost.CalcRootCost(pastTrainPose, pastPos) *
                                Cost.CalcMotionTypeCost(testMotionType, trainMotionType);

                        if (cost < minCost)
                        {
                            minCost = cost;
                            nearestCurrentPose = currentTrainPose;
                            nearestFuturePose = trainMotion.PosList[i + direction];
                            // ルートだけは１フレ分の移動を利用する
                            nearestFuturePose.Joints[0] = trainMotion.PosList[i + (direction > 0 ? 1 : -1)].Joints[0];
                            nearestMotionType = Utils.GetMotionType(trainMotion.MotionId);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 最も近いポーズとその時の次のフレームの値を返す
        /// </summary>
        /// <param name="trainData"></param>
        /// <param name="currentPos"></param>
        /// <param name="direction"></param>
        /// <param name="motionId"></param>
        /// <param name="nearestCurrentPose"></param>
        /// <param name="nearestFuturePose"></param>
        void SearchBestPose(MotionContainer trainData, Pose currentPos, int direction,
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
                    if (i + direction < 0 || trainMotion.Length <= i + direction)
                    {
                        continue;
                    }

                    var currentTrainPose = trainMotion.PosList[i];
                    var trainMotionType = Utils.GetMotionType(trainMotion.MotionId);
                    var cost = Cost.CalcRootCost(currentTrainPose, currentPos) *
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
        }
    }
}