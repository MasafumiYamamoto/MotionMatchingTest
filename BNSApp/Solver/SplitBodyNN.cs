using System;
using System.Collections.Generic;
using System.Numerics;
using BNSApp.Skelton;

namespace BNSApp.Solver
{
    /// <summary>
    /// 上半身と下半身でそれぞれ独立して最近傍探索を行う場合
    /// </summary>
    public class SplitBodyNN : ISolver
    {
        public string GetName()
        {
            return "TwoBody";
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
                    var nextPos = testData.PosList[nextMeasuredId - 1];
                    var prevPastPos = testData.PosList[prevMeasuredId - 1];
                    var nextPastPos = testData.PosList[nextMeasuredId - 1];

                    // 補間２フレーム目以降は計算済みの直近の値を利用する
                    if (i > 1)
                    {
                        prevPos = forwardList[^1];
                        nextPos = backwardList[^1];
                        prevPastPos = forwardList[^1];
                        nextPastPos = backwardList[^1];
                        if (i > 2)
                        {
                            prevPastPos = forwardList[^2];
                            nextPastPos = backwardList[^2];
                        }
                    }
                    
                    // 原点の移動を無視した差分を追加したものがここで作られる
                    var forwardPos = Interp(trainData, prevPos, prevPastPos, 1, testData.MotionId);
                    var backwardPos = Interp(trainData, nextPos, nextPastPos,  -1, testData.MotionId);

                    // Rootだけは動かしてないので別途個別に補間する必要がある
                    // ひとまずRootは線形補間してみる
                    var pose0 = testData.PosList[prevMeasuredId - 1];
                    var pose1 = testData.PosList[nextMeasuredId - 1];
                    var rootForwardDelta = (pose1.Joints[0] - pose0.Joints[0]) / (testData.SkipFrames - 2);
                    var rootBackwardDelta = (pose0.Joints[0] - pose1.Joints[0]) / (testData.SkipFrames - 2);
                    foreach (var index in SkeltonData.BoneNameIndexList.Values)
                    {
                        if (index == 0)
                        {
                            // rootなら線形補間
                            forwardPos.Joints[index] = prevPos.Joints[index] + rootForwardDelta;
                            backwardPos.Joints[index] = nextPos.Joints[index] + rootBackwardDelta;
                        }
                        else
                        {
                            // root以外ならrootの変分を追加する
                            // forwardPos.Joints[index] += rootForwardDelta;
                            // backwardPos.Joints[index] += rootForwardDelta;
                        }
                    }
                    
                    forwardList.Add(forwardPos);
                    backwardList.Add(backwardPos);
                }

                // 前後方向から求めた値を利用して補間
                for (var i = 1; i < testData.SkipFrames; i++)
                {
                    var estimatedPos = Merge.LinearBlend(forwardList[i - 1], backwardList[^i],
                        (float)(i - 1) / (testData.SkipFrames - 2));
                    testData.PosList[prevMeasuredId - 1 + i] = estimatedPos;
                }
            }
        }
        
        /// <summary>
        /// Rootは動かさずにRoot以外を差分を計算する
        /// </summary>
        /// <param name="trainData"></param>
        /// <param name="currentPos"></param>
        /// <param name="pastPos"></param>
        /// <param name="direction"></param>
        /// <param name="motionId"></param>
        /// <returns></returns>
        private Pose Interp(MotionContainer trainData, Pose currentPos, Pose pastPos, int direction, int motionId)
        {
            SearchBestSplitPoseTwoFrame(trainData, currentPos, pastPos, direction, motionId, 
                out var nearestCurrentPose, out var nearestFuturePose);
            var deltaPos = Utils.ComputeDelta(nearestCurrentPose, nearestFuturePose);
            return currentPos.Add(deltaPos);
        }
        
        /// <summary>
        /// 直近2フレームの値を利用して、上半身と下半身それぞれ独立に近いポーズを探す
        /// </summary>
        /// <param name="trainData"></param>
        /// <param name="currentPos"></param>
        /// <param name="pastPos"></param>
        /// <param name="direction"></param>
        /// <param name="motionId"></param>
        /// <param name="nearestCurrentPose"></param>
        /// <param name="nearestFuturePose"></param>
        void SearchBestSplitPoseTwoFrame(MotionContainer trainData, Pose currentPos, Pose pastPos, int direction,
            int motionId, out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var testMotionType = Utils.GetMotionType(motionId);
            var minCostUpperBody = float.MaxValue;
            var minCostLowerBody = float.MaxValue;
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

                    var lowerCost = BNSApp.Cost.CalcPartPoseCost(currentTrainPose, currentPos,
                        SkeltonData.GetLowerBoneIndexList())* Cost.CalcMotionTypeCost(testMotionType, trainMotionType);
                    lowerCost += BNSApp.Cost.CalcPartPoseCost(pastTrainPose, pastPos,
                        SkeltonData.GetLowerBoneIndexList())* Cost.CalcMotionTypeCost(testMotionType, trainMotionType);
                    
                    if (lowerCost < minCostLowerBody)
                    {
                        minCostLowerBody = lowerCost;
                        var futureTrainPose = trainMotion.PosList[i + direction];
                        UpdatePosePart(nearestCurrentPose, currentTrainPose, SkeltonData.GetLowerBoneIndexList());
                        UpdatePosePart(nearestFuturePose, futureTrainPose, SkeltonData.GetLowerBoneIndexList());
                    }
                    
                    var upperCost = Cost.CalcPartPoseCost(currentTrainPose, currentPos,
                        SkeltonData.GetUpperBoneIndexList()) * Cost.CalcMotionTypeCost(testMotionType, trainMotionType);
                    // 1フレ前の情報も考慮
                    upperCost += Cost.CalcPartPoseCost(pastTrainPose, pastPos,
                        SkeltonData.GetUpperBoneIndexList()) * Cost.CalcMotionTypeCost(testMotionType, trainMotionType);
                    if (upperCost < minCostUpperBody)
                    {
                        minCostUpperBody = upperCost;
                        var futureTrainPose = trainMotion.PosList[i + direction];
                        UpdateUpperPart(nearestCurrentPose, currentTrainPose, SkeltonData.GetUpperBoneIndexList());
                        UpdateUpperPart(nearestFuturePose, futureTrainPose, SkeltonData.GetUpperBoneIndexList());
                    }
                }
            }
        }

        private void UpdatePosePart(Pose dstPose, Pose srcPose, List<int> indexList)
        {
            // Root基準にする
            foreach (var index in indexList)
            {
                dstPose.Joints[index] = srcPose.Joints[index] - srcPose.Joints[0];
            }
        }

        private void UpdateUpperPart(Pose dstPose, Pose srcPose, List<int> indexList)
        {
            // 下半身とのRoot差分を補正する
            var rootDelta = srcPose.Joints[0] - dstPose.Joints[0];
            // Root基準にする
            foreach (var index in indexList)
            {
                if (index == 0)
                {
                    continue;
                }

                dstPose.Joints[index] = srcPose.Joints[index] - dstPose.Joints[0];
            }
        }

    }
}