using System.Collections.Generic;

namespace BNSApp
{
    /// <summary>
    /// 最近傍ポーズを利用して次のポーズを計算する
    /// 前後方向から計算して次のポーズをブレンドしながら求めている
    /// </summary>
    public class NNInterpolation : ISolver
    {
        public void Solve(MotionData motionData, MotionContainer trainData){
            for (int loop = 0; loop < motionData.Loop; loop++)
            {
                var prevMeasuredId = loop * motionData.SkipFrames + 1;
                var nextMeasuredId = (loop + 1) * motionData.SkipFrames + 1;
                var forwardList = new List<Pose>();
                var backwardList = new List<Pose>();

                // 双方向から補間計算していく
                for (int i = 1; i < motionData.SkipFrames; i++)
                {
                    var currentPos = motionData.PosList[prevMeasuredId - 2 + i];
                    var prevPos = motionData.PosList[prevMeasuredId - 1];
                    var nextPos = motionData.PosList[nextMeasuredId - 1];
                    
                    if (i != 1)
                    {
                        prevPos = forwardList[^1];
                        nextPos = backwardList[^1];
                    }

                    var forwardPos = Interp(trainData, prevPos, 1, motionData.MotionId);
                    var backwardPos = Interp(trainData, nextPos, -1, motionData.MotionId);
                    forwardList.Add(forwardPos);
                    backwardList.Add(backwardPos);
                }

                for (int i = 1; i < motionData.SkipFrames; i++)
                {
                    var pos = Merge.LinearBlend(forwardList[i - 1], backwardList[^i], (float)(i - 1) / (motionData.SkipFrames - 2));
                    motionData.PosList[prevMeasuredId - 1 + i] = pos;
                }
            }
        }

        static Pose Interp(MotionContainer trainData,Pose pos, int direction, int motionId)
        {
            Search.FindNearestMotion(trainData, pos, direction, motionId, out var nearestCurrentPose, out var nearestFuturePose);
            var delta = Utils.ComputeDelta(nearestCurrentPose, nearestFuturePose);
            var newPos = pos.Add(delta);
            return newPos;
        }
    }
}