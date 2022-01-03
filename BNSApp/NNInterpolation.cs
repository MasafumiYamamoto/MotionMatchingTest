using System.Collections.Generic;

namespace BNSApp
{
    public class NNInterpolation
    {
        public static void Solve(MotionData motionData, MotionContainer trainData){
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
                    var pos = Merge(forwardList[i - 1], backwardList[^i], (float)(i - 1) / (motionData.SkipFrames - 2));
                    motionData.PosList[prevMeasuredId - 1 + i] = pos;
                }
            }
        }

        private static Pose Merge(Pose forward, Pose backward, float ratio)
        {
            var p0 = forward.Mult(1 - ratio);
            var p1 = backward.Mult(ratio);
            return p0.Add(p1);
        }

        static Pose Interp(MotionContainer trainData,Pose pos, int direction, int motionId)
        {
            FindNearestMotion(trainData, pos, direction, motionId, out var nearestCurrentPose, out var nearestFuturePose);
            var delta = ComputeDelta(nearestCurrentPose, nearestFuturePose);
            var newPos = pos.Add(delta);
            return newPos;
        }

        static Pose ComputeDelta(Pose currentPose, Pose futurePose)
        {
            return futurePose.Minus(currentPose);
        }

        static void FindNearestMotion(MotionContainer trainData, Pose pose, int direction, int motionId, out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var minCost = float.MaxValue;
            nearestCurrentPose = new Pose();
            nearestFuturePose = new Pose();
            foreach (var motion in trainData.MotionList)
            {
                for (int i = 0; i < motion.Length; i++)
                {
                    if (i - direction < 0 || motion.Length <= i - direction)
                    {
                        continue;
                    }

                    if (i + direction < 0 || motion.Length <= i + direction)
                    {
                        continue;
                    }

                    var currentPose = motion.PosList[i];
                    var cost = CalcRootCost(currentPose, pose);
                    if (cost < minCost)
                    {
                        minCost = cost;
                        nearestCurrentPose = currentPose;
                        nearestFuturePose = motion.PosList[i + direction];
                    }
                }
            }
        }

        static float CalcRootCost(Pose p0, Pose p1)
        {
            var cost = 0f;
            var root0 = p0.Joints[0];
            var root1 = p1.Joints[0];
            for (int i = 0; i < p0.Joints.Count; i++)
            {
                var j0 = p0.Joints[i] -root0;
                var j1 = p1.Joints[i] - root1;
                cost += (j0 - j1).Length();
            }

            return cost;
        }
    }
}