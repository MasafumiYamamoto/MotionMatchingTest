using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Text;
using BNSApp.Skelton;

namespace BNSApp
{
    public class Utils
    {
        public static void LoadData(string fileName, out string[] header, out List<int> motionIdList,
            out List<int> frameIdList, out List<List<Vector3>> posList)
        {
            const int jointDim = 21;
            const int posDim = 3;

            using (var st = new StreamReader(fileName, Encoding.UTF8))
            {
                header = st.ReadLine().Split(",");
                motionIdList = new List<int>();
                frameIdList = new List<int>();
                posList = new List<List<Vector3>>();
                while (st.EndOfStream == false)
                {
                    var line = st.ReadLine().Split(",");
                    motionIdList.Add(int.Parse(line[0]));
                    frameIdList.Add(int.Parse(line[1]));
                    var pos = new List<Vector3>();
                    for (int i = 0; i < jointDim; i++)
                    {
                        if (line[2 + 3 * i] == "")
                        {
                            pos.Add(Vector3.Zero);
                            continue;
                        }

                        var x = float.Parse(line[2 + 3 * i]);
                        var y = float.Parse(line[3 + 3 * i]);
                        var z = float.Parse(line[4 + 3 * i]);
                        var p = new Vector3(x, y, z);
                        pos.Add(p);
                    }

                    posList.Add(pos);
                }
            }
        }

        public static void CalLinearDelta(List<Vector3> a, List<Vector3> b, float denominator, out List<Vector3> res)
        {
            res = new List<Vector3>();
            for (int i = 0; i < a.Count; i++)
            {
                var aa = a[i];
                var bb = b[i];
                res.Add((aa - bb) / denominator);
            }
        }

        public static void AddDelta(List<Vector3> pos, List<Vector3> delta, out List<Vector3> res)
        {
            res = new List<Vector3>();
            for (int i = 0; i < pos.Count; i++)
            {
                res.Add(pos[i] + delta[i]);
            }
        }

        public static Pose ComputeDelta(Pose currentPose, Pose futurePose)
        {
            return futurePose.Minus(currentPose);
        }

        /// <summary>
        /// currentPoseからfuturePoseへのボーンの回転情報を転写する
        /// ついでにルートはそのまま動かす
        /// </summary>
        /// <param name="currentPose"></param>
        /// <param name="futurePose"></param>
        /// <param name="dstPose"></param>
        public static void RotationDelta(Pose currentPose, Pose futurePose, Pose dstPose)
        {
            var rootMove = futurePose.Joints[0] - currentPose.Joints[0];
        }

        public enum MotionType
        {
            Hello,
            Hand1,
            Hand2,
            Run,
            Finger,
            Walk,
            Back,
            Side1,
            Side2,
        }

        public static MotionType GetMotionType(int motionId)
        {
            switch (motionId)
            {
                case <= 14:
                    return MotionType.Hello;
                case <= 28:
                    return MotionType.Hand1;
                case <= 42:
                    return MotionType.Hand2;
                case <= 56:
                    return MotionType.Run;
                case <= 70:
                    return MotionType.Finger;
                case <= 84:
                    return MotionType.Run;
                case <= 100:
                    return MotionType.Walk;
                case <= 113:
                    return MotionType.Walk;
                case <= 127:
                    return MotionType.Back;
                case <= 142:
                    return MotionType.Side1;
                default:
                    return MotionType.Side2;
            }
        }

        public static string GetOutputFileName(MotionContainer testData, string outputDirectory, string postFix)
        {
            var outputFileName = testData.TestDifficulty switch
            {
                Difficulty.Easy => $"{outputDirectory}/EasyOutput{postFix}.csv",
                Difficulty.Normal => $"{outputDirectory}/NormalOutput{postFix}.csv",
                Difficulty.Hard => $"{outputDirectory}/HardOutput{postFix}.csv",
                _ => throw new ArgumentOutOfRangeException()
            };

            return outputFileName;
        }

        /// <summary>
        /// 現在のポーズだけを利用して一番マッチする次のポーズを探す
        /// </summary>
        /// <param name="trainData"></param>
        /// <param name="currentPos"></param>
        /// <param name="direction"></param>
        /// <param name="motionId"></param>
        /// <returns></returns>
        public static Pose FindNextFrame(MotionContainer trainData, Pose currentPos, int direction, int motionId)
        {
            SearchBestPose(trainData, currentPos, direction, motionId,
                out var nearestCurrentPose, out var nearestFuturePose);
            var deltaPos = ComputeDelta(nearestCurrentPose, nearestFuturePose);
            return currentPos.Add(deltaPos);
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
        private static void SearchBestPose(MotionContainer trainData, Pose currentPos, int direction,
            int motionId, out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var testMotionType = GetMotionType(motionId);
            var minCost = float.MaxValue;
            var nearestMotionType = MotionType.Back;
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
                    var trainMotionType = GetMotionType(trainMotion.MotionId);
                    var cost = Cost.CalcRootCost(currentTrainPose, currentPos) *
                               Cost.CalcMotionTypeCost(testMotionType, trainMotionType);
                    if (cost < minCost)
                    {
                        minCost = cost;
                        nearestCurrentPose = currentTrainPose;
                        nearestFuturePose = trainMotion.PosList[i + direction];
                        nearestMotionType = GetMotionType(trainMotion.MotionId);
                    }
                }
            }
        }

        private static float CalcBoneLength(Pose pose, int[] boneIndexes)
        {
            var j0 = pose.Joints[boneIndexes[0]];
            var j1 = pose.Joints[boneIndexes[1]];
            return (j0 - j1).Length();
        }

        /// <summary>
        /// キャラクターの全身の骨の長さを計算する
        /// キャラクターの差異があれば利用できる可能性がありそう
        /// </summary>
        /// <param name="p0"></param>
        /// <returns></returns>
        public static float CalcTotalBoneLength(Pose p0)
        {
            var cost = CalcBoneLength(p0, SkeltonData.Hip2Spine);
            cost += CalcBoneLength(p0, SkeltonData.spine2Chest);
            cost += CalcBoneLength(p0, SkeltonData.chest2Neck);
            cost += CalcBoneLength(p0, SkeltonData.neck2Head);

            cost += CalcBoneLength(p0, SkeltonData.chest2LeftShoulder);
            cost += CalcBoneLength(p0, SkeltonData.leftShoulder2LeftUpperArm);
            cost += CalcBoneLength(p0, SkeltonData.leftUpperArm2LeftLowerArm);
            cost += CalcBoneLength(p0, SkeltonData.leftLowerArm2LeftHand);

            cost += CalcBoneLength(p0, SkeltonData.chest2RightShoulder);
            cost += CalcBoneLength(p0, SkeltonData.rightShoulder2RightUpperArm);
            cost += CalcBoneLength(p0, SkeltonData.rightUpperArm2RightLowerArm);
            cost += CalcBoneLength(p0, SkeltonData.rightLowerArm2RightHand);

            cost += CalcBoneLength(p0, SkeltonData.hip2LeftUpperLeg);
            cost += CalcBoneLength(p0, SkeltonData.leftUpperLeg2LeftLowerLeg);
            cost += CalcBoneLength(p0, SkeltonData.leftLowerLeg2LeftFoot);
            cost += CalcBoneLength(p0, SkeltonData.leftFoot2LeftToes);

            cost += CalcBoneLength(p0, SkeltonData.hip2RightUpperLeg);
            cost += CalcBoneLength(p0, SkeltonData.rightUpperLeg2RightLowerLeg);
            cost += CalcBoneLength(p0, SkeltonData.rightLowerLeg2RightFoot);
            cost += CalcBoneLength(p0, SkeltonData.rightFoot2RightToes);

            return cost;
        }
    }
}