using System;
using System.Collections.Generic;
using System.Numerics;
using BNSApp.Skelton;

namespace BNSApp
{
    public class Cost
    {
        /// <summary>
        /// ルートの骨位置を使った相対位置でのコストを計算する関数
        /// ルートには先頭要素の腰骨を利用
        /// </summary>
        /// <param name="p0"></param>
        /// <param name="p1"></param>
        /// <returns></returns>
        public static float CalcRootCost(Pose p0, Pose p1)
        {
            var cost = 0f;
            var root0 = p0.Joints[0];
            var root1 = p1.Joints[0];
            for (int i = 0; i < p0.Joints.Count; i++)
            {
                var j0 = p0.Joints[i] - root0;
                var j1 = p1.Joints[i] - root1;
                cost += (j0 - j1).Length();
            }

            return cost;
        }

        /// <summary>
        /// 単純な各ボーンの位置情報を利用してのコスト計算
        /// </summary>
        /// <param name="p0"></param>
        /// <param name="p1"></param>
        /// <returns></returns>
        public static float CalcAbsolutePosCost(Pose p0, Pose p1)
        {
            var cost = 0f;
            for (int i = 0; i < p0.Joints.Count; i++)
            {
                var j0 = p0.Joints[i];
                var j1 = p1.Joints[i];
                cost += (j0 - j1).Length();
            }

            return cost;
        }

        /// <summary>
        /// ２つのモーションが同一種類かで重みづけ
        /// </summary>
        /// <param name="type0"></param>
        /// <param name="type1"></param>
        /// <returns></returns>
        public static float CalcMotionTypeCost(Utils.MotionType type0, Utils.MotionType type1)
        {
            if (type0 == type1)
            {
                return 1;
            }

            // NOTE: 後ろ歩きだけは特殊なので別扱いにしてみる
            if (type0 == Utils.MotionType.Back || type1 == Utils.MotionType.Back)
            {
                return 4;
            }

            return 1.5f;
        }

        /// <summary>
        /// 利用するボーンを指定してルートからの位置を使ってコスト計算
        /// </summary>
        /// <param name="p0"></param>
        /// <param name="p1"></param>
        /// <param name="indexList"></param>
        /// <returns></returns>
        public static float CalcPartPoseCost(Pose p0, Pose p1, List<int> indexList)
        {
            var cost = 0f;
            var root0 = p0.Joints[0];
            var root1 = p1.Joints[0];
            foreach (var index in indexList)
            {
                var j0 = p0.Joints[index] - root0;
                var j1 = p1.Joints[index] - root1;
                cost += (j0 - j1).Length();
            }

            return cost;
        }

        /// <summary>
        /// 各ボーンの回転を利用して類似度計算する
        /// </summary>
        /// <param name="p0"></param>
        /// <param name="p1"></param>
        /// <returns></returns>
        public static float CalcRotationCost(Pose p0, Pose p1)
        {
            var cost = CalcBoneRotationCost(p0, p1, SkeltonData.Hip2Spine);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.spine2Chest);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.chest2Neck);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.neck2Head);

            cost += CalcBoneRotationCost(p0, p1, SkeltonData.chest2LeftShoulder);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.leftShoulder2LeftUpperArm);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.leftUpperArm2LeftLowerArm);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.leftLowerArm2LeftHand);

            cost += CalcBoneRotationCost(p0, p1, SkeltonData.chest2RightShoulder);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.rightShoulder2RightUpperArm);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.rightUpperArm2RightLowerArm);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.rightLowerArm2RightHand);

            cost += CalcBoneRotationCost(p0, p1, SkeltonData.hip2LeftUpperLeg);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.leftUpperLeg2LeftLowerLeg);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.leftLowerLeg2LeftFoot);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.leftFoot2LeftToes);

            cost += CalcBoneRotationCost(p0, p1, SkeltonData.hip2RightUpperLeg);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.rightUpperLeg2RightLowerLeg);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.rightLowerLeg2RightFoot);
            cost += CalcBoneRotationCost(p0, p1, SkeltonData.rightFoot2RightToes);

            return cost;
        }

        private static float CalcBoneRotationCost(Pose pose0, Pose pose1, int[] boneIndexes)
        {
            // var lowerBoneIndexList = SkeltonData.GetLowerBoneIndexList();
            // if (!lowerBoneIndexList.Contains(boneIndexes[0]) || !lowerBoneIndexList.Contains(boneIndexes[1]))
            // {
            //     return 0;
            // }
                
            var bone0 = pose0.Joints[boneIndexes[1]] - pose0.Joints[boneIndexes[0]];
            var bone1 = pose1.Joints[boneIndexes[1]] - pose1.Joints[boneIndexes[0]];

            bone0 = Vector3.Normalize(bone0);
            bone1 = Vector3.Normalize(bone1);

            return -Vector3.Dot(bone0, bone1);
        }

        /// <summary>
        /// 現在のフレームと次の既知フレームとの差分に応じて未来のコストをどれだけ考慮するか計算する
        /// </summary>
        /// <param name="currentFrameIndex"></param>
        /// <param name="futureFrameIndex"></param>
        /// <returns></returns>
        public static float CalcFrameCost(int currentFrameIndex, int futureFrameIndex, int skipFrames)
        {
            var delta = MathF.Abs(futureFrameIndex - currentFrameIndex);
            var k = (skipFrames - delta) / skipFrames;
            return k*4;
        }
    }
}