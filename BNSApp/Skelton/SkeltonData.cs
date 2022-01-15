using System.Collections.Generic;
using System.Linq;

namespace BNSApp.Skelton
{
    public class SkeltonData
    {
        public static readonly Dictionary<string, int> BoneNameIndexList = new Dictionary<string, int>()
        {
            { "腰", 0 },
            { "背骨", 1 },
            { "胸", 2 },
            { "首", 3 },
            { "頭", 4 },
            { "左肩", 5 },
            { "左腕の付け根", 6 },
            { "左ひじ", 7 },
            { "左手", 8 },
            { "右肩", 9 },
            { "右腕の付け根", 10 },
            { "右ひじ", 11 },
            { "右手", 12 },
            { "左脚の付け根", 13 },
            { "左膝", 14 },
            { "左足のかかと", 15 },
            { "左足のつま先", 16 },
            { "右脚の付け根", 17 },
            { "右膝", 18 },
            { "右足のかかと", 19 },
            { "右足のつま先", 20 },
        };
        
        private static readonly List<string> UpperBoneList =  new List<string>()
        {
            "腰","背骨", "胸", "首", "頭",
            "左肩", "左腕の付け根", "左ひじ", "左手", 
            "右肩", "右腕の付け根", "右ひじ", "右手",
        };
        
        private static readonly List<string> LowerBoneList = new List<string>()
        {
            "腰",
            "左脚の付け根", "左膝", "左足のかかと", "左足のつま先",
            "右脚の付け根", "右膝", "右足のかかと", "右足のつま先",
        };

        private static List<int> _upperBoneIndexList;

        public static List<int> GetUpperBoneIndexList()
        {
            return _upperBoneIndexList ??= UpperBoneList.Select(bone => BoneNameIndexList[bone]).ToList();
        }

        public static List<int> GetLowerBoneIndexList()
        {
            return LowerBoneList.Select(bone => BoneNameIndexList[bone]).ToList();
        }
        
        
        public static readonly int[] Hip2Spine = { 0, 1 };
        public static readonly int[] spine2Chest = { 1, 2 };
        public static readonly int[] chest2Neck = { 2, 3 };
        public static readonly int[] neck2Head = { 3, 4 };

        public static readonly int[] chest2LeftShoulder = new[] { 2, 5 };
        public static readonly int[] leftShoulder2LeftUpperArm = new[] { 5, 6 };
        public static readonly int[] leftUpperArm2LeftLowerArm = new[] { 6, 7 };
        public static readonly int[] leftLowerArm2LeftHand = new[] { 7, 8 };

        public static readonly int[] chest2RightShoulder = new[] { 2, 9 };
        public static readonly int[] rightShoulder2RightUpperArm = new[] { 9, 10 };
        public static readonly int[] rightUpperArm2RightLowerArm = new[] { 10, 11 };
        public static readonly int[] rightLowerArm2RightHand = new[] { 11, 12 };

        public static readonly int[] hip2LeftUpperLeg = new[] { 0, 13 };
        public static readonly int[] leftUpperLeg2LeftLowerLeg = new[] { 13, 14 };
        public static readonly int[] leftLowerLeg2LeftFoot = new[] { 14, 15 };
        public static readonly int[] leftFoot2LeftToes = new[] { 15, 16 };

        public static readonly int[] hip2RightUpperLeg = new[] { 0, 17 };
        public static readonly int[] rightUpperLeg2RightLowerLeg = new[] { 17, 18 };
        public static readonly int[] rightLowerLeg2RightFoot = new[] { 18, 19 };
        public static readonly int[] rightFoot2RightToes = new[] { 19, 20 };
    }
}