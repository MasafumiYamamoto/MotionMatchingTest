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
    }
}