using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;

namespace BNSApp
{
    public enum Difficulty
    {
        Easy,
        Normal,
        Hard
    }
    
    public class MotionContainer
    {
        public Difficulty TestDifficulty;
        private readonly string _fileName;
        private readonly int _skipFrames;
        private readonly string[] _header;
        public readonly List<MotionData> MotionList = new List<MotionData>();

        public MotionContainer(string fileName, int skipFrames = 1)
        {
            _fileName = fileName;
            _skipFrames = skipFrames;
            switch (skipFrames)
            {
                case 5:
                    TestDifficulty = Difficulty.Easy;
                    break;
                case 15:
                    TestDifficulty = Difficulty.Normal;
                    break;
                case 45:
                    TestDifficulty = Difficulty.Hard;
                    break;
                default:
                    break;
            }

            Utils.LoadData(fileName, out var header, out var motionIdList, out var frameIdList, out var posList);
            _header = header;

            var currentMotionId = motionIdList[0];
            var tmpFrameIdList = new List<int>();
            var tmpPosList = new List<List<Vector3>>();
            for (var i = 0; i < motionIdList.Count; i++)
            {
                var motionId = motionIdList[i];
                var frameId = frameIdList[i];
                var pos = posList[i];

                // モーションIDが変わったらこれまでの情報を保存してリセット
                if (motionId != currentMotionId)
                {
                    var motion = new MotionData(currentMotionId, tmpFrameIdList, tmpPosList, skipFrames);
                    MotionList.Add(motion);
                    tmpFrameIdList = new List<int>();
                    tmpPosList = new List<List<Vector3>>();
                }

                currentMotionId = motionId;
                tmpFrameIdList.Add(frameId);
                tmpPosList.Add(pos);
            }

            // 最後に残ったデータを保存
            var lastMotion = new MotionData(currentMotionId, tmpFrameIdList, tmpPosList, skipFrames);
            MotionList.Add(lastMotion);
        }

        /// <summary>
        /// ID指定で対応するモーションを探す
        /// </summary>
        /// <param name="motionId"></param>
        /// <returns></returns>
        public MotionData GetMotionById(int motionId)
        {
            return MotionList.FirstOrDefault(motionData => motionData.MotionId == motionId);
        }

        public void SaveData(string fileName)
        {
            using (var sw = new StreamWriter(fileName))
            {
                sw.WriteLine(string.Join(',', _header));
                foreach (var motionData in MotionList)
                {
                    motionData.SaveMotion(sw);
                }
            }
        }
    }

    public class MotionData
    {
        public readonly int MotionId;
        public readonly List<int> FrameList;
        public readonly List<Pose> PosList;
        public readonly int SkipFrames;
        public readonly int Length;
        public readonly int Loop;

        public MotionData(int motionId, List<int> frameList, List<List<Vector3>> posList, int skipFrames)
        {
            MotionId = motionId;
            FrameList = frameList;
            PosList = new List<Pose>();
            foreach (var pose in posList)
            {
                PosList.Add(new Pose(pose));
            }
            SkipFrames = skipFrames;
            Length = posList.Count;
            Loop = Length / skipFrames;
        }

        public void SaveMotion(StreamWriter sw)
        {
            for (int i = 0; i < Length; i++)
            {
                var pose = PosList[i];
                sw.Write($"{MotionId:000},");
                sw.Write($"{FrameList[i]},");
                var posLine = new List<float>();
                foreach (var joint in pose.Joints)
                {
                    posLine.Add(joint.X);
                    posLine.Add(joint.Y);
                    posLine.Add(joint.Z);
                }

                sw.Write($"{string.Join(',', posLine)}");
                sw.Write("\n");
            }
        }
    }
}