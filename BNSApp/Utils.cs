using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Text;

namespace BNSApp
{
    public class Utils
    {
        public static void LoadData(string fileName, out string[] header, out List<int> motionIdList, out List<int> frameIdList, out List<List<Vector3>> posList)
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
                        if (line[2 + 3*i] == "")
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

        public static void CalLinearDelta(List<Vector3> a, List<Vector3> b, float denominator,  out List<Vector3> res)
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
    }
}