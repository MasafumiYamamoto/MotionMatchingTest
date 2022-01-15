using System.Collections.Generic;
using System.Numerics;
using BNSApp.Skelton;

namespace BNSApp
{
    public class Pose
    {
        public List<Vector3> Joints;

        public Pose()
        {
            Joints = new List<Vector3>();
            // 骨を初期化
            foreach (var keyValuePair in SkeltonData.BoneNameIndexList)
            {
                Joints.Add(Vector3.Zero);
            }
        }
        public Pose(List<Vector3> joints)
        {
            Joints = joints;
        }

        public Pose Add(Pose other)
        {
            var res = new Pose();
            for (var i = 0; i < other.Joints.Count; i++)
            {
                res.Joints[i] = Joints[i] + other.Joints[i];
            }

            return res;
        }
        
        public Pose Minus(Pose other)
        {
            var res = new Pose();
            for (var i = 0; i < other.Joints.Count; i++)
            {
                res.Joints[i] = Joints[i] - other.Joints[i];
            }

            return res;
        }

        public Pose Slash(float value)
        {
            var res = new Pose();
            for (var i = 0; i < Joints.Count; i++)
            {
                res.Joints[i] = Joints[i] / value;
            }

            return res;
        }
        
        public Pose Mult(float value)
        {
            var res = new Pose();
            for (var i = 0; i < Joints.Count; i++)
            {
                res.Joints[i] = Joints[i] * value;
            }

            return res;
        }
    }
}