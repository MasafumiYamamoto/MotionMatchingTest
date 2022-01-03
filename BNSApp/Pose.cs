using System.Collections.Generic;
using System.Numerics;

namespace BNSApp
{
    public class Pose
    {
        public List<Vector3> Joints;

        public Pose()
        {
            Joints = new List<Vector3>();
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
                res.Joints.Add(Joints[i] + other.Joints[i]);
            }

            return res;
        }
        
        public Pose Minus(Pose other)
        {
            var res = new Pose();
            for (var i = 0; i < other.Joints.Count; i++)
            {
                res.Joints.Add(Joints[i] - other.Joints[i]);
            }

            return res;
        }

        public Pose Slash(float value)
        {
            var res = new Pose();
            for (var i = 0; i < Joints.Count; i++)
            {
                res.Joints.Add(Joints[i] / value);
            }

            return res;
        }
        
        public Pose Mult(float value)
        {
            var res = new Pose();
            for (var i = 0; i < Joints.Count; i++)
            {
                res.Joints.Add(Joints[i] * value);
            }

            return res;
        }
    }
}