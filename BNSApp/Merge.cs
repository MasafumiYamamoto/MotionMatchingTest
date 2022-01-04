using System;

namespace BNSApp
{
    public class Merge
    {
        /// <summary>
        /// 線形和によってブレンドする
        /// </summary>
        /// <param name="forward"></param>
        /// <param name="backward"></param>
        /// <param name="ratio"></param>
        /// <returns></returns>
        public static Pose LinearBlend(Pose forward, Pose backward, float ratio)
        {
            ratio = MathF.Min(ratio, 1);
            var p0 = forward.Mult(1 - ratio);
            var p1 = backward.Mult(ratio);
            return p0.Add(p1);
        }
    }
}