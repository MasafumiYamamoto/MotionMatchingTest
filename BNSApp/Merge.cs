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

        /// <summary>
        /// ratioが１付近で急激にbackwardを採用するようにする
        /// </summary>
        /// <param name="forward"></param>
        /// <param name="backward"></param>
        /// <param name="ratio"></param>
        /// <returns></returns>
        public static Pose SmoothStepBlend(Pose forward, Pose backward, float ratio)
        {
            ratio = MathF.Min(ratio, 1);
            ratio = SmoothStep(0.2f, 0.8f, ratio);
            return LinearBlend(forward, backward, ratio);
        }

        private static float SmoothStep(float a, float b, float x)
        {
            var t = MathF.Min(1, MathF.Max(0, (x - a) / (b - a)));
            return t * t * (3 - 2 * t);
        }
    }
}