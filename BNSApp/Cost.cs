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
                return 2;
            }
            
            return 1.5f;
        }
    }
}