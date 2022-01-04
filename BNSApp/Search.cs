namespace BNSApp
{
    public class Search
    {
        /// <summary>
        /// テストデータから最近傍ポーズを探す
        /// </summary>
        /// <param name="trainData"></param>
        /// <param name="pose"></param>
        /// <param name="direction"></param>
        /// <param name="motionId"></param>
        /// <param name="nearestCurrentPose"></param>
        /// <param name="nearestFuturePose"></param>
        public static void FindNearestMotion(MotionContainer trainData, Pose pose, int direction, int motionId, out Pose nearestCurrentPose, out Pose nearestFuturePose)
        {
            var minCost = float.MaxValue;
            nearestCurrentPose = new Pose();
            nearestFuturePose = new Pose();
            foreach (var motion in trainData.MotionList)
            {
                for (int i = 0; i < motion.Length; i++)
                {
                    if (i - direction < 0 || motion.Length <= i - direction)
                    {
                        continue;
                    }

                    if (i + direction < 0 || motion.Length <= i + direction)
                    {
                        continue;
                    }

                    var currentPose = motion.PosList[i];
                    var cost = Cost.CalcRootCost(currentPose, pose);
                    if (cost < minCost)
                    {
                        minCost = cost;
                        nearestCurrentPose = currentPose;
                        nearestFuturePose = motion.PosList[i + direction];
                    }
                }
            }
        }
    }
}