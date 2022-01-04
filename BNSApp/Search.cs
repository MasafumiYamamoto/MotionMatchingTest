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
            var motionType = Utils.GetMotionType(motionId);
            var minCost = float.MaxValue;
            nearestCurrentPose = new Pose();
            nearestFuturePose = new Pose();
            foreach (var trainMotion in trainData.MotionList)
            {
                for (int i = 0; i < trainMotion.Length; i++)
                {
                    if (i - direction < 0 || trainMotion.Length <= i - direction)
                    {
                        continue;
                    }

                    if (i + direction < 0 || trainMotion.Length <= i + direction)
                    {
                        continue;
                    }

                    var currentPose = trainMotion.PosList[i];
                    var trainMotionType = Utils.GetMotionType(trainMotion.MotionId);
                    var cost = Cost.CalcRootCost(currentPose, pose) *
                               Cost.CalcMotionTypeCost(motionType, trainMotionType);
                    if (cost < minCost)
                    {
                        minCost = cost;
                        nearestCurrentPose = currentPose;
                        nearestFuturePose = trainMotion.PosList[i + direction];
                    }
                }
            }
        }
    }
}