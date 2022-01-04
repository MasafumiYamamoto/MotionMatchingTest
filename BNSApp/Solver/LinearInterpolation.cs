namespace BNSApp.Solver
{
    /// <summary>
    /// 既知のフレーム情報のみを利用して、線形補間によってポーズを推定する
    /// </summary>
    public class LinearInterpolation : ISolver
    {
        public string GetName()
        {
            return "Linear";
        }
        public void Solve(MotionData motionData, MotionContainer trainData)
        {
            for (int loop = 0; loop < motionData.Loop; loop++)
            {
                var prevMeasuredId = loop * motionData.SkipFrames + 1;
                var nextMeasuredId = (loop + 1) * motionData.SkipFrames + 1;
                for (int i = 1; i < motionData.SkipFrames; i++)
                {
                    var currentPos = motionData.PosList[prevMeasuredId - 2 + i];
                    var nextPos = motionData.PosList[nextMeasuredId - 1];
                    var delta = nextPos.Minus(currentPos);
                    delta = delta.Slash(motionData.SkipFrames - i);
                    var newPos = currentPos.Add(delta);
                    motionData.PosList[prevMeasuredId - 1 + i] = newPos;
                }
            }
        }
    }
}