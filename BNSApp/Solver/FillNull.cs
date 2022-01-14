namespace BNSApp.Solver
{
    public class FillNull : ISolver
    {
        public string GetName()
        {
            return "FillNA";
        }

        public void Solve(MotionData motionData, MotionContainer trainData)
        {
            for (int loop = 0; loop < motionData.Loop; loop++)
            {
                var prevMeasuredId = loop * motionData.SkipFrames + 1;
                for (int i = 1; i < motionData.SkipFrames; i++)
                {
                    motionData.PosList[prevMeasuredId - 1 + i] = motionData.PosList[prevMeasuredId - 1];
                }
            }
        }
    }
}