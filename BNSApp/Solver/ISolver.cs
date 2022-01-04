namespace BNSApp.Solver
{
    public interface ISolver
    {
        public string GetName();
        public void Solve(MotionData motionData, MotionContainer trainData);
    }
}