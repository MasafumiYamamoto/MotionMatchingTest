using System;

/// <summary>
/// 確率分布
/// </summary>
public class Distribution
{
    public static float Sample(float x, float width)
    {
        return MathF.Exp(-x * x / 5);
    }
}