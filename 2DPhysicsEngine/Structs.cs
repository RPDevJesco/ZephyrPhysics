using System.Runtime.InteropServices;

namespace ZephyrRenderer.Platform
{
    [StructLayout(LayoutKind.Sequential)]
    public struct RECT
    {
        public double X;
        public double Y;
        public double Width;
        public double Height;

        public RECT(double x, double y, double width, double height)
        {
            X = x;
            Y = y;
            Width = width;
            Height = height;
        }

        public bool Contains(POINT point)
        {
            // Convert to integers for comparison to avoid floating-point precision issues
            int px = (int)Math.Round(point.X);
            int py = (int)Math.Round(point.Y);
            int rx = (int)Math.Round(X);
            int ry = (int)Math.Round(Y);
            int rw = (int)Math.Round(Width);
            int rh = (int)Math.Round(Height);

            return px >= rx && px < rx + rw &&
                   py >= ry && py < ry + rh;
        }

        public RECT Scale(double scaleFactor)
        {
            return new RECT(X * scaleFactor, Y * scaleFactor, Width * scaleFactor, Height * scaleFactor);
        }

        public bool Intersects(RECT other)
        {
            return X < other.X + other.Width &&
                   X + Width > other.X &&
                   Y < other.Y + other.Height &&
                   Y + Height > other.Y;
        }

        public RECT ToIntRect()
        {
            return new RECT((int)X, (int)Y, (int)Width, (int)Height);
        }
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct POINT
    {
        public double X;
        public double Y;

        public POINT(double x, double y)
        {
            X = x;
            Y = y;
        }

        public POINT ToIntPoint()
        {
            return new POINT((int)X, (int)Y);
        }
    }
}