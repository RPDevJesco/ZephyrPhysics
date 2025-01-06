using ZephyrRenderer.Platform;

namespace _2DPhysicsEngine 
{
    public class SurfaceInfo
    {
        public (float X, float Y) Position { get; set; }
        public (float X, float Y) Normal { get; set; }
        public float Angle { get; set; }
        public bool IsGrounded { get; set; }
        public float Friction { get; set; }
        public bool IsLoop { get; set; }
        public float CurveRadius { get; set; }
        public bool CanTransferVelocity { get; set; }
    }

    public class SurfaceSystem 
    {
        public static SurfaceInfo AnalyzeSurface(Collider surface, (float X, float Y) contactPoint)
        {
            var vertices = surface.GetTransformedVertices();
            var closestEdge = FindClosestEdge(vertices, contactPoint);
            var normal = CalculateNormal(closestEdge);
            var angle = MathF.Atan2(normal.Y, normal.X);

            return new SurfaceInfo {
                Position = contactPoint,
                Normal = normal,
                Angle = angle,
                IsGrounded = normal.Y < -0.1f, // Y-down coordinate system
                Friction = 0.1f, // Default friction value
                IsLoop = DetectLoop(vertices, closestEdge),
                CurveRadius = CalculateCurveRadius(vertices, closestEdge),
                CanTransferVelocity = false // Default to false
            };
        }

        private static ((float X, float Y) Start, (float X, float Y) End) FindClosestEdge(
            (float X, float Y)[] vertices, (float X, float Y) point)
        {
            var closestDist = float.MaxValue;
            var closestEdge = (Start: vertices[0], End: vertices[1]);

            for (int i = 0; i < vertices.Length; i++)
            {
                var start = vertices[i];
                var end = vertices[(i + 1) % vertices.Length];
                var dist = PointToLineDistance(point, start, end);

                if (dist < closestDist)
                {
                    closestDist = dist;
                    closestEdge = (start, end);
                }
            }

            return closestEdge;
        }

        private static float PointToLineDistance(
            (float X, float Y) point, 
            (float X, float Y) lineStart, 
            (float X, float Y) lineEnd)
        {
            float dx = lineEnd.X - lineStart.X;
            float dy = lineEnd.Y - lineStart.Y;
            float magnitude = MathF.Sqrt(dx * dx + dy * dy);
            
            return MathF.Abs((point.Y - lineStart.Y) * dx - (point.X - lineStart.X) * dy) / magnitude;
        }

        private static (float X, float Y) CalculateNormal(
            ((float X, float Y) Start, (float X, float Y) End) edge)
        {
            float dx = edge.End.X - edge.Start.X;
            float dy = edge.End.Y - edge.Start.Y;
            float magnitude = MathF.Sqrt(dx * dx + dy * dy);
            
            return (-dy / magnitude, dx / magnitude);
        }

        private static bool DetectLoop((float X, float Y)[] vertices, 
            ((float X, float Y) Start, (float X, float Y) End) currentEdge)
        {
            // Simplified loop detection based on vertex pattern
            return false; // Implement full loop detection based on your needs
        }

        private static float CalculateCurveRadius((float X, float Y)[] vertices,
            ((float X, float Y) Start, (float X, float Y) End) currentEdge)
        {
            // Simplified curve calculation
            return 0; // Implement curve radius calculation based on your needs
        }
    }
}