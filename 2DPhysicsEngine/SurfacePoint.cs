namespace _2DPhysicsEngine
{
    public class SurfacePoint
    {
        public (float X, float Y) Position { get; set; }
        public (float X, float Y) Normal { get; set; }
        public (float X, float Y) EdgeStart { get; set; }
        public (float X, float Y) EdgeEnd { get; set; }
        public float SurfaceAngle { get; set; }
        public bool IsGrounded { get; set; }
        public float Friction { get; set; }
        public bool IsLoop { get; set; }
        public float CurveRadius { get; set; }
        public bool CanTransferVelocity { get; set; }
        public bool IsCorner { get; set; }
        public float DistanceFromSurface { get; set; }
        public float AttachmentStrength { get; set; }
    }

    public static class OrientationHelper
    {
        private const float MAX_SNAP_DISTANCE = 10f;
        private const float CORNER_ANGLE_THRESHOLD = 45f * (MathF.PI / 180f);
        private const float CURVE_DETECTION_THRESHOLD = 0.1f;
        
        public static SurfacePoint FindBottomPoint(PhysicsObject obj, Collider collider)
        {
            var vertices = collider.GetTransformedVertices();
            var gravityDir = obj.Environment.CalculateGravityAt(obj.X, obj.Y);
            
            // Find multiple lowest points for better surface detection
            var sortedPoints = vertices
                .Select(v => (
                    Point: v,
                    Height: v.X * gravityDir.X + v.Y * gravityDir.Y))
                .OrderBy(p => p.Height)
                .Take(2)
                .ToList();

            // Get the best contact point based on current motion and orientation
            var bestPoint = ChooseBestContactPoint(obj, sortedPoints.Select(p => p.Point).ToArray());

            // Find the edge this point belongs to
            var nearestEdge = FindNearestEdge(bestPoint, vertices);
            
            // Calculate surface properties
            var normal = CalculateNormal(nearestEdge);
            var angle = CalculateSurfaceAngle(normal);
            var curveRadius = CalculateCurveRadius(vertices, nearestEdge);
            var isCorner = DetectCorner(vertices, nearestEdge);

            return new SurfacePoint
            {
                Position = bestPoint,
                Normal = normal,
                EdgeStart = nearestEdge.Start,
                EdgeEnd = nearestEdge.End,
                SurfaceAngle = angle,
                IsGrounded = normal.Y < -0.1f,
                Friction = obj.Config?.GroundFriction ?? 0.1f,
                IsLoop = DetectLoop(vertices, nearestEdge),
                CurveRadius = curveRadius,
                CanTransferVelocity = true,
                IsCorner = isCorner,
                DistanceFromSurface = CalculateDistanceFromSurface(bestPoint, nearestEdge),
                AttachmentStrength = CalculateAttachmentStrength(obj, normal)
            };
        }

        public static void AlignWithSurface(PhysicsObject obj, SurfacePoint surfacePoint)
        {
            // Calculate target rotation based on surface normal and motion
            float targetRotation = CalculateTargetRotation(obj, surfacePoint);
            
            // Handle corner cases
            if (surfacePoint.IsCorner)
            {
                targetRotation = HandleCornerRotation(obj, surfacePoint);
            }

            // Apply smooth rotation with dynamic strength
            float rotationStrength = CalculateRotationStrength(obj, surfacePoint);
            ApplySmoothRotation(obj, targetRotation, rotationStrength);

            // Apply surface attachment forces
            ApplySurfaceAttachment(obj, surfacePoint);
        }

        private static (float X, float Y) ChooseBestContactPoint(PhysicsObject obj, (float X, float Y)[] candidates)
        {
            if (candidates.Length == 0)
                return (obj.X, obj.Y);

            var velocityWeight = (X: obj.VX, Y: obj.VY);
            var orientationVector = (
                X: MathF.Cos(obj.Rotation),
                Y: MathF.Sin(obj.Rotation)
            );

            return candidates.OrderBy(p => 
                CalculatePointScore(p, obj, velocityWeight, orientationVector)
            ).First();
        }

        private static float CalculatePointScore((float X, float Y) point, PhysicsObject obj,
            (float X, float Y) velocityWeight, (float X, float Y) orientationVector)
        {
            // Lower score is better
            float velocityScore = MathF.Abs(
                point.X * velocityWeight.X + point.Y * velocityWeight.Y);
            
            float orientationScore = MathF.Abs(
                point.X * orientationVector.X + point.Y * orientationVector.Y);
            
            // If we have a previous contact point, prefer points closer to it
            float consistencyScore = 0;
            if (obj.CurrentSurface != null)
            {
                var prev = obj.CurrentSurface.Position;
                consistencyScore = MathF.Sqrt(
                    MathF.Pow(point.X - prev.X, 2) + 
                    MathF.Pow(point.Y - prev.Y, 2));
            }

            return velocityScore * 0.4f + orientationScore * 0.3f + consistencyScore * 0.3f;
        }

        private static float CalculateTargetRotation(PhysicsObject obj, SurfacePoint surfacePoint)
        {
            // Base rotation from surface normal
            float baseRotation = MathF.Atan2(surfacePoint.Normal.Y, surfacePoint.Normal.X);
            
            // Adjust for velocity direction
            float velocityAngle = MathF.Atan2(obj.VY, obj.VX);
            float velocityInfluence = MathF.Min(1f, 
                MathF.Sqrt(obj.VX * obj.VX + obj.VY * obj.VY) / 10f);
            
            return baseRotation * (1 - velocityInfluence) + velocityAngle * velocityInfluence;
        }

        private static void ApplySmoothRotation(PhysicsObject obj, float targetRotation, float strength)
        {
            float rotationDiff = targetRotation - obj.Rotation;
            
            // Normalize rotation difference
            while (rotationDiff > MathF.PI) rotationDiff -= 2 * MathF.PI;
            while (rotationDiff < -MathF.PI) rotationDiff += 2 * MathF.PI;
            
            // Apply smooth rotation with dynamic strength
            obj.AngularVelocity += rotationDiff * strength;
            
            // Dampen angular velocity
            obj.AngularVelocity *= 0.9f;
        }

        private static void ApplySurfaceAttachment(PhysicsObject obj, SurfacePoint surfacePoint)
        {
            // Calculate attachment force based on distance from surface
            float attachmentForce = surfacePoint.AttachmentStrength * 
                                  (1f - surfacePoint.DistanceFromSurface / MAX_SNAP_DISTANCE);
            
            // Apply force towards surface along normal
            if (attachmentForce > 0)
            {
                obj.ApplyForce(
                    surfacePoint.Normal.X * attachmentForce,
                    surfacePoint.Normal.Y * attachmentForce
                );
            }
        }

        private static float CalculateAttachmentStrength(PhysicsObject obj, (float X, float Y) normal)
        {
            // Calculate based on:
            // 1. Object's velocity (faster = weaker attachment)
            // 2. Surface angle (steeper = stronger attachment)
            // 3. Object's mass (heavier = stronger attachment)
            
            float speedFactor = 1f / (1f + MathF.Sqrt(obj.VX * obj.VX + obj.VY * obj.VY));
            float angleFactor = MathF.Abs(normal.Y); // Vertical surfaces need more attachment
            float massFactor = MathF.Sqrt(obj.Mass);
            
            return 10f * speedFactor * angleFactor * massFactor;
        }

        private static float HandleCornerRotation(PhysicsObject obj, SurfacePoint surfacePoint)
        {
            // Special handling for corners:
            // 1. Use the velocity direction more heavily
            // 2. Consider both adjacent surfaces
            // 3. Smooth out the transition
            
            float velocityAngle = MathF.Atan2(obj.VY, obj.VX);
            float surfaceAngle = surfacePoint.SurfaceAngle;
            
            // Blend between velocity and surface angle based on speed
            float speed = MathF.Sqrt(obj.VX * obj.VX + obj.VY * obj.VY);
            float velocityWeight = MathF.Min(1f, speed / 5f);
            
            return velocityAngle * velocityWeight + surfaceAngle * (1 - velocityWeight);
        }

        private static float CalculateRotationStrength(PhysicsObject obj, SurfacePoint surfacePoint)
        {
            // Dynamic rotation strength based on:
            // 1. Speed (faster = weaker rotation)
            // 2. Distance from surface (further = stronger rotation)
            // 3. Is corner (corners = stronger rotation)
            
            float speedFactor = 1f / (1f + MathF.Sqrt(obj.VX * obj.VX + obj.VY * obj.VY));
            float distanceFactor = surfacePoint.DistanceFromSurface / MAX_SNAP_DISTANCE;
            float cornerFactor = surfacePoint.IsCorner ? 2f : 1f;
            
            return 5f * speedFactor * distanceFactor * cornerFactor;
        }
        
        public static SurfacePoint AnalyzeSurface(Collider surface, (float X, float Y) contactPoint)
        {
            var vertices = surface.GetTransformedVertices();
            var closestEdge = FindClosestEdge(vertices, contactPoint);
            var normal = CalculateNormal(closestEdge);
            var angle = MathF.Atan2(normal.Y, normal.X);

            return new SurfacePoint
            {
                Position = contactPoint,
                Normal = normal,
                SurfaceAngle = angle,
                IsGrounded = normal.Y < -0.1f,
                Friction = surface.ConnectedObject?.Config?.GroundFriction ?? 0.1f,
                IsLoop = false, // Default to false
                CurveRadius = 0, // Default to 0
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
        
        private static float CalculateSurfaceAngle((float X, float Y) normal)
        {
            return MathF.Atan2(normal.Y, normal.X);
        }

        private static float CalculateCurveRadius((float X, float Y)[] vertices, 
            ((float X, float Y) Start, (float X, float Y) End) currentEdge)
        {
            // Find the angle between the current edge and adjacent edges
            int edgeCount = vertices.Length;
            for (int i = 0; i < edgeCount; i++)
            {
                if (vertices[i] == currentEdge.Start)
                {
                    var prevVertex = vertices[(i - 1 + edgeCount) % edgeCount];
                    var nextVertex = vertices[(i + 2) % edgeCount];
                    
                    // Calculate vectors
                    var v1 = (
                        X: currentEdge.End.X - currentEdge.Start.X,
                        Y: currentEdge.End.Y - currentEdge.Start.Y
                    );
                    var v2 = (
                        X: nextVertex.X - currentEdge.End.X,
                        Y: nextVertex.Y - currentEdge.End.Y
                    );
                    
                    // Calculate angle
                    float dot = v1.X * v2.X + v1.Y * v2.Y;
                    float mag1 = MathF.Sqrt(v1.X * v1.X + v1.Y * v1.Y);
                    float mag2 = MathF.Sqrt(v2.X * v2.X + v2.Y * v2.Y);
                    float angle = MathF.Acos(dot / (mag1 * mag2));
                    
                    // Curvature is inverse of radius
                    float curvature = angle / ((mag1 + mag2) / 2);
                    return curvature > CURVE_DETECTION_THRESHOLD ? 1f / curvature : 0;
                }
            }
            return 0;
        }

        private static bool DetectCorner((float X, float Y)[] vertices, 
            ((float X, float Y) Start, (float X, float Y) End) currentEdge)
        {
            // Find adjacent edges and check their angles
            int edgeCount = vertices.Length;
            for (int i = 0; i < edgeCount; i++)
            {
                if (vertices[i] == currentEdge.Start)
                {
                    var prevVertex = vertices[(i - 1 + edgeCount) % edgeCount];
                    var nextVertex = vertices[(i + 2) % edgeCount];
                    
                    // Calculate angles between adjacent edges
                    var v1 = (
                        X: currentEdge.End.X - currentEdge.Start.X,
                        Y: currentEdge.End.Y - currentEdge.Start.Y
                    );
                    var v2 = (
                        X: nextVertex.X - currentEdge.End.X,
                        Y: nextVertex.Y - currentEdge.End.Y
                    );
                    
                    float angle = MathF.Abs(MathF.Atan2(
                        v1.X * v2.Y - v1.Y * v2.X,
                        v1.X * v2.X + v1.Y * v2.Y
                    ));
                    
                    return angle > CORNER_ANGLE_THRESHOLD;
                }
            }
            return false;
        }

        private static bool DetectLoop((float X, float Y)[] vertices,
            ((float X, float Y) Start, (float X, float Y) End) currentEdge)
        {
            // We'll consider it a loop if:
            // 1. There are at least 4 vertices
            // 2. The curvature is relatively constant
            // 3. The shape eventually returns to near the starting point
            
            if (vertices.Length < 4) return false;
            
            // Calculate average curvature
            float totalCurvature = 0;
            int curveCount = 0;
            
            for (int i = 0; i < vertices.Length; i++)
            {
                var edge = (
                    Start: vertices[i],
                    End: vertices[(i + 1) % vertices.Length]
                );
                float radius = CalculateCurveRadius(vertices, edge);
                if (radius > 0)
                {
                    totalCurvature += 1f / radius;
                    curveCount++;
                }
            }
            
            if (curveCount < 3) return false; // Need at least 3 curved sections
            
            // Check if shape returns to start
            var start = vertices[0];
            var end = vertices[vertices.Length - 1];
            float distance = MathF.Sqrt(
                MathF.Pow(end.X - start.X, 2) + 
                MathF.Pow(end.Y - start.Y, 2)
            );
            
            return distance < (vertices.Length * CURVE_DETECTION_THRESHOLD);
        }

        private static float CalculateDistanceFromSurface(
            (float X, float Y) point,
            ((float X, float Y) Start, (float X, float Y) End) edge)
        {
            float dx = edge.End.X - edge.Start.X;
            float dy = edge.End.Y - edge.Start.Y;
            float magnitude = MathF.Sqrt(dx * dx + dy * dy);
            
            if (magnitude == 0) return 0;
            
            // Calculate perpendicular distance
            return MathF.Abs(
                (point.Y - edge.Start.Y) * dx - 
                (point.X - edge.Start.X) * dy
            ) / magnitude;
        }
        
        private static ((float X, float Y) Start, (float X, float Y) End) FindNearestEdge(
            (float X, float Y) point, (float X, float Y)[] vertices)
        {
            var closestDist = float.MaxValue;
            var closestEdge = default(((float X, float Y) Start, (float X, float Y) End));

            for (int i = 0; i < vertices.Length; i++)
            {
                var start = vertices[i];
                var end = vertices[(i + 1) % vertices.Length];

                // Calculate distance from point to current edge
                float dist = DistanceToLineSegment(point, start, end);

                if (dist < closestDist)
                {
                    closestDist = dist;
                    closestEdge = (Start: start, End: end);
                }
            }

            return closestEdge;
        }

        private static float DistanceToLineSegment(
            (float X, float Y) point,
            (float X, float Y) lineStart,
            (float X, float Y) lineEnd)
        {
            float dx = lineEnd.X - lineStart.X;
            float dy = lineEnd.Y - lineStart.Y;
            float lengthSquared = dx * dx + dy * dy;

            if (lengthSquared == 0)
                return Distance(point, lineStart);

            // Calculate projection of point onto line
            float t = ((point.X - lineStart.X) * dx + (point.Y - lineStart.Y) * dy) / lengthSquared;
            t = Math.Clamp(t, 0, 1);

            // Calculate closest point on line segment
            float projX = lineStart.X + t * dx;
            float projY = lineStart.Y + t * dy;

            // Return distance to closest point
            return Distance(point, (projX, projY));
        }

        private static float Distance((float X, float Y) a, (float X, float Y) b)
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            return MathF.Sqrt(dx * dx + dy * dy);
        }
    }
}