namespace _2DPhysicsEngine
{
    public static class CollisionHelper
    {
        public static bool IsSeparated(Collider a, Collider b)
        {
            foreach (var axis in GetAxes(a).Concat(GetAxes(b)))
            {
                // Project both colliders onto the axis
                var (minA, maxA) = ProjectOntoAxis(a, axis);
                var (minB, maxB) = ProjectOntoAxis(b, axis);

                // Check for separation
                if (maxA < minB || maxB < minA)
                {
                    return true; // Separated on this axis
                }
            }
            return false; // No separation found
        }

        public static List<(float X, float Y)> GetAxes(Collider collider)
        {
            if (collider == null)
                return new List<(float X, float Y)>();

            if (collider is PentagonCollider pentagon)
            {
                var vertices = pentagon.GetTransformedVertices();
                if (vertices == null || vertices.Length == 0)
                    throw new InvalidOperationException("Pentagon collider must have vertices.");
                return CalculateAxes(vertices);
            }
            if (collider is RectangleCollider rectangle)
            {
                var vertices = rectangle.GetTransformedVertices();
                if (vertices == null || vertices.Length == 0)
                    throw new InvalidOperationException("Rectangle collider must have vertices.");
                return CalculateAxes(vertices);
            }
            if (collider is SquareCollider square)
            {
                var vertices = square.GetTransformedVertices();
                if (vertices == null || vertices.Length == 0)
                    throw new InvalidOperationException("Square collider must have vertices.");
                return CalculateAxes(vertices);
            }
            if (collider is TriangleCollider triangle)
            {
                var vertices = triangle.GetTransformedVertices();
                if (vertices == null || vertices.Length == 0)
                    throw new InvalidOperationException("Triangle collider must have vertices.");
                return CalculateAxes(vertices);
            }

            throw new NotSupportedException($"Collider type '{collider.GetType()}' not supported for axis generation.");
        }

        private static List<(float X, float Y)> CalculateAxes((float X, float Y)[] vertices)
        {
            var axes = new List<(float X, float Y)>();

            for (int i = 0; i < vertices.Length; i++)
            {
                // Edge vector
                var p1 = vertices[i];
                var p2 = vertices[(i + 1) % vertices.Length]; // Wrap to the first vertex
                var edge = (X: p2.X - p1.X, Y: p2.Y - p1.Y);

                // Perpendicular vector (axis)
                var axis = (X: -edge.Y, Y: edge.X);
                axes.Add(Normalize(axis));
            }

            return axes;
        }

        public static (float Min, float Max) ProjectOntoAxis(Collider collider, (float X, float Y) axis)
        {
            // Get transformed vertices from the collider
            var vertices = collider switch
            {
                PentagonCollider pentagon => pentagon.GetTransformedVertices(),
                SquareCollider square => square.GetTransformedVertices(),
                RectangleCollider rectangle => rectangle.GetTransformedVertices(),
                TriangleCollider triangle => triangle.GetTransformedVertices(),
                _ => null // If no vertices are defined, return default
            };

            if (vertices == null)
                throw new NotSupportedException("Collider type not supported for projection.");

            float min = float.MaxValue, max = float.MinValue;

            foreach (var vertex in vertices)
            {
                float projection = vertex.X * axis.X + vertex.Y * axis.Y;
                min = Math.Min(min, projection);
                max = Math.Max(max, projection);
            }

            return (min, max);
        }

        public static (float X, float Y)? FindMTV(Collider a, Collider b)
        {
            (float X, float Y)? mtv = null;
            float minOverlap = float.MaxValue;

            foreach (var axis in GetAxes(a).Concat(GetAxes(b)))
            {
                // Project both shapes onto the axis
                var (minA, maxA) = ProjectOntoAxis(a, axis);
                var (minB, maxB) = ProjectOntoAxis(b, axis);

                // Calculate overlap
                float overlap = MathF.Min(maxA, maxB) - MathF.Max(minA, minB);
                if (overlap <= 0)
                {
                    return null; // No collision
                }

                // Keep track of the smallest overlap
                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    mtv = Normalize(axis); // Store the axis as the MTV direction
                }
            }

            // Scale the MTV direction by the overlap to get the final MTV
            return mtv.HasValue ? (mtv.Value.X * minOverlap, mtv.Value.Y * minOverlap) : null;
        }

        public static void SeparateShapes(Collider a, Collider b, (float X, float Y) mtv)
        {
            // Calculate the total inverse mass
            float totalInverseMass = (1 / a.ConnectedObject.Mass) + (1 / b.ConnectedObject.Mass);

            // Calculate the displacement for each object
            float displacementA = 1 / a.ConnectedObject.Mass / totalInverseMass;
            float displacementB = 1 / b.ConnectedObject.Mass / totalInverseMass;

            // Move the objects
            a.ConnectedObject.X -= mtv.X * displacementA;
            a.ConnectedObject.Y -= mtv.Y * displacementA;
            b.ConnectedObject.X += mtv.X * displacementB;
            b.ConnectedObject.Y += mtv.Y * displacementB;
        }

        public static void ApplyImpulse(Collider a, Collider b, (float X, float Y) mtv)
        {
            // Normalize the MTV to get the collision normal
            var normal = Normalize(mtv);

            // Relative velocity
            float rvx = b.ConnectedObject.VX - a.ConnectedObject.VX;
            float rvy = b.ConnectedObject.VY - a.ConnectedObject.VY;

            // Calculate relative velocity along the normal
            float velocityAlongNormal = rvx * normal.X + rvy * normal.Y;

            // Do not resolve if objects are separating
            if (velocityAlongNormal > 0) return;

            // Calculate restitution (elasticity)
            float restitution = 0.8f; // Example value, can vary per object

            // Impulse scalar
            float impulse = -(1 + restitution) * velocityAlongNormal;
            impulse /= (1 / a.ConnectedObject.Mass) + (1 / b.ConnectedObject.Mass);

            // Apply impulse to each object
            float impulseX = impulse * normal.X;
            float impulseY = impulse * normal.Y;

            a.ConnectedObject.VX -= impulseX / a.ConnectedObject.Mass;
            a.ConnectedObject.VY -= impulseY / a.ConnectedObject.Mass;

            b.ConnectedObject.VX += impulseX / b.ConnectedObject.Mass;
            b.ConnectedObject.VY += impulseY / b.ConnectedObject.Mass;

            // Apply friction
            ApplyFriction(a, b, normal);
        }

        public static void ApplyFriction(Collider a, Collider b, (float X, float Y) normal)
        {
            // Relative velocity
            float rvx = b.ConnectedObject.VX - a.ConnectedObject.VX;
            float rvy = b.ConnectedObject.VY - a.ConnectedObject.VY;

            // Calculate the tangent vector (perpendicular to the collision normal)
            var tangent = Normalize((-normal.Y, normal.X));

            // Project the relative velocity onto the tangent
            float velocityAlongTangent = rvx * tangent.X + rvy * tangent.Y;

            // Coefficient of friction (can vary per object, for now using a constant)
            float frictionCoefficient = 0.2f;

            // Impulse magnitude for friction
            float frictionImpulse = -velocityAlongTangent;
            frictionImpulse /= (1 / a.ConnectedObject.Mass) + (1 / b.ConnectedObject.Mass);

            // Clamp the friction impulse to a fraction of the collision impulse (Coulomb's law)
            float collisionImpulse = MathF.Abs(velocityAlongTangent); // Approximation
            frictionImpulse = Math.Clamp(frictionImpulse, -frictionCoefficient * collisionImpulse, frictionCoefficient * collisionImpulse);

            // Apply the friction impulse to the objects
            float impulseX = frictionImpulse * tangent.X;
            float impulseY = frictionImpulse * tangent.Y;

            a.ConnectedObject.VX -= impulseX / a.ConnectedObject.Mass;
            a.ConnectedObject.VY -= impulseY / a.ConnectedObject.Mass;

            b.ConnectedObject.VX += impulseX / b.ConnectedObject.Mass;
            b.ConnectedObject.VY += impulseY / b.ConnectedObject.Mass;
        }

        public static (float X, float Y) Normalize((float X, float Y) vector)
        {
            float magnitude = MathF.Sqrt(vector.X * vector.X + vector.Y * vector.Y);
            return (X: vector.X / magnitude, Y: vector.Y / magnitude);
        }
    }
}