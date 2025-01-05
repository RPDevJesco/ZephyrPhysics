using ZephyrRenderer.Platform;

namespace _2DPhysicsEngine
{
    public class TriangleCollider : Collider
    {
        public (float X, float Y)[] Vertices { get; private set; } // Array of 3 vertices

        public TriangleCollider(PhysicsObject connectedObject, (float X, float Y)[] vertices) : base(connectedObject)
        {
            if (vertices.Length != 3)
                throw new ArgumentException("A triangle must have exactly 3 vertices.");
            Vertices = vertices;
        }
        
        public override RECT GetBoundingBox()
        {
            var vertices = GetTransformedVertices();

            float minX = vertices.Min(v => v.X);
            float minY = vertices.Min(v => v.Y);
            float maxX = vertices.Max(v => v.X);
            float maxY = vertices.Max(v => v.Y);

            return new RECT(minX, minY, maxX - minX, maxY - minY);
        }

        
        public override (float X, float Y)[] GetTransformedVertices()
        {
            var transformedVertices = new (float X, float Y)[3];

            // Apply rotation and translation
            float cosAngle = MathF.Cos(ConnectedObject.Rotation);
            float sinAngle = MathF.Sin(ConnectedObject.Rotation);

            for (int i = 0; i < Vertices.Length; i++)
            {
                float localX = Vertices[i].X;
                float localY = Vertices[i].Y;

                // Apply rotation
                float rotatedX = localX * cosAngle - localY * sinAngle;
                float rotatedY = localX * sinAngle + localY * cosAngle;

                // Apply translation
                transformedVertices[i] = (
                    X: rotatedX + ConnectedObject.X,
                    Y: rotatedY + ConnectedObject.Y
                );
            }

            return transformedVertices;
        }
    }
}