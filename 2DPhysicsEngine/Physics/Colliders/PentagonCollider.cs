using ZephyrRenderer.Platform;

namespace _2DPhysicsEngine
{
    public class PentagonCollider : Collider
    {
        public (float X, float Y)[] Vertices { get; private set; }

        public PentagonCollider(PhysicsObject connectedObject, (float X, float Y)[] vertices) : base(connectedObject)
        {
            if (vertices.Length != 5)
                throw new ArgumentException("A pentagon must have exactly 5 vertices.");
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
            var transformedVertices = new (float X, float Y)[Vertices.Length];

            for (int i = 0; i < Vertices.Length; i++)
            {
                // Apply translation from the connected object
                transformedVertices[i] = (
                    X: Vertices[i].X + ConnectedObject.X,
                    Y: Vertices[i].Y + ConnectedObject.Y
                );
            }

            return transformedVertices;
        }
    }
}