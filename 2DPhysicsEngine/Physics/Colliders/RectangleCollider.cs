using ZephyrRenderer.Platform;

namespace _2DPhysicsEngine
{
    public class RectangleCollider : Collider
    {
        public float Width { get; private set; }
        public float Height { get; private set; }

        public RectangleCollider(PhysicsObject connectedObject, float width, float height, (float OffsetX, float OffsetY)? offset = null) : base(connectedObject)
        {
            Width = width;
            Height = height;
            Offset = offset ?? (0, 0); // Default to no offset
        }
        
        public override RECT GetBoundingBox()
        {
            float halfWidth = Width / 2;
            float halfHeight = Height / 2;

            return new RECT(
                ConnectedObject.X - halfWidth,
                ConnectedObject.Y - halfHeight,
                Width,
                Height
            );
        }

        
        public override (float X, float Y)[] GetTransformedVertices()
        {
            var transformedVertices = new (float X, float Y)[4];
            float halfWidth = Width / 2;
            float halfHeight = Height / 2;

            // Define local vertices for a rectangle centered at (0, 0)
            var localVertices = new[]
            {
                (-halfWidth, -halfHeight),
                (halfWidth, -halfHeight),
                (halfWidth, halfHeight),
                (-halfWidth, halfHeight)
            };

            // Apply rotation and translation
            float cosAngle = MathF.Cos(ConnectedObject.Rotation);
            float sinAngle = MathF.Sin(ConnectedObject.Rotation);

            for (int i = 0; i < localVertices.Length; i++)
            {
                float localX = localVertices[i].Item1;
                float localY = localVertices[i].Item2;

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