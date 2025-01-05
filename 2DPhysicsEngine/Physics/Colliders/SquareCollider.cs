using ZephyrRenderer.Platform;

namespace _2DPhysicsEngine
{
    public class SquareCollider : Collider
    {
        public float Size { get; private set; } // Side length of the square

        public SquareCollider(PhysicsObject connectedObject, float size, (float OffsetX, float OffsetY)? offset = null) 
            : base(connectedObject)
        {
            Size = size;
            Offset = offset ?? (0, 0); // Default to no offset
        }

        public override RECT GetBoundingBox()
        {
            float halfSize = Size / 2;

            return new RECT(
                ConnectedObject.X + Offset.OffsetX - halfSize,
                ConnectedObject.Y + Offset.OffsetY - halfSize,
                Size,
                Size
            );
        }

        public override (float X, float Y)[] GetTransformedVertices()
        {
            var transformedVertices = new (float X, float Y)[4];
            float halfSize = Size / 2;

            // Define local vertices for a square centered at (0, 0)
            var localVertices = new[]
            {
                (-halfSize, -halfSize),
                (halfSize, -halfSize),
                (halfSize, halfSize),
                (-halfSize, halfSize)
            };

            // Apply rotation, offset, and translation
            float cosAngle = MathF.Cos(ConnectedObject.Rotation);
            float sinAngle = MathF.Sin(ConnectedObject.Rotation);

            for (int i = 0; i < localVertices.Length; i++)
            {
                float localX = localVertices[i].Item1;
                float localY = localVertices[i].Item2;

                // Apply rotation
                float rotatedX = localX * cosAngle - localY * sinAngle;
                float rotatedY = localX * sinAngle + localY * cosAngle;

                // Apply offset and translation
                transformedVertices[i] = (
                    X: rotatedX + ConnectedObject.X + Offset.OffsetX,
                    Y: rotatedY + ConnectedObject.Y + Offset.OffsetY
                );
            }

            return transformedVertices;
        }

        /// <summary>
        /// Adjust the position of the collider relative to the connected object.
        /// </summary>
        /// <param name="offsetX">Offset in the X direction.</param>
        /// <param name="offsetY">Offset in the Y direction.</param>
        public void SetOffset(float offsetX, float offsetY)
        {
            Offset = (offsetX, offsetY);
        }
    }
}