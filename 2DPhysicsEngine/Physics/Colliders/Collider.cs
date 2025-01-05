using ZephyrRenderer.Platform;

namespace _2DPhysicsEngine
{
    public abstract class Collider
    {
        public (float OffsetX, float OffsetY) Offset { get; set; } // Offset of the collider from the object center
        public PhysicsObject ConnectedObject { get; set; }
        public bool DebugMode { get; set; } = false;

        public Collider(PhysicsObject connectedObject)
        {
            ConnectedObject = connectedObject;
        }

        // General collision handling
        public virtual bool CheckCollision(Collider other)
        {
            if (DebugMode)
                DebugCollision(other);

            return !CollisionHelper.IsSeparated(this, other);
        }

        public virtual void ResolveCollision(Collider other)
        {
            if (!CheckCollision(other)) return;

            var mtv = CollisionHelper.FindMTV(this, other);
            if (mtv == null) return;

            CollisionHelper.SeparateShapes(this, other, mtv.Value);
            CollisionHelper.ApplyImpulse(this, other, mtv.Value);
            CollisionHelper.ApplyFriction(this, other, CollisionHelper.Normalize(mtv.Value));
        }

        // Specific handlers for cross-shape collisions (override if needed)
        public virtual bool CheckCollisionWith(SquareCollider square)
        {
            return CheckCollision(square);
        }

        public virtual bool CheckCollisionWith(RectangleCollider rectangle)
        {
            return CheckCollision(rectangle);
        }

        public virtual bool CheckCollisionWith(PentagonCollider pentagon)
        {
            return CheckCollision(pentagon);
        }

        public virtual bool CheckCollisionWith(TriangleCollider triangle)
        {
            return CheckCollision(triangle);
        }
        
        public abstract RECT GetBoundingBox();

        // Abstract method for shape-specific vertex transformations
        public abstract (float X, float Y)[] GetTransformedVertices();

        private void DebugCollision(Collider other)
        {
            Console.WriteLine($"Checking collision between {this.GetType().Name} and {other.GetType().Name}");
            Console.WriteLine($"Position: {ConnectedObject.X}, {ConnectedObject.Y}");
        }
    }
}