namespace _2DPhysicsEngine 
{
    public class PhysicsConfig
    {
        public float MaxVelocityX { get; set; } = 20.0f;
        public float MaxVelocityY { get; set; } = 20.0f;
        public float MaxAccelerationX { get; set; } = 15.0f;
        public float MaxAccelerationY { get; set; } = 15.0f;
        public float AirFriction { get; set; } = 0.01f;
        public float GroundFriction { get; set; } = 0.1f;
        public float GravityScale { get; set; } = 1.0f;
        public float BounceThreshold { get; set; } = 0.2f;
        public float SurfaceAngleLimit { get; set; } = 45.0f;
    }

    public class CollisionLayer
    {
        public int Layer { get; set; }
        public int Mask { get; set; } = -1;
        public bool OneWay { get; set; }
        public float FrictionMultiplier { get; set; } = 1.0f;
        public bool TransferVelocity { get; set; }
    }
}