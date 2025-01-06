namespace _2DPhysicsEngine
{
    public class PhysicsObject
    {
        public float X { get; set; }
        public float Y { get; set; }
        public float VX { get; set; } // Velocity X
        public float VY { get; set; } // Velocity Y
        public float AX { get; set; } // Acceleration X
        public float AY { get; set; } // Acceleration Y
        public float Mass { get; set; }
        public float Friction { get; set; } // Coefficient of friction
        public float Drag { get; set; }     // Air resistance
        
        public float Rotation { get; set; } // In degrees or radians
        public float AngularVelocity { get; set; }
        public float AngularAcceleration { get; set; }
        public float MomentOfInertia { get; set; }
        
        public PhysicsEnvironment Environment { get; set; }
        public SurfacePoint CurrentSurface { get; private set; }
        public bool AutoOrient { get; set; } = true;
        public Collider AttachedCollider { get; set; }
        public PhysicsConfig Config { get; set; }
        public CollisionLayer Layer { get; set; }
        public bool IsGrounded { get; private set; }
        public bool IsSleeping { get; private set; }

        public PhysicsObject(float x, float y, float mass, PhysicsEnvironment environment, float friction = 0.1f, float drag = 0.01f)
        {
            X = x;
            Y = y;
            Mass = mass;
            Friction = friction;
            Drag = drag;
            VX = 0;
            VY = 0;
            AX = 0;
            AY = 0;
            Rotation = 0;
            AngularVelocity = 0;
            AngularAcceleration = 0;
            MomentOfInertia = 1; // Default value to prevent division by zero
            Environment = environment;
            Layer = new CollisionLayer();
            ResetMotion();
        }
        
        public void ResetMotion()
        {
            VX = VY = AX = AY = 0;
            AngularVelocity = AngularAcceleration = 0;
        }

        public void Update(float deltaTime)
        {
            if (IsSleeping) return;

            // Update surface point and orientation
            if (AttachedCollider != null)
            {
                CurrentSurface = OrientationHelper.FindBottomPoint(this, AttachedCollider);
                if (AutoOrient)
                {
                    OrientationHelper.AlignWithSurface(this, CurrentSurface);
                }
            }

            // Apply environmental forces
            var gravity = Environment.CalculateGravityAt(X, Y);
            ApplyForce(
                gravity.X * Mass * Config.GravityScale,
                gravity.Y * Mass * Config.GravityScale
            );

            var wind = Environment.CalculateWindAt(X, Y);
            ApplyForce(wind.X, wind.Y);

            // Update velocities with acceleration caps
            VX += MathF.Min(MathF.Max(AX * deltaTime, -Config.MaxAccelerationX), Config.MaxAccelerationX);
            VY += MathF.Min(MathF.Max(AY * deltaTime, -Config.MaxAccelerationY), Config.MaxAccelerationY);

            // Apply velocity caps
            VX = MathF.Min(MathF.Max(VX, -Config.MaxVelocityX), Config.MaxVelocityX);
            VY = MathF.Min(MathF.Max(VY, -Config.MaxVelocityY), Config.MaxVelocityY);

            // Apply friction
            float frictionCoef = IsGrounded ? Config.GroundFriction : Config.AirFriction;
            if (CurrentSurface != null)
            {
                frictionCoef *= CurrentSurface.Friction;
            }

            VX *= (1 - frictionCoef * deltaTime);
            VY *= (1 - frictionCoef * deltaTime);

            // Update position
            X += VX * deltaTime;
            Y += VY * deltaTime;

            // Update rotation
            AngularVelocity += AngularAcceleration * deltaTime;
            Rotation += AngularVelocity * deltaTime;

            // Reset accelerations
            AX = AY = 0;
            AngularAcceleration = 0;

            // Check for sleep state
            if (MathF.Abs(VX) < 0.01f && MathF.Abs(VY) < 0.01f &&
                MathF.Abs(AngularVelocity) < 0.01f)
            {
                IsSleeping = true;
            }
        }

        public void ApplyForce(float fx, float fy) 
        {
            AX += fx / Mass;
            AY += fy / Mass;
            IsSleeping = false;
        }

        public void ApplyTorque(float torque)
        {
            AngularAcceleration += torque / MomentOfInertia;
        }
        
        public void UpdateSurfaceInfo(Collider surface, (float X, float Y) contactPoint)
        {
            CurrentSurface = OrientationHelper.AnalyzeSurface(surface, contactPoint);
            IsGrounded = CurrentSurface.Normal.Y < -0.1f; // Assuming Y-down coordinate system
        }
    }
}