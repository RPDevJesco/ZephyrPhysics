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

        public PhysicsObject(float x, float y, float mass, float friction = 0.1f, float drag = 0.01f)
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
        }

        public void ApplyForce(float fx, float fy)
        {
            AX += fx / Mass;
            AY += fy / Mass;
        }

        public void ApplyTorque(float torque)
        {
            AngularAcceleration += torque / MomentOfInertia;
        }

        public void Update(float deltaTime)
        {
            // Linear motion
            VX += AX * deltaTime;
            VY += AY * deltaTime;

            VX *= (1 - Friction * deltaTime); // Apply friction
            VY *= (1 - Friction * deltaTime);

            X += VX * deltaTime;
            Y += VY * deltaTime;

            AX = 0;
            AY = 0;

            // Rotational motion
            AngularVelocity += AngularAcceleration * deltaTime;
            Rotation += AngularVelocity * deltaTime;

            AngularAcceleration = 0; // Reset angular acceleration
        }
    }
}