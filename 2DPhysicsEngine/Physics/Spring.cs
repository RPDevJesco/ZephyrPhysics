namespace _2DPhysicsEngine
{
    public class Spring
    {
        public PhysicsObject A { get; set; }
        public PhysicsObject B { get; set; }
        public float RestLength { get; set; }
        public float Stiffness { get; set; }

        public void ApplySpringForce()
        {
            float dx = B.X - A.X;
            float dy = B.Y - A.Y;
            float distance = MathF.Sqrt(dx * dx + dy * dy);
            float forceMagnitude = Stiffness * (distance - RestLength);

            float fx = (dx / distance) * forceMagnitude;
            float fy = (dy / distance) * forceMagnitude;

            A.ApplyForce(fx, fy);
            B.ApplyForce(-fx, -fy);
        }
    }
}