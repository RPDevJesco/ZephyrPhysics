using ZephyrRenderer.Platform;

namespace _2DPhysicsEngine
{
    public class PhysicsEnvironment
    {
        public (float X, float Y) GlobalGravity { get; set; } = (0, 9.81f);
        public (float X, float Y) WindForce { get; set; } = (0, 0);
        public List<GravityZone> GravityZones { get; private set; } = new();

        public void AddGravityZone(GravityZone zone) => GravityZones.Add(zone);
        
        public (float X, float Y) CalculateGravityAt(float x, float y)
        {
            var activeZone = GravityZones.FirstOrDefault(z => z.Contains(x, y));
            return activeZone?.GravityVector ?? GlobalGravity;
        }

        public (float X, float Y) CalculateWindAt(float x, float y)
        {
            // Can be extended for local wind zones
            return WindForce;
        }
    }

    public class GravityZone
    {
        public RECT Bounds { get; set; }
        public (float X, float Y) GravityVector { get; set; }
        public float TransitionSmoothing { get; set; } = 0.1f;

        public bool Contains(float x, float y) => 
            x >= Bounds.X && x <= Bounds.X + Bounds.Width && 
            y >= Bounds.Y && y <= Bounds.Y + Bounds.Height;
    }
}