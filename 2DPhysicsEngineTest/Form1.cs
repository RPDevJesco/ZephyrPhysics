using _2DPhysicsEngine;
using ZephyrRenderer.Platform;
using Timer = System.Windows.Forms.Timer;

namespace _2DPhysicsEngineTest
{
    public partial class Form1 : Form
    {
        private Timer _timer;
        private Bitmap _bitmap;
        private Sprite _sprite;
        private Sprite _anotherSprite;
        private Quadtree _quadTree;
        private float _spriteScaleFactor;
        private float _anotherSpriteScaleFactor;

        public Form1()
        {
            this.Text = "2D Physics Engine";
            this.Width = 800;
            this.Height = 600;
            this.DoubleBuffered = true;
            
            var env = new PhysicsEnvironment();
            env.GlobalGravity = (0, 9.81f);
            env.WindForce = (2.0f, 0);

            // Create zones with different physics properties
            env.AddGravityZone(new GravityZone {
                Bounds = new RECT(100, 400, 100, 100),
                GravityVector = (0, -13.81f),
                TransitionSmoothing = 0.2f
            });
            
            _bitmap = new Bitmap(800, 600);
            _quadTree = new Quadtree(0, new RECT(0, 0, 800, 600));

            string desktopPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);

            // Initialize first sprite with enhanced physics
            string instaFilePath = Path.Combine(desktopPath, "insta.png");
            _sprite = Sprite.FromImage(instaFilePath);
            var spriteConfig = new PhysicsConfig {
                MaxVelocityX = 15.0f,
                MaxVelocityY = 20.0f,
                MaxAccelerationX = 10.0f,
                MaxAccelerationY = 15.0f,
                GravityScale = 1.2f,
                GroundFriction = 0.15f,
                AirFriction = 0.01f,
                BounceThreshold = 0.2f,
                SurfaceAngleLimit = 45.0f
            };
            
            var spritePhysics = new PhysicsObject(100, 100, 1, env)
            {
                Config = spriteConfig,
                AutoOrient = true,
                VX = 2,
                VY = 1
            };
            _sprite.AttachPhysics(spritePhysics);
            _sprite.AttachCollider(new SquareCollider(spritePhysics, 200, new ValueTuple<float, float>(100,100)));

            // Initialize second sprite with different physics properties
            string diceFilePath = Path.Combine(desktopPath, "dice.png");
            _anotherSprite = Sprite.FromImage(diceFilePath);
            var anotherConfig = new PhysicsConfig {
                MaxVelocityX = 10.0f,
                MaxVelocityY = 15.0f,
                MaxAccelerationX = 8.0f,
                MaxAccelerationY = 12.0f,
                GravityScale = 1.5f,
                GroundFriction = 0.2f,
                AirFriction = 0.02f,
                BounceThreshold = 0.3f,
                SurfaceAngleLimit = 30.0f
            };
            
            var anotherSpritePhysics = new PhysicsObject(100, 400, 10, env)
            {
                Config = anotherConfig,
                AutoOrient = true,
                Layer = new CollisionLayer {
                    Layer = 2,
                    Mask = 1,
                    FrictionMultiplier = 1.2f,
                    TransferVelocity = true
                }
            };
            _anotherSprite.AttachPhysics(anotherSpritePhysics);
            _anotherSprite.AttachCollider(new SquareCollider(anotherSpritePhysics, 150, new ValueTuple<float, float>(70,50)));

            _quadTree.Insert(_sprite.Collider);
            _quadTree.Insert(_anotherSprite.Collider);

            _spriteScaleFactor = CalculateScaleFactor(_sprite.Width, _sprite.Height, 200, 200);
            _anotherSpriteScaleFactor = CalculateScaleFactor(_anotherSprite.Width, _anotherSprite.Height, 150, 150);

            _timer = new Timer { Interval = 16 };
            _timer.Tick += Update;
            _timer.Start();

            // Add keyboard input handling
            this.KeyPreview = true;
            this.KeyDown += HandleKeyDown;
        }

        private void HandleKeyDown(object sender, KeyEventArgs e)
        {
            float moveForce = 50.0f;
            var obj = _sprite.Collider.ConnectedObject;

            switch (e.KeyCode)
            {
                case Keys.Left:
                    obj.ApplyForce(-moveForce, 0);
                    break;
                case Keys.Right:
                    obj.ApplyForce(moveForce, 0);
                    break;
                case Keys.Up:
                    obj.ApplyForce(0, -moveForce);
                    break;
                case Keys.Down:
                    obj.ApplyForce(0, moveForce);
                    break;
                case Keys.Space:
                    // Apply torque for rotation
                    obj.ApplyTorque(10.0f);
                    break;
            }
        }

        private void Update(object sender, EventArgs e)
        {
            _quadTree.Clear();
            if (_sprite.Collider != null) _quadTree.Insert(_sprite.Collider);
            if (_anotherSprite.Collider != null) _quadTree.Insert(_anotherSprite.Collider);
            
            // Update physics with surface interactions
            float deltaTime = 0.016f;
            UpdateObject(_sprite.Collider.ConnectedObject, deltaTime);
            UpdateObject(_anotherSprite.Collider.ConnectedObject, deltaTime);

            // Handle collisions
            HandleCollisions();

            RenderFrame();
            this.Invalidate();
        }

        private void UpdateObject(PhysicsObject obj, float deltaTime)
        {
            // Perform physics update
            obj.Update(deltaTime);

            // Handle screen boundaries with surface orientation
            HandleScreenBoundaries(obj);
        }

        private void HandleScreenBoundaries(PhysicsObject obj)
        {
            bool collision = false;
            (float X, float Y) normal = (0, 0);
            (float X, float Y) point = (0, 0);

            // Check each boundary and find collision point and normal
            if (obj.X < 0)
            {
                collision = true;
                normal = (1, 0);
                point = (0, obj.Y);
                obj.X = 0;
            }
            else if (obj.X > _bitmap.Width)
            {
                collision = true;
                normal = (-1, 0);
                point = (_bitmap.Width, obj.Y);
                obj.X = _bitmap.Width;
            }

            if (obj.Y < 0)
            {
                collision = true;
                normal = (0, 1);
                point = (obj.X, 0);
                obj.Y = 0;
            }
            else if (obj.Y > _bitmap.Height)
            {
                collision = true;
                normal = (0, -1);
                point = (obj.X, _bitmap.Height);
                obj.Y = _bitmap.Height;
            }

            if (collision)
            {
                // Create a surface point for the boundary
                var surfacePoint = new SurfacePoint
                {
                    Position = point,
                    Normal = normal,
                    IsGrounded = normal.Y < -0.1f,
                    Friction = 0.1f
                };

                // Update object's surface info
                obj.UpdateSurfaceInfo(obj.AttachedCollider, point);

                // Reflect velocity with dampening
                float dampening = 0.8f;
                obj.VX = normal.X == 0 ? obj.VX : -obj.VX * dampening;
                obj.VY = normal.Y == 0 ? obj.VY : -obj.VY * dampening;
            }
        }

        private void HandleCollisions()
        {
            var potentialCollisions = _quadTree.Retrieve(_sprite.Collider);
            foreach (var collider in potentialCollisions)
            {
                if (_sprite.Collider.CheckCollision(collider))
                {
                    _sprite.Collider.ResolveCollision(collider);
                    
                    // Update surface information for both objects
                    var contactPoint = (_sprite.Collider.ConnectedObject.X, _sprite.Collider.ConnectedObject.Y);
                    _sprite.Collider.ConnectedObject.UpdateSurfaceInfo(collider, contactPoint);
                    collider.ConnectedObject.UpdateSurfaceInfo(_sprite.Collider, contactPoint);
                }
            }
        }

        private void RenderFrame()
        {
            if (_bitmap == null) return;

            using (Graphics g = Graphics.FromImage(_bitmap))
            {
                g.Clear(Color.Black);

                // Draw debug visualization
                DrawDebugInfo(g);

                // Draw sprites
                _sprite?.Draw(g, _spriteScaleFactor);
                _anotherSprite?.Draw(g, _anotherSpriteScaleFactor);
            }
        }

        private void DrawDebugInfo(Graphics g)
        {
            // Draw collision shapes
            _sprite?.DebugRender(g, Pens.Red);
            _anotherSprite?.DebugRender(g, Pens.Blue);

            // Draw surface normals and contact points
            if (_sprite?.Collider?.ConnectedObject?.CurrentSurface != null)
            {
                var surface = _sprite.Collider.ConnectedObject.CurrentSurface;
                DrawSurfaceInfo(g, surface, Pens.Yellow);
            }

            if (_anotherSprite?.Collider?.ConnectedObject?.CurrentSurface != null)
            {
                var surface = _anotherSprite.Collider.ConnectedObject.CurrentSurface;
                DrawSurfaceInfo(g, surface, Pens.Green);
            }
        }

        private void DrawSurfaceInfo(Graphics g, SurfacePoint surface, Pen pen)
        {
            // Draw normal vector
            float normalLength = 30f;
            PointF start = new PointF(surface.Position.X, surface.Position.Y);
            PointF end = new PointF(
                surface.Position.X + surface.Normal.X * normalLength,
                surface.Position.Y + surface.Normal.Y * normalLength
            );
            g.DrawLine(pen, start, end);

            // Draw contact point
            float size = 4f;
            g.DrawEllipse(pen, 
                surface.Position.X - size/2, 
                surface.Position.Y - size/2,
                size, size);
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            e.Graphics.DrawImage(_bitmap, 0, 0);
        }

        private float CalculateScaleFactor(int originalWidth, int originalHeight, int targetWidth, int targetHeight)
        {
            float widthScale = (float)targetWidth / originalWidth;
            float heightScale = (float)targetHeight / originalHeight;
            return Math.Min(widthScale, heightScale);
        }
    }

    public class Sprite
    {
        public int Width { get; private set; }
        public int Height { get; private set; }
        private Bitmap _bitmap;
        public PhysicsObject ConnectedObject { get; private set; }
        public Collider Collider { get; private set; }

        public Sprite(int width, int height, Bitmap bitmap)
        {
            Width = width;
            Height = height;
            _bitmap = bitmap;
        }

        /// <summary>
        /// Loads a sprite from an image file.
        /// </summary>
        public static Sprite FromImage(string filePath)
        {
            if (!System.IO.File.Exists(filePath))
                throw new Exception("File not found: " + filePath);

            Bitmap bitmap = new Bitmap(filePath);
            return new Sprite(bitmap.Width, bitmap.Height, bitmap);
        }

        /// <summary>
        /// Attaches a physics object to this sprite for movement and collision handling.
        /// </summary>
        public void AttachPhysics(PhysicsObject physicsObject)
        {
            ConnectedObject = physicsObject;

            // If a collider already exists, update its ConnectedObject reference
            if (Collider != null)
            {
                Collider.ConnectedObject = physicsObject;
            }
        }

        /// <summary>
        /// Attaches a collider to this sprite for collision detection and resolution.
        /// </summary>
        public void AttachCollider(Collider collider)
        {
            Collider = collider;

            // Ensure the collider's ConnectedObject is linked to the sprite's PhysicsObject
            if (ConnectedObject != null)
            {
                Collider.ConnectedObject = ConnectedObject;
            }
        }

        /// <summary>
        /// Draws the sprite onto the graphics context with scaling.
        /// </summary>
        public void Draw(Graphics g, float scaleFactor)
        {
            if (ConnectedObject == null)
                throw new InvalidOperationException("PhysicsObject is not attached to the sprite.");

            g.DrawImage(_bitmap,
                new Rectangle((int)ConnectedObject.X, (int)ConnectedObject.Y,
                    (int)(Width * scaleFactor), (int)(Height * scaleFactor)));
        }

        /// <summary>
        /// Gets the bitmap representation of this sprite.
        /// </summary>
        public Bitmap ToBitmap() => _bitmap;

        /// <summary>
        /// Debug rendering for the sprite's collider.
        /// </summary>
        public void DebugRender(Graphics g, Pen pen)
        {
            if (Collider == null)
                return;

            var vertices = Collider.GetTransformedVertices();
            var points = vertices.Select(v => new PointF(v.X, v.Y)).ToArray();

            g.DrawPolygon(pen, points);
        }
    }
}