using _2DPhysicsEngine;
using ZephyrRenderer.Platform;
using Timer = System.Windows.Forms.Timer;

namespace _2DPhysicsEngineTest
{
    public partial class Form1 : Form
    {
        private Timer _timer; // For the game loop
        private Bitmap _bitmap; // Back buffer for rendering
        private Sprite _sprite;
        private Sprite _anotherSprite;
        private Quadtree _quadTree; // Spatial partitioning for objects

        private float _spriteScaleFactor; // Scale factor for the first sprite
        private float _anotherSpriteScaleFactor; // Scale factor for the second sprite

        public Form1()
        {
            // Set up the form
            this.Text = "2D Physics Engine";
            this.Width = 800;
            this.Height = 600;
            this.DoubleBuffered = true; // Enable double buffering

            // Create the back buffer
            _bitmap = new Bitmap(800, 600);

            // Initialize the quadtree with screen dimensions
            _quadTree = new Quadtree(0, new RECT(0, 0, 800, 600));

            // Load and scale the sprites
            string desktopPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);

            // Initialize _sprite
            string instaFilePath = Path.Combine(desktopPath, "insta.png");
            if (!File.Exists(instaFilePath))
            {
                throw new FileNotFoundException("File not found: " + instaFilePath);
            }
            _sprite = Sprite.FromImage(instaFilePath);
            var spritePhysics = new PhysicsObject(100, 100, 1)
            {
                VX = 2, // Horizontal velocity
                VY = 1  // Vertical velocity
            };
            _sprite.AttachPhysics(spritePhysics);
            _sprite.AttachCollider(new SquareCollider(spritePhysics, 200, new ValueTuple<float, float>(100,100))); // Attach collider with the existing physics object

            // Initialize _anotherSprite
            string diceFilePath = Path.Combine(desktopPath, "dice.png");
            if (!File.Exists(diceFilePath))
            {
                throw new FileNotFoundException("File not found: " + diceFilePath);
            }
            _anotherSprite = Sprite.FromImage(diceFilePath);
            var anotherSpritePhysics = new PhysicsObject(100, 400, 10);
            _anotherSprite.AttachPhysics(anotherSpritePhysics);
            _anotherSprite.AttachCollider(new SquareCollider(anotherSpritePhysics, 150, new ValueTuple<float, float>(70,50))); // Attach collider with the existing physics object

            // Insert colliders into the quadtree
            _quadTree.Insert(_sprite.Collider);
            _quadTree.Insert(_anotherSprite.Collider);

            // Calculate scale factors to fit the sprites into the window
            _spriteScaleFactor = CalculateScaleFactor(_sprite.Width, _sprite.Height, 200, 200); // Target size: 200x200
            _anotherSpriteScaleFactor = CalculateScaleFactor(_anotherSprite.Width, _anotherSprite.Height, 150, 150); // Target size: 150x150

            // Set up the game loop
            _timer = new Timer();
            _timer.Interval = 16; // ~60 FPS
            _timer.Tick += Update;
            _timer.Start();
        }

        private float CalculateScaleFactor(int originalWidth, int originalHeight, int targetWidth, int targetHeight)
        {
            float widthScale = (float)targetWidth / originalWidth;
            float heightScale = (float)targetHeight / originalHeight;
            return Math.Min(widthScale, heightScale); // Keep aspect ratio
        }

        private void Update(object sender, EventArgs e)
        {
            // Clear the quadtree
            _quadTree.Clear();

            // Insert colliders into the quadtree if they are not null
            if (_sprite.Collider != null)
                _quadTree.Insert(_sprite.Collider);

            if (_anotherSprite.Collider != null)
                _quadTree.Insert(_anotherSprite.Collider);
            
            // Apply gravity to the sprite
            _sprite.Collider.ConnectedObject.ApplyForce(0, 9.8f); // Gravity force

            // Update physics
            _sprite.Collider.ConnectedObject.Update(0.016f); // 16 ms per frame (0.016 seconds)

            // Check for collision with window boundaries
            if (_sprite.Collider.ConnectedObject.X < 0 || _sprite.Collider.ConnectedObject.X + _sprite.Width * _spriteScaleFactor > _bitmap.Width)
                _sprite.Collider.ConnectedObject.VX *= -1;
            if (_sprite.Collider.ConnectedObject.Y < 0 || _sprite.Collider.ConnectedObject.Y + _sprite.Height * _spriteScaleFactor > _bitmap.Height)
                _sprite.Collider.ConnectedObject.VY *= -1;

            // Retrieve potential collisions from the quadtree
            var potentialCollisions = _quadTree.Retrieve(_sprite.Collider);

            foreach (var collider in potentialCollisions)
            {
                if (_sprite.Collider.CheckCollision(collider))
                {
                    _sprite.Collider.ResolveCollision(collider);
                }
            }

            // Render the frame
            RenderFrame();
            this.Invalidate(); // Force window repaint
        }
        
        private void RenderFrame()
        {
            // Ensure bitmap is initialized
            if (_bitmap == null)
            {
                Console.WriteLine("Bitmap is null in RenderFrame.");
                return;
            }

            using (Graphics g = Graphics.FromImage(_bitmap))
            {
                // Clear the background
                g.Clear(Color.Black);

                _sprite.DebugRender(g, Pens.Red);
                _anotherSprite.DebugRender(g, Pens.Blue);
                // Draw the first sprite
                if (_sprite != null)
                {
                    _sprite.Draw(g, _spriteScaleFactor);
                }

                // Draw the second sprite
                if (_anotherSprite != null)
                {
                    _anotherSprite.Draw(g, _anotherSpriteScaleFactor);
                }
            }
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            // Draw the back buffer to the screen
            e.Graphics.DrawImage(_bitmap, 0, 0);
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