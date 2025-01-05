namespace _2DPhysicsEngine
{
    using ZephyrRenderer.Platform;
    public class Quadtree
    {
        private readonly int _maxObjects;
        private readonly int _maxLevels;
        private readonly int _level;
        private readonly List<Collider> _colliders; // Store colliders instead of PhysicsObjects
        private readonly RECT _bounds;
        private Quadtree[] _nodes;

        public Quadtree(int level, RECT bounds, int maxObjects = 10, int maxLevels = 5)
        {
            _level = level;
            _bounds = bounds;
            _maxObjects = maxObjects;
            _maxLevels = maxLevels;
            _colliders = new List<Collider>();
            _nodes = null;
        }

        public void Clear()
        {
            _colliders.Clear();

            if (_nodes != null)
            {
                foreach (var node in _nodes)
                {
                    node.Clear();
                }
                _nodes = null;
            }
        }

        public void Insert(Collider collider)
        {
            // If the object fits into a child node, add it there
            if (_nodes != null)
            {
                int index = GetIndex(collider);
                if (index != -1)
                {
                    _nodes[index].Insert(collider);
                    return;
                }
            }

            // Otherwise, add it to this node
            _colliders.Add(collider);

            // If the node exceeds the max objects, subdivide
            if (_colliders.Count > _maxObjects && _level < _maxLevels)
            {
                if (_nodes == null) Subdivide();

                int i = 0;
                while (i < _colliders.Count)
                {
                    int index = GetIndex(_colliders[i]);
                    if (index != -1)
                    {
                        _nodes[index].Insert(_colliders[i]);
                        _colliders.RemoveAt(i);
                    }
                    else
                    {
                        i++;
                    }
                }
            }
        }

        private void Subdivide()
        {
            // Split the current node into four subnodes
            double subWidth = _bounds.Width / 2;
            double subHeight = _bounds.Height / 2;
            double x = _bounds.X;
            double y = _bounds.Y;

            _nodes = new Quadtree[4];
            _nodes[0] = new Quadtree(_level + 1, new RECT(x, y, subWidth, subHeight));
            _nodes[1] = new Quadtree(_level + 1, new RECT(x + subWidth, y, subWidth, subHeight));
            _nodes[2] = new Quadtree(_level + 1, new RECT(x, y + subHeight, subWidth, subHeight));
            _nodes[3] = new Quadtree(_level + 1, new RECT(x + subWidth, y + subHeight, subWidth, subHeight));
        }

        private int GetIndex(Collider collider)
        {
            int index = -1;

            // Compute the bounding box of the collider
            var bounds = collider.GetBoundingBox();

            double verticalMidpoint = _bounds.X + _bounds.Width / 2;
            double horizontalMidpoint = _bounds.Y + _bounds.Height / 2;

            // Check if the object is in the top quadrants
            bool topQuadrant = (bounds.Y < horizontalMidpoint && bounds.Y + bounds.Height < horizontalMidpoint);

            // Check if the object is in the bottom quadrants
            bool bottomQuadrant = (bounds.Y > horizontalMidpoint);

            // Check if the object is in the left quadrants
            bool leftQuadrant = (bounds.X < verticalMidpoint && bounds.X + bounds.Width < verticalMidpoint);

            // Check if the object is in the right quadrants
            bool rightQuadrant = (bounds.X > verticalMidpoint);

            // Determine the index based on the object's position
            if (topQuadrant)
            {
                if (leftQuadrant)
                {
                    index = 0; // Top-left quadrant
                }
                else if (rightQuadrant)
                {
                    index = 1; // Top-right quadrant
                }
            }
            else if (bottomQuadrant)
            {
                if (leftQuadrant)
                {
                    index = 2; // Bottom-left quadrant
                }
                else if (rightQuadrant)
                {
                    index = 3; // Bottom-right quadrant
                }
            }

            return index;
        }

        public List<Collider> Retrieve(Collider collider)
        {
            List<Collider> potentialCollisions = new List<Collider>();

            // Get the index of the object's quadrant
            int index = GetIndex(collider);

            // If the object fits into a quadrant and the node has subdivided, retrieve from that quadrant
            if (index != -1 && _nodes != null)
            {
                potentialCollisions.AddRange(_nodes[index].Retrieve(collider));
            }

            // Add all colliders from this node (colliders that don't fit into a single quadrant)
            potentialCollisions.AddRange(_colliders);

            return potentialCollisions;
        }
    }
}