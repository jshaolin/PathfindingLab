using System;
using System.Runtime.Serialization;

using Microsoft.Xna.Framework;



namespace PathfindingLab
{
    [Serializable]
    public class Node : IComparable<Node>, ISerializable
    {
        private float f, g, h;
        private Node parent;
        private bool isObstacle;
        private Point int_coords;

        public void GetObjectData(SerializationInfo info, StreamingContext context)
        {
            info.AddValue("f", f);
            info.AddValue("g", g);
            info.AddValue("h", h);
            info.AddValue("parent", parent);
            info.AddValue("isObstacle", isObstacle);
            info.AddValue("int_coords.X", int_coords.X);
            info.AddValue("int_coords.Y", int_coords.Y);
        }

        private Node(SerializationInfo info, StreamingContext context)
        {
            f = info.GetSingle("f");
            g = info.GetSingle("g");
            h = info.GetSingle("h");
            parent = (Node)info.GetValue("parent", typeof(Node));
            isObstacle = info.GetBoolean("isObstacle");
            int_coords.X = info.GetInt32("int_coords.X");
            int_coords.Y = info.GetInt32("int_coords.Y");
        }

        public Node(bool isObstacle, Point int_coords)
        {
            this.int_coords = int_coords;
            this.isObstacle = isObstacle;
        }

        public Node()
        {
            int_coords = Point.Zero;
            isObstacle = false;
        }

        public bool IsObstacle
        {
            get { return isObstacle; }
            set { isObstacle = value; }
        }

        public float FCost
        {
            get { return f; }
            set { f = value; }
        }

        public float GCost
        {
            get { return g; }
            set { g = value; }
        }

        public float HCost
        {
            get { return h; }
            set { h = value; }
        }

        public Node Parent
        {
            get { return parent; }
            set { parent = value; }
        }

        public Point IntCoords
        {
            get { return int_coords; }
            set { int_coords = value; }
        }

        public int X
        {
            get { return int_coords.X; }
        }

        public int Y
        {
            get { return int_coords.Y; }
        }

        public Node Clone()
        {
            Node clone = new Node();
            clone.GCost = GCost;
            clone.IntCoords = IntCoords;
            clone.IsObstacle = IsObstacle;

            return clone;
        }

        public int CompareTo(Node other)
        {
            if (other != null)
            {
                if (FCost < other.FCost)
                {
                    return -1;
                }
                else if (FCost > other.FCost)
                {
                    return 1;
                }
            }
            return 0;
        }
    }
}
