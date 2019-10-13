using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;


namespace PathfindingLab
{
    public class Node
    {
        private float f, g, h;
        private Node parent;
        private bool isObstacle;
        private Point int_coords;

        public Node(bool isObstacle, Point int_coords)
        {
            this.int_coords = int_coords;
            this.isObstacle = isObstacle;
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
    }
}
