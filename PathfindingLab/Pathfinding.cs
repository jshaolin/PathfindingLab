using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace PathfindingLab
{
    public class Pathfinding
    {
        private Node[,] node_map;
        private int tile_size;
        private List<Node> neighbors;
        private List<Node> open_list;
        private Dictionary<Node, bool> closed_list;

        private const float SQRT2 = 1.4142f; //Math.Sqrt(2)
        private const float F = 0.4142f; //SQRT2 - 1
        private bool straight_path;


        public Pathfinding(Node[,] node_map, int tile_size)
        {
            this.node_map = node_map;
            this.tile_size = tile_size;
            open_list = new List<Node>();
            closed_list = new Dictionary<Node, bool>();
            neighbors = new List<Node>(8);
        }

        public List<Node> Find(Node start, Node end, ref List<Node> _closed_list, bool dijkstra)
        {
            open_list.Clear();
            closed_list.Clear();

            start.GCost = 0;
            start.FCost = 0;

            open_list.Add(start);

            while (open_list.Count > 0)
            {
                Node node = GetSmallest();
                
                closed_list.Add(node, true);

                if (node.IntCoords == end.IntCoords)
                {
                    //Reached end, reconstruct path
                    _closed_list = closed_list.Keys.ToList();
                    return ReconstructPath(node);
                }

                neighbors = GetNeighborTiles(node.IntCoords);

                for (int i = 0; i < neighbors.Count; i++)
                {
                    Node neighbor = neighbors[i];

                    if (closed_list.ContainsKey(neighbor))
                    {
                        continue;
                    }

                    int x = neighbor.IntCoords.X;
                    int y = neighbor.IntCoords.Y;

                    float ng = node.GCost + ((x - node.IntCoords.X == 0 || y - node.IntCoords.Y == 0) ? 1 : SQRT2);

                    if (!open_list.Contains(neighbor) || ng < neighbor.GCost)
                    {
                        neighbor.GCost = ng;
                        if (dijkstra)
                        {
                            neighbor.HCost = 0; //For Dijkstra
                        }
                        else
                        {
                            neighbor.HCost = Heuristic(Math.Abs(x - end.IntCoords.X), Math.Abs(y - end.IntCoords.Y)); //For A*
                        }
                        
                        float old_f_value = neighbor.FCost;
                        neighbor.FCost = neighbor.GCost + neighbor.HCost;
                        neighbor.Parent = node;

                        if (!open_list.Contains(neighbor))
                        {
                            open_list.Add(neighbor);
                        }
                        else
                        {
                            UpdateOpenList(neighbor, old_f_value);
                        }
                    }
                }
            }

            _closed_list = closed_list.Keys.ToList();
            return null;
        }
        
        private Node GetSmallest()
        {
            Node node = open_list[0];
            for (int i = 0; i < open_list.Count; i++)
            {
                if (open_list[i].FCost <= node.FCost) node = open_list[i];
            }
            open_list.Remove(node);
            return node;
        }

        private void UpdateOpenList(Node neighbor, float old_f_value)
        {
            for (int i = 0; i < open_list.Count; i++)
            {
                if (open_list[i].IntCoords == neighbor.IntCoords && Math.Abs(open_list[i].FCost - old_f_value) <= 0.1f)
                {
                    open_list[i].FCost = neighbor.FCost;
                }
            }
        }

        private float Heuristic(int dx, int dy)
        {
            return (dx < dy) ? F * dx + dy : F * dy + dx;
        }

        private List<Node> ReconstructPath(Node end_node)
        {
            List<Node> path = new List<Node>();
            path.Add(end_node);

            while (end_node.Parent != null)
            {
                end_node = end_node.Parent;
                path.Add(end_node);
            }
            return path; //It contains one more node, the start node.
        }

        private bool IsBusyTile(int x, int y)
        {
            return node_map[x, y].IsObstacle;
        }

        private bool IsBusyTile(Point tile)
        {
            return node_map[tile.X, tile.Y].IsObstacle;
        }

        private bool IsWalkableAt(int x, int y)
        {
            return this.IsInside(x, y) && !node_map[x, y].IsObstacle;
        }

        private bool IsInside(int x, int y)
        {
            return (x >= 0 && x < node_map.GetLength(0)) && (y >= 0 && y < node_map.GetLength(1)); 
        }

        private List<Node> GetNeighborTiles(Point tile)
        {
            int x = tile.X;
            int y = tile.Y;
            bool up_free = false, down_free = false, left_free = false, right_free = false;

            neighbors.Clear();

            if (x - 1 > -1 && y - 1 > -1 && x + 1 < node_map.GetLength(0) && y + 1 < node_map.GetLength(1))
            {
                if (IsWalkableAt(x, y - 1)) //Up
                {
                    up_free = true;
                    neighbors.Add(node_map[x, y - 1]);
                }
                else
                {
                    up_free = false;
                }

                if (IsWalkableAt(x - 1, y)) //Left
                {
                    left_free = true;
                    neighbors.Add(node_map[x - 1, y]);
                }
                else
                {
                    left_free = false;
                }

                if (IsWalkableAt(x, y + 1)) //Down
                {
                    down_free = true;
                    neighbors.Add(node_map[x, y + 1]);
                }
                else
                {
                    down_free = false;
                }

                if (IsWalkableAt(x + 1, y)) //Right
                {
                    right_free = true;
                    neighbors.Add(node_map[x + 1, y]);
                }
                else
                {
                    right_free = false;
                }

                if (IsWalkableAt(x - 1, y - 1) && up_free && left_free) //Up Left
                {
                    neighbors.Add(node_map[x - 1, y - 1]);
                }

                if (IsWalkableAt(x - 1, y + 1) && left_free && down_free) //Left Down
                {
                    neighbors.Add(node_map[x - 1, y + 1]);
                }

                if (IsWalkableAt(x + 1, y + 1) && down_free && right_free) //Down Right
                {
                    neighbors.Add(node_map[x + 1, y + 1]);
                }

                if (IsWalkableAt(x + 1, y - 1) && up_free && right_free) //Up Right
                {
                    neighbors.Add(node_map[x + 1, y - 1]);
                }
            }

            neighbors.TrimExcess();
            return neighbors;
        }

        /*public void BuildPathToPlayerFrom(Point start)
        {
            straight_path = BuildPath(start, Character.PlayerActor.PositionInTile, out path);

            if (straight_path == false)
            {
                path = PrunePath(path);
                for (int i = 0; i < path.Count; i++)
                {
                    path[i] = path[i] * new Point(tile_size);
                }
            }
        }*/

        public bool BuildPath(Point start, Point target, out List<Point> path)
        {
            if (GeneralRayTest(start, target) == true) //Check to see if the straight path has any obstacle
            {
                // There is at least an obstacle, so it is not a straight path and we must execute A* pathfind.
                //path = Find(start, target);
                path = null;
                return false;
            }
            else
            {
                //It is a tiled straight path since no obstacles where found in it
                path = null;
                return true;
            }
        }

        private List<Point> PrunePath(List<Point> path)
        {
            List<Point> pruned_path = new List<Point>();
            Point current = path[0];

            for (int i = 1; i < path.Count; i++)
            {
                if (RayTest(current, path[i]) == true) //If an obstacle exist in the vector path
                {
                    //Vector path to target path[i] NOT free from tile obstacles
                    current = path[i - 1];
                    pruned_path.Add(current);
                }
            }
            pruned_path.Add(path.Last());

            return pruned_path;
        }

        private bool RayTest(Point start, Point target) //Javier's line algorithm THE REAL STUFF
        {
            if (target.X == start.X || target.Y == start.Y) // I moved in a cardinal direction
            {
                return node_map[target.X, target.Y].IsObstacle; //Returns true if an obstacle exists, false otherwise.
            }
            else //We moved in a diagonal direction. Need to check a ray from every corner of the collision rectangle towards the desired tile origin. A path is clear when every ray is clear.
            {
                Vector2 ftargetTopLeft = (target * new Point(tile_size)).ToVector2();
                Vector2 fstartTopLeft = (start * new Point(tile_size)).ToVector2();
                Vector2 dirTopLeft = ftargetTopLeft - fstartTopLeft;
                dirTopLeft.Normalize();

                int tile_x;
                int tile_y;

                if (dirTopLeft.X < 0 && dirTopLeft.Y > 0) // Moving in Bottom left diagonal, check Top Left ray, Bottom left, and Bottom right rays.
                {
                    #region Bottom left diagonal ray checks
                    Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                    Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();

                    Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                    Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();

                    Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;
                    Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;

                    dirBottomLeft.Normalize();
                    dirBottomRight.Normalize();

                    while (fstartTopLeft.X > ftargetTopLeft.X) //Top Left ray
                    {
                        tile_x = (int)(fstartTopLeft.X / tile_size);
                        tile_y = (int)(fstartTopLeft.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the bottom left diagonal vector path
                        }

                        fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                    }

                    while (fstartBottomLeft.X > ftargetBottomLeft.X) //Bottom left ray
                    {
                        tile_x = (int)(fstartBottomLeft.X / tile_size);
                        tile_y = (int)(fstartBottomLeft.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the bottom left diagonal vector path
                        }

                        fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                    }

                    while (fstartBottomRight.X > ftargetBottomRight.X) //Bottom right ray
                    {
                        tile_x = (int)(fstartBottomRight.X / tile_size);
                        tile_y = (int)(fstartBottomRight.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the bottom left diagonal vector path
                        }

                        fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                    }

                    return false;
                    #endregion
                }
                else if (dirTopLeft.X < 0 && dirTopLeft.Y < 0) // Moving in Top left diagonal, check Top Left ray, Bottom left, and Top right rays.
                {
                    #region Top left diagonal ray checks
                    Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                    Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                    Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                    Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                    Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;
                    Vector2 dirTopRight = ftargetTopRight - fstartTopRight;

                    dirBottomLeft.Normalize();
                    dirTopRight.Normalize();

                    while (fstartTopLeft.X > ftargetTopLeft.X) //Top left ray check
                    {
                        tile_x = (int)(fstartTopLeft.X / tile_size);
                        tile_y = (int)(fstartTopLeft.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the top left diagonal vector path
                        }

                        fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                    }

                    while (fstartBottomLeft.X > ftargetBottomLeft.X) //Bottom left ray check
                    {
                        tile_x = (int)(fstartBottomLeft.X / tile_size);
                        tile_y = (int)(fstartBottomLeft.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the top left diagonal vector path
                        }

                        fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                    }

                    while (fstartTopRight.X > ftargetTopRight.X) //Top right ray check
                    {
                        tile_x = (int)(fstartTopRight.X / tile_size);
                        tile_y = (int)(fstartTopRight.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the top left diagonal vector path
                        }

                        fstartTopRight = fstartTopRight + 60f * dirTopRight;
                    }
                    return false;
                    #endregion
                }
                else if (dirTopLeft.X > 0 && dirTopLeft.Y > 0) //Moving in Bottom right diagonal, check Bottom left, Bottom right, and Top right rays.
                {
                    #region Bottom right diagonal ray checks
                    Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                    Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();
                    Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                    Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                    Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();
                    Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                    Vector2 dirTopRight = ftargetTopRight - fstartTopRight;
                    Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;
                    Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;

                    dirBottomLeft.Normalize();
                    dirBottomRight.Normalize();
                    dirTopRight.Normalize();

                    while (fstartBottomLeft.X < ftargetBottomLeft.X) //Bottom left ray check
                    {
                        tile_x = (int)(fstartBottomLeft.X / tile_size);
                        tile_y = (int)(fstartBottomLeft.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the bottom right diagonal vector path
                        }

                        fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                    }

                    while (fstartBottomRight.X < ftargetBottomRight.X) //Bottom right ray check
                    {
                        tile_x = (int)(fstartBottomRight.X / tile_size);
                        tile_y = (int)(fstartBottomRight.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the bottom right diagonal vector path
                        }

                        fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                    }

                    while (fstartTopRight.X < ftargetTopRight.X) //Top right ray check
                    {
                        tile_x = (int)(fstartTopRight.X / tile_size);
                        tile_y = (int)(fstartTopRight.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the bottom right diagonal vector path
                        }

                        fstartTopRight = fstartTopRight + 60f * dirTopRight;
                    }
                    return false;
                    #endregion
                }
                else if (dirTopLeft.X > 0 && dirTopLeft.Y < 0) //Moving in Top right diagonal, check Top left, Top right, Bottom right
                {
                    #region Top right diagonal ray checks
                    Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                    Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();

                    Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                    Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();

                    Vector2 dirTopRight = ftargetTopRight - fstartTopRight;
                    Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;

                    dirTopRight.Normalize();
                    dirBottomRight.Normalize();

                    while (fstartTopLeft.X < ftargetTopLeft.X) //Top left ray check
                    {
                        tile_x = (int)(fstartTopLeft.X / tile_size);
                        tile_y = (int)(fstartTopLeft.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the top right diagonal vector path
                        }

                        fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                    }

                    while (fstartTopRight.X < ftargetTopRight.X) //Top right ray check
                    {
                        tile_x = (int)(fstartTopRight.X / tile_size);
                        tile_y = (int)(fstartTopRight.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the top right diagonal vector path
                        }

                        fstartTopRight = fstartTopRight + 60f * dirTopRight;
                    }

                    while (fstartBottomRight.X < ftargetBottomRight.X) //Bottom right ray check
                    {
                        tile_x = (int)(fstartBottomRight.X / tile_size);
                        tile_y = (int)(fstartBottomRight.Y / tile_size);

                        if (IsBusyTile(tile_x, tile_y) == true)
                        {
                            return true; //There was an obstacle in the top right diagonal vector path
                        }

                        fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                    }
                    return false;
                    #endregion
                }
            }
            return false;
        }

        private bool GeneralRayTest(Point start, Point target) //Javier's line algorithm THE REAL STUFF
        {
            Vector2 ftargetTopLeft = (target * new Point(tile_size)).ToVector2();
            Vector2 fstartTopLeft = (start * new Point(tile_size)).ToVector2();
            Vector2 dirTopLeft = ftargetTopLeft - fstartTopLeft;
            dirTopLeft.Normalize();

            int tile_x;
            int tile_y;

            if (dirTopLeft.X > 0 && dirTopLeft.Y == 0f) //Moving in Right direction, check Top right, and Bottom right ray.
            {
                #region Right ray checks
                Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 dirTopRight = ftargetTopRight - fstartTopRight;
                Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;

                dirTopRight.Normalize();
                dirBottomRight.Normalize();

                while (fstartTopRight.X < ftargetTopLeft.X) //Top right ray
                {
                    tile_x = (int)(fstartTopRight.X / tile_size);
                    tile_y = (int)(fstartTopRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartTopRight = fstartTopRight + 60f * dirTopRight;
                }

                while (fstartBottomRight.X < ftargetTopLeft.X) //Bottom right ray
                {
                    tile_x = (int)(fstartBottomRight.X / tile_size);
                    tile_y = (int)(fstartBottomRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X == 0f && dirTopLeft.Y < 0) //Moving in Up direction, check Top left, and Top right ray.
            {
                #region Up ray checks
                Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 dirTopRight = ftargetTopRight - fstartTopRight;
                dirTopRight.Normalize();

                while (fstartTopLeft.Y > ftargetTopLeft.Y) //Top left ray
                {
                    tile_x = (int)(fstartTopLeft.X / tile_size);
                    tile_y = (int)(fstartTopLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                }

                while (fstartTopRight.Y > ftargetTopLeft.Y) //Top right ray
                {
                    tile_x = (int)(fstartTopRight.X / tile_size);
                    tile_y = (int)(fstartTopRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartTopRight = fstartTopRight + 60f * dirTopRight;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X < 0 && dirTopLeft.Y == 0f) //Moving in Left direction, check Top left, and Bottom left ray.
            {
                #region Left ray checks
                Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;
                dirBottomLeft.Normalize();

                while (fstartTopLeft.X > ftargetTopLeft.X) //Top left ray
                {
                    tile_x = (int)(fstartTopLeft.X / tile_size);
                    tile_y = (int)(fstartTopLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                }

                while (fstartBottomLeft.X > ftargetTopLeft.X) //Bottom left ray
                {
                    tile_x = (int)(fstartBottomLeft.X / tile_size);
                    tile_y = (int)(fstartBottomLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X == 0f && dirTopLeft.Y > 0) //Moving in Down direction, check Bottom left, and Bottom right ray.
            {
                #region Down ray checks
                Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;
                Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;

                dirBottomLeft.Normalize();
                dirBottomRight.Normalize();

                while (fstartBottomLeft.Y < ftargetTopLeft.Y) //Bottom left ray
                {
                    tile_x = (int)(fstartBottomLeft.X / tile_size);
                    tile_y = (int)(fstartBottomLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                }

                while (fstartBottomRight.Y < ftargetTopLeft.Y) //Bottom right ray
                {
                    tile_x = (int)(fstartBottomRight.X / tile_size);
                    tile_y = (int)(fstartBottomRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X < 0 && dirTopLeft.Y > 0) // Moving in Bottom left diagonal, check Top Left ray, Bottom left, and Bottom right rays.
            {
                #region Bottom left diagonal ray checks
                Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;
                Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;

                dirBottomLeft.Normalize();
                dirBottomRight.Normalize();

                while (fstartTopLeft.X > ftargetTopLeft.X) //Top Left ray
                {
                    tile_x = (int)(fstartTopLeft.X / tile_size);
                    tile_y = (int)(fstartTopLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                }

                while (fstartBottomLeft.X > ftargetBottomLeft.X) //Bottom left ray
                {
                    tile_x = (int)(fstartBottomLeft.X / tile_size);
                    tile_y = (int)(fstartBottomLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                }

                while (fstartBottomRight.X > ftargetBottomRight.X) //Bottom right ray
                {
                    tile_x = (int)(fstartBottomRight.X / tile_size);
                    tile_y = (int)(fstartBottomRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                }

                return false;
                #endregion
            }
            else if (dirTopLeft.X < 0 && dirTopLeft.Y < 0) // Moving in Top left diagonal, check Top Left ray, Bottom left, and Top right rays.
            {
                #region Top left diagonal ray checks
                Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;
                Vector2 dirTopRight = ftargetTopRight - fstartTopRight;

                dirBottomLeft.Normalize();
                dirTopRight.Normalize();

                while (fstartTopLeft.X > ftargetTopLeft.X) //Top left ray check
                {
                    tile_x = (int)(fstartTopLeft.X / tile_size);
                    tile_y = (int)(fstartTopLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top left diagonal vector path
                    }

                    fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                }

                while (fstartBottomLeft.X > ftargetBottomLeft.X) //Bottom left ray check
                {
                    tile_x = (int)(fstartBottomLeft.X / tile_size);
                    tile_y = (int)(fstartBottomLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top left diagonal vector path
                    }

                    fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                }

                while (fstartTopRight.X > ftargetTopRight.X) //Top right ray check
                {
                    tile_x = (int)(fstartTopRight.X / tile_size);
                    tile_y = (int)(fstartTopRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top left diagonal vector path
                    }

                    fstartTopRight = fstartTopRight + 60f * dirTopRight;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X > 0 && dirTopLeft.Y > 0) //Moving in Bottom right diagonal, check Bottom left, Bottom right, and Top right rays.
            {
                #region Bottom right diagonal ray checks
                Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                Vector2 dirTopRight = ftargetTopRight - fstartTopRight;
                Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;
                Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;

                dirBottomLeft.Normalize();
                dirBottomRight.Normalize();
                dirTopRight.Normalize();

                while (fstartBottomLeft.X < ftargetBottomLeft.X) //Bottom left ray check
                {
                    tile_x = (int)(fstartBottomLeft.X / tile_size);
                    tile_y = (int)(fstartBottomLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom right diagonal vector path
                    }

                    fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                }

                while (fstartBottomRight.X < ftargetBottomRight.X) //Bottom right ray check
                {
                    tile_x = (int)(fstartBottomRight.X / tile_size);
                    tile_y = (int)(fstartBottomRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom right diagonal vector path
                    }

                    fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                }

                while (fstartTopRight.X < ftargetTopRight.X) //Top right ray check
                {
                    tile_x = (int)(fstartTopRight.X / tile_size);
                    tile_y = (int)(fstartTopRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom right diagonal vector path
                    }

                    fstartTopRight = fstartTopRight + 60f * dirTopRight;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X > 0 && dirTopLeft.Y < 0) //Moving in Top right diagonal, check Top left, Top right, Bottom right
            {
                #region Top right diagonal ray checks
                Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 dirTopRight = ftargetTopRight - fstartTopRight;
                Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;

                dirTopRight.Normalize();
                dirBottomRight.Normalize();

                while (fstartTopLeft.X < ftargetTopLeft.X) //Top left ray check
                {
                    tile_x = (int)(fstartTopLeft.X / tile_size);
                    tile_y = (int)(fstartTopLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top right diagonal vector path
                    }

                    fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                }

                while (fstartTopRight.X < ftargetTopRight.X) //Top right ray check
                {
                    tile_x = (int)(fstartTopRight.X / tile_size);
                    tile_y = (int)(fstartTopRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top right diagonal vector path
                    }

                    fstartTopRight = fstartTopRight + 60f * dirTopRight;
                }

                while (fstartBottomRight.X < ftargetBottomRight.X) //Bottom right ray check
                {
                    tile_x = (int)(fstartBottomRight.X / tile_size);
                    tile_y = (int)(fstartBottomRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top right diagonal vector path
                    }

                    fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                }
                return false;
                #endregion
            }
            return true;
        }

        private bool CollisionRayTest(Point start, Point target)
        {
            Vector2 ftargetTopLeft = (target * new Point(tile_size)).ToVector2();
            Vector2 fstartTopLeft = (start * new Point(tile_size)).ToVector2();
            Vector2 dirTopLeft = ftargetTopLeft - fstartTopLeft;
            dirTopLeft.Normalize();

            int tile_x;
            int tile_y;

            if (dirTopLeft.X > 0 && dirTopLeft.Y == 0f) //Moving in Right direction, check Top right, and Bottom right ray.
            {
                #region Right ray checks
                Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 dirTopRight = ftargetTopRight - fstartTopRight;
                Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;

                dirTopRight.Normalize();
                dirBottomRight.Normalize();

                while (fstartTopRight.X < ftargetTopLeft.X) //Top right ray
                {
                    tile_x = (int)(fstartTopRight.X / tile_size);
                    tile_y = (int)(fstartTopRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartTopRight = fstartTopRight + 60f * dirTopRight;
                }

                while (fstartBottomRight.X < ftargetTopLeft.X) //Bottom right ray
                {
                    tile_x = (int)(fstartBottomRight.X / tile_size);
                    tile_y = (int)(fstartBottomRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X == 0f && dirTopLeft.Y < 0) //Moving in Up direction, check Top left, and Top right ray.
            {
                #region Up ray checks
                Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 dirTopRight = ftargetTopRight - fstartTopRight;
                dirTopRight.Normalize();

                while (fstartTopLeft.Y > ftargetTopLeft.Y) //Top left ray
                {
                    tile_x = (int)(fstartTopLeft.X / tile_size);
                    tile_y = (int)(fstartTopLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                }

                while (fstartTopRight.Y > ftargetTopLeft.Y) //Top right ray
                {
                    tile_x = (int)(fstartTopRight.X / tile_size);
                    tile_y = (int)(fstartTopRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartTopRight = fstartTopRight + 60f * dirTopRight;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X < 0 && dirTopLeft.Y == 0f) //Moving in Left direction, check Top left, and Bottom left ray.
            {
                #region Left ray checks
                Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;
                dirBottomLeft.Normalize();

                while (fstartTopLeft.X > ftargetTopLeft.X) //Top left ray
                {
                    tile_x = (int)(fstartTopLeft.X / tile_size);
                    tile_y = (int)(fstartTopLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                }

                while (fstartBottomLeft.X > ftargetTopLeft.X) //Bottom left ray
                {
                    tile_x = (int)(fstartBottomLeft.X / tile_size);
                    tile_y = (int)(fstartBottomLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X == 0f && dirTopLeft.Y > 0) //Moving in Down direction, check Bottom left, and Bottom right ray.
            {
                #region Down ray checks
                Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;
                Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;

                dirBottomLeft.Normalize();
                dirBottomRight.Normalize();

                while (fstartBottomLeft.Y < ftargetTopLeft.Y) //Bottom left ray
                {
                    tile_x = (int)(fstartBottomLeft.X / tile_size);
                    tile_y = (int)(fstartBottomLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                }

                while (fstartBottomRight.Y < ftargetTopLeft.Y) //Bottom right ray
                {
                    tile_x = (int)(fstartBottomRight.X / tile_size);
                    tile_y = (int)(fstartBottomRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X < 0 && dirTopLeft.Y > 0) // Moving in Bottom left diagonal, check Top Left ray, Bottom left, and Bottom right rays.
            {
                #region Bottom left diagonal ray checks
                Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;
                Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;

                dirBottomLeft.Normalize();
                dirBottomRight.Normalize();

                while (fstartTopLeft.X > ftargetTopLeft.X) //Top Left ray
                {
                    tile_x = (int)(fstartTopLeft.X / tile_size);
                    tile_y = (int)(fstartTopLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                }

                while (fstartBottomLeft.X > ftargetBottomLeft.X) //Bottom left ray
                {
                    tile_x = (int)(fstartBottomLeft.X / tile_size);
                    tile_y = (int)(fstartBottomLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                }

                while (fstartBottomRight.X > ftargetBottomRight.X) //Bottom right ray
                {
                    tile_x = (int)(fstartBottomRight.X / tile_size);
                    tile_y = (int)(fstartBottomRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom left diagonal vector path
                    }

                    fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                }

                return false;
                #endregion
            }
            else if (dirTopLeft.X < 0 && dirTopLeft.Y < 0) // Moving in Top left diagonal, check Top Left ray, Bottom left, and Top right rays.
            {
                #region Top left diagonal ray checks
                Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;
                Vector2 dirTopRight = ftargetTopRight - fstartTopRight;

                dirBottomLeft.Normalize();
                dirTopRight.Normalize();

                while (fstartTopLeft.X > ftargetTopLeft.X) //Top left ray check
                {
                    tile_x = (int)(fstartTopLeft.X / tile_size);
                    tile_y = (int)(fstartTopLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top left diagonal vector path
                    }

                    fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                }

                while (fstartBottomLeft.X > ftargetBottomLeft.X) //Bottom left ray check
                {
                    tile_x = (int)(fstartBottomLeft.X / tile_size);
                    tile_y = (int)(fstartBottomLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top left diagonal vector path
                    }

                    fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                }

                while (fstartTopRight.X > ftargetTopRight.X) //Top right ray check
                {
                    tile_x = (int)(fstartTopRight.X / tile_size);
                    tile_y = (int)(fstartTopRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top left diagonal vector path
                    }

                    fstartTopRight = fstartTopRight + 60f * dirTopRight;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X > 0 && dirTopLeft.Y > 0) //Moving in Bottom right diagonal, check Bottom left, Bottom right, and Top right rays.
            {
                #region Bottom right diagonal ray checks
                Vector2 ftargetBottomLeft = ((target + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                Vector2 fstartBottomLeft = ((start + new Point(0, 1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();
                Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();

                Vector2 dirTopRight = ftargetTopRight - fstartTopRight;
                Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;
                Vector2 dirBottomLeft = ftargetBottomLeft - fstartBottomLeft;

                dirBottomLeft.Normalize();
                dirBottomRight.Normalize();
                dirTopRight.Normalize();

                while (fstartBottomLeft.X < ftargetBottomLeft.X) //Bottom left ray check
                {
                    tile_x = (int)(fstartBottomLeft.X / tile_size);
                    tile_y = (int)(fstartBottomLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom right diagonal vector path
                    }

                    fstartBottomLeft = fstartBottomLeft + 60f * dirBottomLeft;
                }

                while (fstartBottomRight.X < ftargetBottomRight.X) //Bottom right ray check
                {
                    tile_x = (int)(fstartBottomRight.X / tile_size);
                    tile_y = (int)(fstartBottomRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom right diagonal vector path
                    }

                    fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                }

                while (fstartTopRight.X < ftargetTopRight.X) //Top right ray check
                {
                    tile_x = (int)(fstartTopRight.X / tile_size);
                    tile_y = (int)(fstartTopRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the bottom right diagonal vector path
                    }

                    fstartTopRight = fstartTopRight + 60f * dirTopRight;
                }
                return false;
                #endregion
            }
            else if (dirTopLeft.X > 0 && dirTopLeft.Y < 0) //Moving in Top right diagonal, check Top left, Top right, Bottom right
            {
                #region Top right diagonal ray checks
                Vector2 ftargetTopRight = ((target + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 ftargetBottomRight = ((target + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 fstartTopRight = ((start + new Point(1, 0)) * new Point(tile_size)).ToVector2();
                Vector2 fstartBottomRight = ((start + new Point(1)) * new Point(tile_size)).ToVector2();

                Vector2 dirTopRight = ftargetTopRight - fstartTopRight;
                Vector2 dirBottomRight = ftargetBottomRight - fstartBottomRight;

                dirTopRight.Normalize();
                dirBottomRight.Normalize();

                while (fstartTopLeft.X < ftargetTopLeft.X) //Top left ray check
                {
                    tile_x = (int)(fstartTopLeft.X / tile_size);
                    tile_y = (int)(fstartTopLeft.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top right diagonal vector path
                    }

                    fstartTopLeft = fstartTopLeft + 60f * dirTopLeft;
                }

                while (fstartTopRight.X < ftargetTopRight.X) //Top right ray check
                {
                    tile_x = (int)(fstartTopRight.X / tile_size);
                    tile_y = (int)(fstartTopRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top right diagonal vector path
                    }

                    fstartTopRight = fstartTopRight + 60f * dirTopRight;
                }

                while (fstartBottomRight.X < ftargetBottomRight.X) //Bottom right ray check
                {
                    tile_x = (int)(fstartBottomRight.X / tile_size);
                    tile_y = (int)(fstartBottomRight.Y / tile_size);

                    if (IsBusyTile(tile_x, tile_y) == true)
                    {
                        return true; //There was an obstacle in the top right diagonal vector path
                    }

                    fstartBottomRight = fstartBottomRight + 60f * dirBottomRight;
                }
                return false;
                #endregion
            }
            return true;
        }

        /*public List<Point> Path
        {
            get
            {
                return path;
            }
        }*/

        public bool IsStraightPath
        {
            get
            {
                return straight_path;
            }
        }
    }
}
