using System;
using System.Collections.Generic;

using Microsoft.Xna.Framework;

namespace PathfindingLab
{
    public class Pathfinding
    {
        private Node[,] node_map;
        private int tile_size;
        private List<Node> neighbors;
        private List<Node> open_list;
        private HashSet<Point> closed_hash_set, open_hash_set;

        public const int NODE_SIZE = 32;
        public const int NODE_SUBFACTOR = 64 / NODE_SIZE; //Hacer el 64 una constante
        private const float SQRT2 = 1.4142f; //Math.Sqrt(2)
        private const float F = 0.4142f; //SQRT2 - 1
        private const float DIAGONAL_DISTANCE = SQRT2 * NODE_SIZE;
        private const float CARDINAL_DISTANCE = NODE_SIZE;
        private int max_tiles_to_search;

        private int horizontal_node_extent;
        private int vertical_node_extent;

        private Node start;
        private Node end;

        private Point p;

        public Pathfinding(int tile_size, int search_limit, int horizontal_node_extent, int vertical_node_extent)
        {
            this.tile_size = tile_size;
            max_tiles_to_search = search_limit;
            open_list = new List<Node>();
            closed_hash_set = new HashSet<Point>();
            open_hash_set = new HashSet<Point>();
            neighbors = new List<Node>(8);
            this.horizontal_node_extent = horizontal_node_extent;
            this.vertical_node_extent = vertical_node_extent;
        }

        public bool FindImplicitPath(Point _start, Point _end, ref Node endNode, bool dijkstra)
        {
            start = node_map[_start.X, _start.Y].Clone();
            end = node_map[_end.X, _end.Y].Clone();

            open_list.Clear();
            open_hash_set.Clear();
            closed_hash_set.Clear();

            start.GCost = 0;
            start.FCost = 0;

            open_list.Add(start);
            open_hash_set.Add(start.IntCoords);

            while (open_list.Count > 0 && open_list.Count < max_tiles_to_search)
            {
                Node node = GetSmallest();
                open_hash_set.Remove(node.IntCoords);

                if (closed_hash_set.Contains(node.IntCoords) == false)
                {
                    closed_hash_set.Add(node.IntCoords);
                }

                if (node.IntCoords == end.IntCoords)
                {
                    //Reached end
                    endNode = node.Parent; //We don't want the start node, we want the path to start with the node after the start node.
                    return true;
                }

                neighbors = GetNeighborTiles(node.IntCoords);

                for (int i = 0; i < neighbors.Count; i++)
                {
                    float ng = node.GCost + ((neighbors[i].X - node.X == 0 || neighbors[i].Y - node.Y == 0) ? 1 : SQRT2);

                    if (!open_hash_set.Contains(neighbors[i].IntCoords) || ng < neighbors[i].GCost)
                    {
                        neighbors[i].GCost = ng;
                        if (dijkstra)
                        {
                            neighbors[i].HCost = 0; //For Dijkstra
                        }
                        else
                        {
                            neighbors[i].HCost = Heuristic(Math.Abs(neighbors[i].X - end.X), Math.Abs(neighbors[i].Y - end.Y)); //For A*
                        }
                        
                        neighbors[i].FCost = neighbors[i].GCost + neighbors[i].HCost;
                        neighbors[i].Parent = node;

                        if (!open_hash_set.Contains(neighbors[i].IntCoords))
                        {
                            open_list.Add(neighbors[i]);
                            open_hash_set.Add(neighbors[i].IntCoords);
                        }
                    }
                }
            }
            return false;
        }

        public bool FindImplicitPath(Point _start, Point _end, ref Node endNode, bool dijkstra, ref List<Point> closed_list, ref List<Vector2> position_of_path_tiles)
        {
            start = node_map[_start.X, _start.Y].Clone();
            end = node_map[_end.X, _end.Y].Clone();

            open_list.Clear();
            open_hash_set.Clear();
            closed_hash_set.Clear();

            start.GCost = 0;
            start.FCost = 0;

            open_list.Add(start);
            open_hash_set.Add(start.IntCoords);

            while (open_list.Count > 0 && open_list.Count < max_tiles_to_search)
            {
                Node node = GetSmallest();
                open_hash_set.Remove(node.IntCoords);

                if (closed_hash_set.Contains(node.IntCoords) == false)
                {
                    closed_list.Add(node.IntCoords);
                    closed_hash_set.Add(node.IntCoords);
                }

                if (node.IntCoords == end.IntCoords)
                {
                    //Reached end
                    endNode = node.Parent; //We don't want the start node, we want the path to start with the node after the start node.
                    Node dummy = endNode.Clone();
                    dummy.Parent = endNode.Parent;

                    while (dummy != null)
                    {
                        position_of_path_tiles.Add(new Vector2(tile_size * dummy.IntCoords.X, tile_size * dummy.IntCoords.Y));
                        dummy = dummy.Parent;
                    }
                    
                    return true;
                }

                neighbors = GetNeighborTiles(node.IntCoords);

                for (int i = 0; i < neighbors.Count; i++)
                {
                    float ng = node.GCost + ((neighbors[i].X - node.X == 0 || neighbors[i].Y - node.Y == 0) ? 1 : SQRT2);

                    if (!open_hash_set.Contains(neighbors[i].IntCoords) || ng < neighbors[i].GCost)
                    {
                        neighbors[i].GCost = ng;
                        if (dijkstra)
                        {
                            neighbors[i].HCost = 0; //For Dijkstra
                        }
                        else
                        {
                            neighbors[i].HCost = Heuristic(Math.Abs(neighbors[i].X - end.X), Math.Abs(neighbors[i].Y - end.Y)); //For A*
                        }

                        neighbors[i].FCost = neighbors[i].GCost + neighbors[i].HCost;
                        neighbors[i].Parent = node;

                        if (!open_hash_set.Contains(neighbors[i].IntCoords))
                        {
                            open_list.Add(neighbors[i]);
                            open_hash_set.Add(neighbors[i].IntCoords);
                        }
                    }
                }
            }
            return false;
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

        private float Heuristic(int dx, int dy)
        {
            return (dx < dy) ? F * dx + dy : F * dy + dx;
        }

        private bool IsWalkableAt(int neighbor_x, int neighbor_y, Point tile)
        {
            return !node_map[neighbor_x, neighbor_y].IsObstacle;
        }

        private List<Node> GetNeighborTiles(Point tile)
        {
            int x = tile.X;
            int y = tile.Y;
            bool up_free = false, down_free = false, left_free = false, right_free = false;
            neighbors.Clear();

            if (x - 1 > -1 && y - 1 > -1 && x + 1 < node_map.GetLength(0) && y + 1 < node_map.GetLength(1)) //Is inside the node map boundaries
            {
                #region Up
                p.X = x;
                p.Y = y - 1;
                if (closed_hash_set.Contains(p))
                {
                    goto _left;
                }
                for (int i = 0; i < horizontal_node_extent; i++)
                {
                    for (int j = 1; j <= vertical_node_extent; j++)
                    {
                        if (IsWalkableAt(x - i, y - j, tile) == false || IsWalkableAt(x + i, y - j, tile) == false)
                        {
                            up_free = false;
                            goto _left;
                        }
                    }
                }
                up_free = true;
                neighbors.Add(node_map[x, y - 1].Clone());
                #endregion

                #region Left
                _left:
                p.X = x - 1;
                p.Y = y;
                if (closed_hash_set.Contains(p))
                {
                    goto _down;
                }
                for (int i = 0; i < vertical_node_extent; i++)
                {
                    for (int j = 1; j <= horizontal_node_extent; j++)
                    {
                        if (IsWalkableAt(x - j, y + i, tile) == false || IsWalkableAt(x - j, y - i, tile) == false)
                        {
                            left_free = false;
                            goto _down;
                        }
                    }
                }
                left_free = true;
                neighbors.Add(node_map[x - 1, y].Clone());
                #endregion

                #region Down
                _down:
                p.X = x;
                p.Y = y + 1;
                if (closed_hash_set.Contains(p))
                {
                    goto _right;
                }
                for (int i = 0; i < horizontal_node_extent; i++)
                {
                    for (int j = 1; j <= vertical_node_extent; j++)
                    {
                        if (IsWalkableAt(x - i, y + j, tile) == false || IsWalkableAt(x + i, y + j, tile) == false)
                        {
                            down_free = false;
                            goto _right;
                        }
                    }
                }
                down_free = true;
                neighbors.Add(node_map[x, y + 1].Clone());
                #endregion

                #region Right
                _right:
                p.X = x + 1;
                p.Y = y;
                if (closed_hash_set.Contains(p))
                {
                    goto _diagonals;
                }
                for (int i = 0; i < vertical_node_extent; i++)
                {
                    for (int j = 1; j <= horizontal_node_extent; j++)
                    {
                        if (IsWalkableAt(x + j, y + i, tile) == false || IsWalkableAt(x + j, y - i, tile) == false)
                        {
                            right_free = false;
                            goto _diagonals;
                        }
                    }
                }
                right_free = true;
                neighbors.Add(node_map[x + 1, y].Clone());
                #endregion

                #region Diagonals
                _diagonals:
                if (up_free && left_free) //Up Left
                {
                    p.X = x - 1;
                    p.Y = y - 1;
                    if (closed_hash_set.Contains(p))
                    {
                        goto _leftDown;
                    }
                    for (int i = x - 1 - horizontal_node_extent + 1; i <= x - 1 + horizontal_node_extent - 1; i++)
                    {
                        for (int j = y - 1 - vertical_node_extent + 1; j <= y - 1 + vertical_node_extent - 1; j++)
                        {
                            if (IsWalkableAt(i, j, tile) == false)
                            {
                                goto _leftDown;
                            }
                        }
                    }

                    neighbors.Add(node_map[x - 1, y - 1].Clone());
                }
                _leftDown:
                if (left_free && down_free) //Left Down
                {
                    p.X = x - 1;
                    p.Y = y + 1;
                    if (closed_hash_set.Contains(p))
                    {
                        goto _downRight;
                    }
                    for (int i = x - 1 - horizontal_node_extent + 1; i <= x - 1 + horizontal_node_extent - 1; i++)
                    {
                        for (int j = y + 1 - vertical_node_extent + 1; j <= y + 1 + vertical_node_extent - 1; j++)
                        {
                            if (IsWalkableAt(i, j, tile) == false)
                            {
                                goto _downRight;
                            }
                        }
                    }
                    neighbors.Add(node_map[x - 1, y + 1].Clone());
                }

                _downRight:
                if (down_free && right_free) //Down Right
                {
                    p.X = x + 1;
                    p.Y = y + 1;
                    if (closed_hash_set.Contains(p))
                    {
                        goto _upRight;
                    }
                    for (int i = x + 1 - horizontal_node_extent + 1; i <= x + 1 + horizontal_node_extent - 1; i++)
                    {
                        for (int j = y + 1 - vertical_node_extent + 1; j <= y + 1 + vertical_node_extent - 1; j++)
                        {
                            if (IsWalkableAt(i, j, tile) == false)
                            {
                                goto _upRight;
                            }
                        }
                    }
                    neighbors.Add(node_map[x + 1, y + 1].Clone());
                }

                _upRight:
                if (up_free && right_free) //Up Right
                {
                    p.X = x + 1;
                    p.Y = y - 1;
                    if (closed_hash_set.Contains(p))
                    {
                        goto _return;
                    }
                    for (int i = x + 1 - horizontal_node_extent + 1; i <= x + 1 + horizontal_node_extent - 1; i++)
                    {
                        for (int j = y - 1 - vertical_node_extent + 1; j <= y - 1 + vertical_node_extent - 1; j++)
                        {
                            if (IsWalkableAt(i, j, tile) == false)
                            {
                                goto _return;
                            }
                        }
                    }
                    neighbors.Add(node_map[x + 1, y - 1].Clone());
                }
                #endregion
            }
            _return:
            return neighbors;
        }

        public float GetDistanceToNode(Vector2 from_int_coords, Vector2 to_int_coords)
        {
            if (from_int_coords.X != to_int_coords.X && from_int_coords.Y != to_int_coords.Y)
            {
                return DIAGONAL_DISTANCE;
            }
            return CARDINAL_DISTANCE;
        }

        public Node[,] NodeMap
        {
            get { return node_map; }
            set { node_map = value; }
        }

        public int TileSize
        {
            get { return tile_size; }
            set { tile_size = value; }
        }
    }
}
