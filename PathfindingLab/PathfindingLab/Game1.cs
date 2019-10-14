using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;


using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace PathfindingLab
{
    /// <summary>
    /// This is the main type for your game.
    /// </summary>
    public class Game1 : Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        Texture2D path_tile, obstacle_tile, grid_tile, considered_tile, agent;
        SpriteFont font;
        Node[,] node_map;
        int amount_horizontal_grid;
        int amount_vertical_grid;
        MouseState current_mstate, old_mstate;
        KeyboardState current_kstate, old_kstate;
        List<Vector2> position_of_obstacles;
        List<Vector2> position_of_path_tiles;
        
        Pathfinding pathfinder;
        List<Node> path_nodes;

        Point agent_tile = new Point(28, 15);
        Vector2 agent_pos;
        Vector2 agent_center;
        Stopwatch sw;

        bool show_help;
        bool dijkstra;
        bool path_found;
        const int tile_size = 32;
        float speed;
        Vector2 offset_to_center;

        List<Point> closed_list;
        private Vector2 parent_node_pos;
        private float distance_to_target;
        private float distance_to_next_tile;
        private Node end_node;

        public Game1()
        {
            graphics = new GraphicsDeviceManager(this);
            graphics.PreferredBackBufferWidth = 1024;
            graphics.PreferredBackBufferHeight = 608;
            Content.RootDirectory = "Content";
            IsMouseVisible = true;
            amount_horizontal_grid = graphics.PreferredBackBufferWidth / tile_size;
            amount_vertical_grid = graphics.PreferredBackBufferHeight / tile_size;
            node_map = new Node[amount_horizontal_grid, amount_vertical_grid];
            for (int i = 0; i < node_map.GetLength(0); i++)
            {
                for (int j = 0; j < node_map.GetLength(1); j++)
                {
                    node_map[i, j] = new Node(false, new Point(i, j));
                }
            }
            position_of_obstacles = new List<Vector2>(amount_horizontal_grid * amount_vertical_grid);
            position_of_path_tiles = new List<Vector2>();
            closed_list = new List<Point>();
            agent_pos = (agent_tile * new Point(tile_size)).ToVector2();
            speed = 200f;
            path_nodes = new List<Node>();

            sw = new Stopwatch();

            pathfinder = new Pathfinding(tile_size, 500, 1, 1);
            pathfinder.NodeMap = node_map;
        }

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        protected override void Initialize()
        {
            // TODO: Add your initialization logic here

            base.Initialize();
        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch(GraphicsDevice);

            grid_tile = Content.Load<Texture2D>("GridTile");
            obstacle_tile = Content.Load<Texture2D>("ObstacleTile");
            path_tile = Content.Load<Texture2D>("PathTile");
            considered_tile = Content.Load<Texture2D>("ConsideredTile");
            agent = Content.Load<Texture2D>("Agent");
            font = Content.Load<SpriteFont>("font");
        }

        /// <summary>
        /// UnloadContent will be called once per game and is the place to unload
        /// game-specific content.
        /// </summary>
        protected override void UnloadContent()
        {
            // TODO: Unload any non ContentManager content here
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {
            if (Keyboard.GetState().IsKeyDown(Keys.Escape))
                Exit();

            old_mstate = current_mstate;
            current_mstate = Mouse.GetState();
            old_kstate = current_kstate;
            current_kstate = Keyboard.GetState();
            int x_tile = current_mstate.Position.X / tile_size;
            int y_tile = current_mstate.Position.Y / tile_size;

            if (x_tile >= node_map.GetLength(0) || x_tile < 0 || y_tile >= node_map.GetLength(1) || y_tile < 0)
            {
                return;
            }
            agent_tile = (agent_pos / new Vector2(tile_size)).ToPoint();

            Vector2 pos = new Vector2(x_tile * tile_size, y_tile * tile_size);
            
            if (old_mstate.LeftButton == ButtonState.Released && current_mstate.LeftButton == ButtonState.Pressed)
            {
                if (position_of_obstacles.Contains(pos) == false)
                {
                    position_of_obstacles.Add(pos);
                    node_map[x_tile, y_tile].IsObstacle = true;
                }
                else
                {
                    position_of_obstacles.Remove(pos);
                    node_map[x_tile, y_tile].IsObstacle = false;
                }
            }
            else if (old_mstate.RightButton == ButtonState.Released && current_mstate.RightButton == ButtonState.Pressed)
            {
                if (node_map[x_tile, y_tile].IsObstacle)
                {
                    return;
                }
                
                sw.Restart();

                position_of_path_tiles.Clear();
                closed_list.Clear();

                FindImplicitPathTo(new Point(x_tile, y_tile));
                sw.Stop();
            }

            if (old_kstate.IsKeyUp(Keys.Space) && current_kstate.IsKeyDown(Keys.Space))
            {
                dijkstra = !dijkstra;
            }
            else if (old_kstate.IsKeyUp(Keys.Space) && current_kstate.IsKeyDown(Keys.Space))
            {
                show_help = !show_help;
            }

            if (path_found == false)
            {
                if (old_kstate.IsKeyUp(Keys.W) && current_kstate.IsKeyDown(Keys.W))
                {
                    agent_tile.Y -= 1;

                }
                else if (old_kstate.IsKeyUp(Keys.A) && current_kstate.IsKeyDown(Keys.A))
                {
                    agent_tile.X -= 1;
                }
                else if (old_kstate.IsKeyUp(Keys.S) && current_kstate.IsKeyDown(Keys.S))
                {
                    agent_tile.Y += 1;
                }
                else if (old_kstate.IsKeyUp(Keys.D) && current_kstate.IsKeyDown(Keys.D))
                {
                    agent_tile.X += 1;
                }
                agent_pos = new Vector2(agent_tile.X * agent.Bounds.Width, agent_tile.Y * agent.Bounds.Height);
            }

            Window.Title = x_tile.ToString() + ", " + y_tile.ToString() + "  Ticks: " + sw.ElapsedTicks.ToString() + "  Length: " + position_of_path_tiles.Count.ToString() + (dijkstra == true ? "  Dijkstra" : "  A*" + "  Press F1 for help");

            if (path_found)
            {
                FollowImplicitPath(gameTime);
            }

            base.Update(gameTime);
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);

            spriteBatch.Begin();
            DrawGrid();
            DrawConsideredTiles();
            DrawPath();
            DrawObstacles();
            DrawAgent();
            spriteBatch.End();

            base.Draw(gameTime);
        }

        void DrawGrid()
        {
            for (int j = 0; j < amount_vertical_grid; j++)
            {
                for (int i = 0; i < amount_horizontal_grid; i++)
                {
                    spriteBatch.Draw(grid_tile, new Vector2(tile_size * i, tile_size * j), Color.White);
                }
            }
        }

        void DrawObstacles()
        {
            for (int i = 0; i < position_of_obstacles.Count; i++)
            {
                spriteBatch.Draw(obstacle_tile, position_of_obstacles[i], Color.White);
            }
        }

        void DrawAgent()
        {
            spriteBatch.Draw(agent, agent_pos, Color.White);
        }

        void DrawPath()
        {
            for (int i = 0; i < position_of_path_tiles.Count; i++)
            {
                spriteBatch.Draw(path_tile, position_of_path_tiles[i], Color.White);
            }
        }

        void DrawConsideredTiles()
        {
            for (int i = 0; i < closed_list.Count; i++)
            {
                spriteBatch.Draw(considered_tile, closed_list[i].ToVector2() * new Vector2(tile_size), Color.White);
            }
        }

        public void FindImplicitPathTo(Point destination_node)
        {
            agent_center = CalculateAgentCenter();
            Point start_node = new Point((int)(agent_center.X / Pathfinding.NODE_SIZE), (int)(agent_center.Y / Pathfinding.NODE_SIZE));
            
            if (start_node != destination_node) //We don't want to the same place we start
            {
                path_found = pathfinder.FindImplicitPath(destination_node, start_node, ref end_node, dijkstra, ref closed_list, ref position_of_path_tiles); //FOR DEBUGGING
                //path_found = pathfinder.FindImplicitPath(destination_node, start_node, ref end_node, dijkstra); //FOR PRODUCTION
                if (path_found == true)
                {
                    offset_to_center = (agent_center - agent_pos) - new Vector2(Pathfinding.NODE_SIZE * 0.5f);
                    parent_node_pos = new Vector2(end_node.X * Pathfinding.NODE_SIZE, end_node.Y * Pathfinding.NODE_SIZE) - offset_to_center;
                    distance_to_target = (parent_node_pos - agent_pos).Length();
                    distance_to_next_tile = 0;
                }
            }
        }

        public void FollowImplicitPath(GameTime gameTime)
        {
            if (path_found == true)
            {
                float ds = speed * (float)gameTime.ElapsedGameTime.TotalSeconds;
                Vector2 dir = parent_node_pos - agent_pos;
                dir.Normalize();
                if (float.IsNaN(dir.X) || float.IsNaN(dir.Y) || float.IsInfinity(dir.X) || float.IsInfinity(dir.Y))
                {
                    dir = Vector2.Zero;
                }
                distance_to_next_tile += ds;
                if (distance_to_next_tile - distance_to_target >= 0)
                {
                    agent_pos = new Vector2(agent_pos.X, parent_node_pos.Y);
                    agent_pos = new Vector2(parent_node_pos.X, agent_pos.Y);

                    end_node = end_node.Parent;

                    if (end_node == null)
                    {
                        path_found = false;
                        return;
                    }

                    parent_node_pos = new Vector2(end_node.X * Pathfinding.NODE_SIZE, end_node.Y * Pathfinding.NODE_SIZE) - offset_to_center;
                    distance_to_target = pathfinder.GetDistanceToNode(parent_node_pos, agent_pos);
                    distance_to_next_tile = 0;
                }
                else
                {
                    agent_pos = agent_pos + new Vector2(0, dir.Y * ds);
                    agent_pos = agent_pos + new Vector2(dir.X * ds, 0);
                }
            }
        }

        public Vector2 CalculateAgentCenter()
        {
            Vector2 center = new Vector2();
            center.X = agent_pos.X + agent.Bounds.Width / 2;
            center.Y = agent_pos.Y + agent.Bounds.Height / 2;

            return center;
        }
    }
}
