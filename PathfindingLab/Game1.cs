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
        Stopwatch sw;

        bool dijkstra;
        const int tile_size = 32;


        List<Node> closed_list;

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
            agent_pos = (agent_tile * new Point(tile_size)).ToVector2();
            path_nodes = new List<Node>();

            sw = new Stopwatch();

            pathfinder = new Pathfinding(node_map, tile_size);
            closed_list = new List<Node>();
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
            agent_pos = (agent_tile * new Point(tile_size)).ToVector2();

            if (x_tile >= node_map.GetLength(0) || x_tile <= 0 || y_tile >= node_map.GetLength(1) || y_tile <= 0)
            {
                return;
            }

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
                agent_tile = (agent_pos / new Vector2(tile_size)).ToPoint();
                sw.Restart();
                path_nodes = pathfinder.Find(node_map[agent_tile.X, agent_tile.Y], node_map[x_tile, y_tile], ref closed_list, dijkstra);
                sw.Stop();
                
                position_of_path_tiles.Clear();

                for (int i = path_nodes.Count - 2; i >= 0; i--) //We substract 1 because we don't want to consider the start tile since it is the one the actor is in.
                {
                    position_of_path_tiles.Add(path_nodes[i].IntCoords.ToVector2() * new Vector2(tile_size));
                }
            }

            if (old_kstate.IsKeyUp(Keys.Space) && current_kstate.IsKeyDown(Keys.Space))
            {
                dijkstra = !dijkstra;
            }
            /*else if (old_kstate.IsKeyUp(Keys.W) && current_kstate.IsKeyDown(Keys.W))
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
            }*/


            Window.Title = x_tile.ToString() + ", " + y_tile.ToString() + "  Ticks: " + sw.ElapsedTicks.ToString() + "  Length: " + position_of_path_tiles.Count.ToString() + (dijkstra == true ? "  Dijkstra" : "  A*");

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
                spriteBatch.Draw(considered_tile, closed_list[i].IntCoords.ToVector2() * new Vector2(tile_size), Color.White);
            }
        }
    }
}
