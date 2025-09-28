// AvoiderPlugin/PoissonDiscSampler.cs
// Bridson-style Poisson-disc sampling (2D) adapted for Unity-friendly use
using System;
using System.Collections.Generic;
using UnityEngine;

namespace AvoiderPlugin
{
    /// <summary>
    /// Uniform Poisson-disc sampling in [0..width]x[0..height].
    /// Returns IEnumerable of Vector2 sample positions.
    /// </summary>
    public sealed class PoissonDiscSampler
    {
        readonly float width, height, radius, cellSize;
        readonly int k; // attempts per point

        readonly Vector2[,] grid;
        readonly float invCell;
        readonly List<Vector2> active = new();
        readonly System.Random rng = new();

        int gridW, gridH;

        public PoissonDiscSampler(float width, float height, float radius, int attemptsPerPoint = 30)
        {
            this.width = Mathf.Max(0.001f, width);
            this.height = Mathf.Max(0.001f, height);
            this.radius = Mathf.Max(0.0001f, radius);
            this.k = Mathf.Max(1, attemptsPerPoint);

            cellSize = radius / Mathf.Sqrt(2f);
            invCell = 1f / cellSize;
            gridW = Mathf.CeilToInt(width * invCell);
            gridH = Mathf.CeilToInt(height * invCell);
            grid = new Vector2[gridW, gridH];

            for (int y = 0; y < gridH; y++)
                for (int x = 0; x < gridW; x++)
                    grid[x, y] = INVALID;

            // seed with a random initial point
            var init = new Vector2(RandomRange(0f, width), RandomRange(0f, height));
            Place(init);
            active.Add(init);
        }

        static readonly Vector2 INVALID = new(float.NegativeInfinity, float.NegativeInfinity);

        public IEnumerable<Vector2> Samples()
        {
            // yield the initial point immediately
            foreach (var s in Iterate())
                yield return s;
        }

        IEnumerable<Vector2> Iterate()
        {
            // emit the seed
            yield return active[0];

            while (active.Count > 0)
            {
                int idx = rng.Next(active.Count);
                Vector2 point = active[idx];
                bool found = false;

                for (int i = 0; i < k; i++)
                {
                    var cand = GenerateAround(point);
                    if (IsValid(cand))
                    {
                        Place(cand);
                        active.Add(cand);
                        found = true;
                        yield return cand;
                        break;
                    }
                }

                if (!found)
                {
                    // retire this point
                    active[idx] = active[^1];
                    active.RemoveAt(active.Count - 1);
                }
            }
        }

        Vector2 GenerateAround(Vector2 p)
        {
            // sample radius in [r, 2r), angle in [0, 2pi)
            float r = radius * (1f + RandomRange(0f, 1f));
            float ang = RandomRange(0f, Mathf.PI * 2f);
            return new Vector2(
                p.x + Mathf.Cos(ang) * r,
                p.y + Mathf.Sin(ang) * r
            );
        }

        bool IsValid(Vector2 p)
        {
            if (p.x < 0f || p.x >= width || p.y < 0f || p.y >= height) return false;

            var gc = GridCoord(p);
            int xmin = Mathf.Max(0, gc.x - 2);
            int xmax = Mathf.Min(gridW - 1, gc.x + 2);
            int ymin = Mathf.Max(0, gc.y - 2);
            int ymax = Mathf.Min(gridH - 1, gc.y + 2);

            for (int y = ymin; y <= ymax; y++)
                for (int x = xmin; x <= xmax; x++)
                {
                    var s = grid[x, y];
                    if (!float.IsNegativeInfinity(s.x))
                    {
                        if ((s - p).sqrMagnitude < radius * radius) return false;
                    }
                }

            return true;
        }

        void Place(Vector2 p)
        {
            var gc = GridCoord(p);
            grid[gc.x, gc.y] = p;
        }

        (int x, int y) GridCoord(Vector2 p)
        {
            int gx = Mathf.Clamp((int)(p.x * invCell), 0, gridW - 1);
            int gy = Mathf.Clamp((int)(p.y * invCell), 0, gridH - 1);
            return (gx, gy);
        }

        float RandomRange(float min, float max) => (float)(rng.NextDouble() * (max - min) + min);
    }
}
