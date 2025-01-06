using UnityEngine;

public class Node
{
    public Vector3 position; // Position in world space
    public bool walkable;    // Whether this node is walkable (not an obstacle)
    public float gCost;      // Cost from the start node
    public float hCost;      // Heuristic cost to the target node
    public Node parent;      // Parent node in the pathfinding algorithm

    public Node(Vector3 position, bool walkable)
    {
        this.position = position;
        this.walkable = walkable;
        this.gCost = float.MaxValue; // Initialize with a high value
        this.hCost = 0;
        this.parent = null;
    }

    public float fCost
    {
        get { return gCost + hCost; }
    }
}
