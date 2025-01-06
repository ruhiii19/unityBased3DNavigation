using System.Collections.Generic;
using UnityEngine;

public class GridManager : MonoBehaviour
{
    public float gridSize = 0.1f; // Size of each cube
    public int gridWidth = 10; // Width of the grid
    public int gridHeight = 10; // Height of the grid
    public int gridDepth = 10; // Depth of the grid

    public Node[,,] grid; // Grid of nodes
    public Transform startTransform; // Transform of the start point
    public Transform targetTransform; // Transform of the target point
    public LayerMask obstacleLayer; // Layer for obstacles

    private List<Node> path = new List<Node>();

    void Start()
    {
        GenerateGrid();
        UpdateObstacleNodes();
    }

    void Update()
    {
        if (startTransform && targetTransform)
        {
            path = FindPath(startTransform.position, targetTransform.position);
        }
    }

    void GenerateGrid()
    {
        grid = new Node[gridWidth, gridHeight, gridDepth];
        Vector3 gridOrigin = transform.position - new Vector3(gridWidth * gridSize / 2f, gridHeight * gridSize / 2f, gridDepth * gridSize / 2f);

        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                for (int z = 0; z < gridDepth; z++)
                {
                    Vector3 position = gridOrigin + new Vector3(x * gridSize, y * gridSize, z * gridSize);
                    grid[x, y, z] = new Node(position, true); // Initialize all nodes as walkable
                }
            }
        }
    }

    void UpdateObstacleNodes()
    {
        foreach (Node node in grid)
        {
            Collider[] colliders = Physics.OverlapBox(node.position, new Vector3(gridSize / 2f, gridSize / 2f, gridSize / 2f), Quaternion.identity, obstacleLayer);
            node.walkable = colliders.Length == 0;
        }
    }

    public Node GetNodeFromWorldPosition(Vector3 worldPosition)
    {
        int x = Mathf.FloorToInt((worldPosition.x - transform.position.x + (gridWidth * gridSize / 2f)) / gridSize);
        int y = Mathf.FloorToInt((worldPosition.y - transform.position.y + (gridHeight * gridSize / 2f)) / gridSize);
        int z = Mathf.FloorToInt((worldPosition.z - transform.position.z + (gridDepth * gridSize / 2f)) / gridSize);

        // Ensure that the indices are within bounds
        if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight && z >= 0 && z < gridDepth)
        {
            return grid[x, y, z];
        }

        Debug.LogWarning($"Node at ({x}, {y}, {z}) is out of bounds.");
        return null;
    }

    public List<Node> FindPath(Vector3 startPosition, Vector3 targetPosition)
    {
        Node startNode = GetNodeFromWorldPosition(startPosition);
        Node targetNode = GetNodeFromWorldPosition(targetPosition);

        if (startNode == null || targetNode == null || !startNode.walkable || !targetNode.walkable)
        {
            Debug.Log("Start or Target node is invalid.");
            return null;
        }

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node currentNode = GetLowestFCostNode(openSet);
            if (currentNode == targetNode)
            {
                return RetracePath(startNode, targetNode);
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            foreach (Node neighbor in GetNeighbors(currentNode))
            {
                if (!neighbor.walkable || closedSet.Contains(neighbor))
                {
                    continue;
                }

                float newGCost = currentNode.gCost + GetDistance(currentNode, neighbor);
                if (newGCost < neighbor.gCost || !openSet.Contains(neighbor))
                {
                    neighbor.gCost = newGCost;
                    neighbor.hCost = GetDistance(neighbor, targetNode);
                    neighbor.parent = currentNode;

                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                }
            }
        }

        Debug.LogWarning("No path found.");
        return null; // No path found
    }

    Node GetLowestFCostNode(List<Node> nodes)
    {
        Node lowest = nodes[0];
        foreach (Node node in nodes)
        {
            if (node.fCost < lowest.fCost || (node.fCost == lowest.fCost && node.hCost < lowest.hCost))
            {
                lowest = node;
            }
        }
        return lowest;
    }

    void AddNeighborIfInBounds(List<Node> neighbors, int x, int y, int z)
    {
        if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight && z >= 0 && z < gridDepth)
        {
            neighbors.Add(grid[x, y, z]);
        }
    }

    List<Node> GetNeighbors(Node node)
    {
        List<Node> neighbors = new List<Node>();

        // Calculate node coordinates
        int x = Mathf.FloorToInt((node.position.x - transform.position.x + (gridWidth * gridSize / 2f)) / gridSize);
        int y = Mathf.FloorToInt((node.position.y - transform.position.y + (gridHeight * gridSize / 2f)) / gridSize);
        int z = Mathf.FloorToInt((node.position.z - transform.position.z + (gridDepth * gridSize / 2f)) / gridSize);

        // Check each neighbor's position in a 3D grid
        AddNeighborIfInBounds(neighbors, x - 1, y, z); // Left
        AddNeighborIfInBounds(neighbors, x + 1, y, z); // Right
        AddNeighborIfInBounds(neighbors, x, y - 1, z); // Down
        AddNeighborIfInBounds(neighbors, x, y + 1, z); // Up
        AddNeighborIfInBounds(neighbors, x, y, z - 1); // Back
        AddNeighborIfInBounds(neighbors, x, y, z + 1); // Forward

        return neighbors;
    }

    float GetDistance(Node nodeA, Node nodeB)
    {
        return Vector3.Distance(nodeA.position, nodeB.position);
    }

    List<Node> RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }

        path.Reverse();
        return path;
    }

    void OnDrawGizmos()
    {
        if (grid == null) return;

        // Draw the gridx`
        Gizmos.color = Color.white;
        foreach (Node node in grid)
        {
            Gizmos.DrawWireCube(node.position, new Vector3(gridSize, gridSize, gridSize));
        }

        // Draw the path found by A* algorithm
        Gizmos.color = Color.black;
        if (path != null)
        {
            foreach (Node node in path)
            {
                Gizmos.DrawWireCube(node.position, new Vector3(gridSize, gridSize, gridSize));
            }
        }
    }

    /*void OnDrawGizmos()
    {
        if (grid != null)
        {
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    for (int z = 0; z < gridDepth; z++)
                    {
                        Node currentNode = grid[x, y, z];
                        Gizmos.color = currentNode.walkable ? Color.white : Color.red;
                        Gizmos.DrawCube(currentNode.position, Vector3.one * (gridSize - 0.1f));
                    }
                }
            }
        }
    }*/

}
