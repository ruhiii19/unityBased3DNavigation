using System.Collections.Generic;
using UnityEngine;

public enum Orientation
{
    Upright,
    TiltedX,
    TiltedZ
}

public class Pathfinding : MonoBehaviour
{
    public GridManager gridManager;
    public Transform seeker;
    public Transform target;
    public float moveSpeed = 0.5f; // Speed of the seeker movement

    private List<Node> currentPath = new List<Node>();
    private int currentPathIndex = 0; // To keep track of the current node in the path

    private Orientation currentOrientation = Orientation.Upright;

    private void Update()
    {
        if (seeker != null && target != null)
        {
            // Only find the path if the seeker or target has moved
            if (currentPath == null || currentPath.Count == 0)
            {
                FindPath(seeker.position, target.position);
            }

            // If a valid path exists, move the seeker along the path
            if (currentPath != null && currentPath.Count > 0)
            {
                MoveAlongPath();
            }
        }
    }

    void FindPath(Vector3 startPos, Vector3 targetPos)
    {
        Node startNode = gridManager.GetNodeFromWorldPosition(startPos);
        Node targetNode = gridManager.GetNodeFromWorldPosition(targetPos);

        if (startNode == null || targetNode == null) return;

        currentPath = AStar(startNode, targetNode);
        currentPathIndex = 0; // Reset the path index
    }

    void MoveAlongPath()
    {
        if (currentPathIndex < currentPath.Count)
        {
            Node targetNode = currentPath[currentPathIndex];
            Vector3 targetPosition = targetNode.position;

            // Try to find an orientation that allows the seeker to fit at the current target node
            if (!TryDynamicRotationFit(targetNode))
            {
                // If no orientation fits, stop movement
                return;
            }

            // Move seeker towards the next node
            seeker.position = Vector3.MoveTowards(seeker.position, targetPosition, moveSpeed * Time.deltaTime);

            // If seeker has reached the node, move to the next node in the path
            if (Vector3.Distance(seeker.position, targetPosition) < 0.1f)
            {
                currentPathIndex++;
            }
        }
    }

    bool TryDynamicRotationFit(Node node)
    {
        // Define the ranges and steps for testing different angles
        float[] angles = { 0, 15, 30, 45, -15, -30, -45 };

        foreach (float angleX in angles)
        {
            foreach (float angleZ in angles)
            {
                // Set the current rotation and check if the seeker can fit
                seeker.rotation = Quaternion.Euler(angleX, 0, angleZ);
                if (CanFit(node))
                {
                    // If the seeker can fit with this rotation, keep the rotation and return true
                    return true;
                }
            }
        }

        // Revert to upright if no rotation works
        seeker.rotation = Quaternion.identity;
        return false;
    }

    bool CanFit(Node node)
    {
        // Adjust the size based on the seeker's current orientation and rotation
        Vector3 seekerSize = GetSeekerBoundsBasedOnRotation();

        // Use Physics.OverlapBox to check if the seeker can fit in the given node space
        Collider[] hitColliders = Physics.OverlapBox(node.position, seekerSize / 2, seeker.rotation);
        foreach (var hitCollider in hitColliders)
        {
            if (hitCollider.CompareTag("Obstacle"))
            {
                return false; // If any obstacle is found, return false
            }
        }
        return true;
    }

    Vector3 GetSeekerBoundsBasedOnRotation()
    {
        // Get the Capsule Collider component attached to the seeker
        CapsuleCollider capsuleCollider = seeker.GetComponent<CapsuleCollider>();
        if (capsuleCollider == null)
        {
            Debug.LogError("Capsule Collider not found on the seeker!");
            return Vector3.zero; // Return zero if the collider is not found
        }

        // Calculate the bounds based on the current orientation and rotation of the capsule
        Vector3 bounds = Vector3.zero;

        switch (capsuleCollider.direction)
        {
            case 0: // X-axis
                bounds = new Vector3(capsuleCollider.height, capsuleCollider.radius * 2, capsuleCollider.radius * 2);
                break;
            case 1: // Y-axis (default, upright position)
                bounds = new Vector3(capsuleCollider.radius * 2, capsuleCollider.height, capsuleCollider.radius * 2);
                break;
            case 2: // Z-axis
                bounds = new Vector3(capsuleCollider.radius * 2, capsuleCollider.radius * 2, capsuleCollider.height);
                break;
        }

        // Apply the rotation to the bounds based on the seeker's current orientation
        bounds = Quaternion.Euler(seeker.rotation.eulerAngles) * bounds;

        return bounds;
    }


    void OnDrawGizmos()
    {
        if (currentPath.Count > 0)
        {
            Gizmos.color = Color.black;

            foreach (Node node in currentPath)
            {
                Gizmos.DrawCube(node.position, new Vector3(gridManager.gridSize, gridManager.gridSize, gridManager.gridSize));
            }
        }
    }

    List<Node> AStar(Node startNode, Node targetNode)
    {
        List<Node> openSet = new List<Node> { startNode };
        HashSet<Node> closedSet = new HashSet<Node>();
        Dictionary<Node, Node> cameFrom = new Dictionary<Node, Node>();

        Dictionary<Node, float> gScore = new Dictionary<Node, float>();
        Dictionary<Node, float> fScore = new Dictionary<Node, float>();

        foreach (Node node in gridManager.grid)
        {
            gScore[node] = float.MaxValue;
            fScore[node] = float.MaxValue;
        }

        gScore[startNode] = 0;
        fScore[startNode] = Heuristic(startNode, targetNode);

        while (openSet.Count > 0)
        {
            Node current = GetLowestScoreNode(openSet, fScore);

            if (current == targetNode)
            {
                return ReconstructPath(cameFrom, current);
            }

            openSet.Remove(current);
            closedSet.Add(current);

            foreach (Node neighbor in GetNeighbors(current))
            {
                if (!neighbor.walkable || closedSet.Contains(neighbor))
                {
                    continue;
                }

                float tentativeGScore = gScore[current] + Heuristic(current, neighbor);

                if (!openSet.Contains(neighbor))
                {
                    openSet.Add(neighbor);
                }
                else if (tentativeGScore >= gScore[neighbor])
                {
                    continue;
                }

                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;
                fScore[neighbor] = gScore[neighbor] + Heuristic(neighbor, targetNode);
            }
        }

        Debug.LogWarning($"No path found");
        return new List<Node>(); // Return an empty path if no path is found
    }

    Node GetLowestScoreNode(List<Node> nodes, Dictionary<Node, float> scores)
    {
        Node lowest = nodes[0];
        foreach (Node node in nodes)
        {
            if (scores[node] < scores[lowest])
            {
                lowest = node;
            }
        }
        return lowest;
    }

    List<Node> GetNeighbors(Node node)
    {
        List<Node> neighbors = new List<Node>();

        Vector3[] directions = {
            Vector3.right, Vector3.left, Vector3.up, Vector3.down, Vector3.forward, Vector3.back
        };

        foreach (Vector3 dir in directions)
        {
            Vector3 neighborPosition = node.position + dir * gridManager.gridSize;
            Node neighborNode = gridManager.GetNodeFromWorldPosition(neighborPosition);

            if (neighborNode != null)
            {
                neighbors.Add(neighborNode);
            }
        }

        return neighbors;
    }

    float Heuristic(Node a, Node b)
    {
        return Vector3.Distance(a.position, b.position);
    }

    List<Node> ReconstructPath(Dictionary<Node, Node> cameFrom, Node current)
    {
        List<Node> totalPath = new List<Node> { current };
        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            totalPath.Add(current);
        }
        totalPath.Reverse();
        return totalPath;
    }
}
