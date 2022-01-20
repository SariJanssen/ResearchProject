# ResearchTopic: JumpPointSearch (JPS) algorithm

The Jump Point Search algorithm is similar to the A* algorithm used for pathfinding from a single start to a single destination node.
While A* is very good in most cases, it does perform badly with uniform cost grids.

The Jump Point Search algorithm (JPS) improves on the A* algorithm when it comes to uniform cost grids.
It doesn't thoroughly search every possible path since all paths have equal cost so it stores less in the open and closest list, making it a better choice in these situations.

## Grid Requirements
The main requirement for the JPS algorithm is for the grid to be *uniform in cost*. This means that every available path will have a cost of 1 for horizontal and vertical travelling, meanwhile diagonal travelling will have a larger cost.

Another requirement for these grids is that it should be an *8-way grid* since the algorithm uses it by also searching diagonally.

## Eliminating intermediate nodes
The main reason JPS performs better in these grids is because it eliminates the nodes it sees as unimportant.
Unimportant nodes are those that are easily explored by other paths, what we are looking for are ***forced neighbours***.

In the image below you can see an example of a forced neighbour (purple). This is because an obstacle is blocking the way (dark grey).
This means that this particular neighbour is only easily reachable through this path.
The JPS algorithm will not add any nodes to the open and closed list until it reaches one of these forced neighbours. These are the only ones it will add, resulting in a faster pathfinding algorithm.

![afbeelding](https://user-images.githubusercontent.com/78912061/150272514-0dfac00e-d60d-4267-bc53-7090067ea8e4.png)

## Example pathfinding
As you can see in the following pictures, A* will be more methodical in searching for the goal node, it will cover every node along the way.
JPS on the other hand will explore a lot more, maybe some areas even repetitively but only a few nodes are actually actively considered.

A* pathfinding

![afbeelding](https://user-images.githubusercontent.com/78912061/150273224-9900d1f9-7689-43cc-bcc0-414cb2cdb5fd.png)

JPS pathfinding

![afbeelding](https://user-images.githubusercontent.com/78912061/150273262-79ced0ed-02ee-4322-b89a-f30df987ecde.png)
