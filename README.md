# ResearchTopic: JumpPointSearch

The Jump Point Search algorithm is similar to the A* algorithm used for pathfinding from a single start to a single destiation node.
While A* is very good in most cases, it does perform badly with uniform cost grids.

The Jump Point Search algorithm (JPS) improves on the A* algorithm when it comes to uniform cost grids.
It doesnt search every possible path since all paths have equal cost so it stores less in the open and closest list, making it a better choise for uniform cost grids.
