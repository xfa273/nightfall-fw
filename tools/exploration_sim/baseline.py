"""Mode 1 case 1 goal-then-full Adachi exploration, without hardware actions.

Coordinates are [y][x], y increases north. Heading N/E/S/W is 0/1/2/3;
wall and known masks use 1 << heading (unlike the reversed firmware nibble).
Only observed walls may be supplied; unknown edges remain optimistic passages.
"""

from __future__ import annotations

from collections import deque


DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)
REL_PRIORITY = (0, 1, 3, 2)
INF = 0xFFFF


def _smap(walls, targets, pos):
    height, width = len(walls), len(walls[0])
    steps = [[INF] * width for _ in range(height)]
    queue = deque()
    for x, y in targets:
        if 0 <= x < width and 0 <= y < height and steps[y][x] == INF:
            steps[y][x] = 0
            queue.append((x, y))
    px, py = pos
    # Firmware stops BFS as soon as the current cell is discovered. Preserving
    # this also preserves what the forward lookahead can see in the step map.
    while queue and steps[py][px] == INF:
        x, y = queue.popleft()
        for d in range(4):
            nx, ny = x + DX[d], y + DY[d]
            if walls[y][x] & (1 << d):
                continue
            if 0 <= nx < width and 0 <= ny < height and steps[ny][nx] == INF:
                steps[ny][nx] = steps[y][x] + 1
                queue.append((nx, ny))
    return steps


def _choose(walls, steps, pos, heading):
    height, width = len(walls), len(walls[0])
    x, y = pos
    if steps[y][x] == INF:
        return None
    for rel in REL_PRIORITY:
        d = (heading + rel) % 4
        nx, ny = x + DX[d], y + DY[d]
        if walls[y][x] & (1 << d):
            continue
        if 0 <= nx < width and 0 <= ny < height and steps[ny][nx] < steps[y][x]:
            return d
    return None


def next_baseline(known, walls, visited, goals, pos, heading, phase="goal") -> dict:
    """Return an absolute next direction and current goal/full/done phase.

    Caller observes the current cell and marks it visited before calling. The
    full phase seeks every unvisited *cell*, even if all its edges are known.
    Completion leaves the robot where exploration ends; there is no home return.
    This reproduces F413 routing semantics and F405 next-cell decisions, while
    omitting F405's goal stop/back-up/relaunch transition from the timing model.
    """
    if phase not in ("goal", "full", "done"):
        raise ValueError(f"Unknown baseline phase: {phase}")
    if not walls or not walls[0]:
        raise ValueError("Maze must not be empty")
    height, width = len(walls), len(walls[0])
    if any(len(row) != width for row in walls):
        raise ValueError("Wall matrix must be rectangular")
    for matrix in (known, visited):
        if len(matrix) != height or any(len(row) != width for row in matrix):
            raise ValueError("Known, walls and visited matrices must match")
    x, y = pos
    if not (0 <= x < width and 0 <= y < height and 0 <= heading < 4):
        raise ValueError("Invalid robot pose")
    result = {"direction": None, "phase": phase, "known_straight": False,
              "next_is_turn90": False}
    if phase == "done":
        return result
    goal_set = {tuple(goal) for goal in goals}
    if phase == "goal" and (x, y) in goal_set:
        phase = "full"
    if phase == "full":
        # The firmware forces start visited even when supplied state says not.
        targets = [(ix, iy) for iy in range(height) for ix in range(width)
                   if not visited[iy][ix] and (ix, iy) != (0, 0)]
    else:
        targets = sorted(goal_set, key=lambda p: (p[1], p[0]))
    # AND with the known mask prevents accidental ground-truth disclosure.
    observed_walls = [[walls[iy][ix] & known[iy][ix] for ix in range(width)]
                      for iy in range(height)]
    steps = _smap(observed_walls, targets, (x, y))
    direction = _choose(observed_walls, steps, (x, y), heading)
    result["phase"] = phase
    if direction is None:
        if phase == "full":
            result["phase"] = "done"
        else:
            result["error"] = "goal_unreachable"
        return result
    result["direction"] = direction
    if direction == heading:
        nx, ny = x + DX[heading], y + DY[heading]
        # F413 only looks beyond a forward destination if it was visited.
        if visited[ny][nx]:
            following = _choose(observed_walls, steps, (nx, ny), heading)
            result["known_straight"] = following == heading
            result["next_is_turn90"] = following is not None and (following - heading) % 4 in (1, 3)
    return result

