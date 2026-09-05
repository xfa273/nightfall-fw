"""Partial-observation exploration with a shared clock and motion model.

The policy and route oracle receive only Knowledge. Truth is read exclusively
by observe() and the collision assertion in move(). Directions/bits are NESW.
"""
from __future__ import annotations

import heapq
import math
from typing import Callable

from .baseline import next_baseline

DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)


class Knowledge:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.known = [[0] * width for _ in range(height)]
        self.walls = [[0] * width for _ in range(height)]
        self.visited = [[False] * width for _ in range(height)]
        self.count = 0
        for y in range(height):
            for x in range(width):
                for d in range(4):
                    if not self.inside(x + DX[d], y + DY[d]):
                        self.set_edge(x, y, d, True)

    def inside(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def set_edge(self, x, y, d, wall):
        if self.known[y][x] & (1 << d):
            if bool(self.walls[y][x] & (1 << d)) != bool(wall):
                raise ValueError("Conflicting wall observation")
            return None
        self.known[y][x] |= 1 << d
        if wall:
            self.walls[y][x] |= 1 << d
        nx, ny, od = x + DX[d], y + DY[d], (d + 2) % 4
        if self.inside(nx, ny):
            self.known[ny][nx] |= 1 << od
            if wall:
                self.walls[ny][nx] |= 1 << od
        self.count += 1
        return [x, y, d, int(bool(wall))]

    def observe(self, truth, pos, heading):
        """Ideal front/left/right sensors at a cell centre; no look-through."""
        x, y = pos
        changes = []
        for d in (heading, (heading + 1) % 4, (heading + 3) % 4):
            change = self.set_edge(x, y, d, truth[y][x] & (1 << d))
            if change is not None:
                changes.append(change)
        self.visited[y][x] = True
        return changes

    def map_for(self, optimistic):
        return [[self.walls[y][x] | (0 if optimistic else (15 ^ self.known[y][x]))
                 for x in range(self.width)] for y in range(self.height)]

    def open(self, x, y, d):
        return bool(self.known[y][x] & (1 << d)) and not self.walls[y][x] & (1 << d)

    def edge_unknown(self, edge):
        x, y, d = edge
        return not self.known[y][x] & (1 << d)


def navigation(knowledge, pos, heading, profile, optimistic=False):
    """Heading-dependent approximate travel-time Dijkstra for access routes.

    Probe access may use hopeful unknown shortcuts, penalized for uncertainty;
    the next real step is still required to be observed open. Home uses K only.
    """
    start = (*pos, heading)
    distances, parent = {start: 0.0}, {}
    queue = [(0.0, start)]
    base = profile["search_speed_mm_s"]
    cell = profile["cell_mm"]
    while queue:
        cost, state = heapq.heappop(queue)
        if cost != distances[state]:
            continue
        x, y, hd = state
        # Preserve deterministic straight/right/left/back tie order.
        for d in (hd, (hd + 1) % 4, (hd + 3) % 4, (hd + 2) % 4):
            if knowledge.walls[y][x] & (1 << d) or (not optimistic and not knowledge.open(x, y, d)):
                continue
            nx, ny = x + DX[d], y + DY[d]
            if not knowledge.inside(nx, ny):
                continue
            turn = (d - hd) % 4
            speed = (profile["known_speed_mm_s"] if
                     knowledge.visited[y][x] and knowledge.visited[ny][nx] else base)
            step = cell / speed
            if turn in (1, 3):
                step = profile["turn90_s"]
            elif turn == 2:
                step = profile["uturn_s"] + cell / base
            if optimistic and not knowledge.visited[ny][nx]:
                step *= 1.3
            nxt = (nx, ny, d)
            new_cost = cost + step
            if new_cost < distances.get(nxt, math.inf) - 1e-10:
                distances[nxt] = new_cost
                parent[nxt] = state
                heapq.heappush(queue, (new_cost, nxt))
    return distances, parent


def route_to_cell(distances, parents, pos):
    candidates = [(*pos, d) for d in range(4) if (*pos, d) in distances]
    if not candidates:
        return math.inf, []
    end = min(candidates, key=lambda s: (distances[s], s[2]))
    cost = distances[end]
    path = []
    while end in parents:
        path.append(end)
        end = parents[end]
    return cost, list(reversed(path))


def choose_probe(knowledge, required_edges, pos, heading, profile):
    """Visit a reachable observer of the optimistic route's unknown edges.

    A target earns credit only for route-relevant edges it will observe. The
    score balances travel time and the number of these edges (no truth access).
    Replan after every observation, committing to one movement at a time.
    """
    dist, parents = navigation(knowledge, pos, heading, profile, optimistic=True)
    relevant = set()
    candidates = {}
    for x, y, d in required_edges:
        if not knowledge.edge_unknown((x, y, d)):
            continue
        edge = tuple(sorted(((x, y), (x + DX[d], y + DY[d]))))
        relevant.add(edge)
        for endpoint in edge:
            if knowledge.inside(*endpoint):
                candidates.setdefault(endpoint, set()).add(edge)
    options = []
    for cell, edges in candidates.items():
        cost, path = route_to_cell(dist, parents, cell)
        if not path:
            continue
        # More information at the same observer is valuable, but distance
        # dominates. The constant prevents arbitrarily large information bias.
        score = cost / (1 + 0.35 * (len(edges) - 1))
        options.append((score, cost, cell, path))
    if options:
        _, _, target, path = min(options, key=lambda item: (item[0], item[1], item[2]))
        return path, list(target), len(relevant)
    return [], None, len(relevant)


def linear_time(distance, entry, exit_speed, cap, acceleration):
    """Exact triangular/trapezoidal segment with fixed end velocities."""
    if acceleration <= 0 or distance <= 0 or cap <= 0 or entry < 0 or exit_speed < 0:
        raise ValueError("Invalid kinematic segment")
    if abs(entry**2 - exit_speed**2) > 2 * acceleration * distance + 1e-6:
        raise ValueError("Insufficient distance for the requested speed transition")
    cap = max(cap, entry, exit_speed)
    peak = min(cap, math.sqrt(max(0, acceleration * distance + (entry**2 + exit_speed**2) / 2)))
    accel_dist = max(0, (peak**2 - entry**2) / (2 * acceleration))
    decel_dist = max(0, (peak**2 - exit_speed**2) / (2 * acceleration))
    cruise_dist = max(0, distance - accel_dist - decel_dist)
    return ((peak - entry + peak - exit_speed) / acceleration + cruise_dist / peak)


def apply_timing(events, profile):
    """Time each cell edge; carry speed across contiguous straight cells.

    Only already-known straight sections have the higher cap. Unknown sections
    retain search speed. A forward/backward velocity pass respects acceleration
    and braking. This ideal model excludes ADC/control/FRAM latency and slip.
    """
    base, cap, cell = (profile[k] for k in ("search_speed_mm_s", "known_speed_mm_s", "cell_mm"))
    accel, dash = (profile[k] for k in ("acceleration_mm_s2", "dash_acceleration_mm_s2"))
    events[0]["t"] = 0.0
    index, elapsed = 1, 0.0
    while index < len(events):
        event = events[index]
        rel = event["turn"]
        if rel in (1, 3, 2):
            duration = (profile["turn90_s"] if rel in (1, 3) else
                        profile["uturn_s"] + 2 * linear_time(cell / 2, base, 0, base, accel))
            event.update(dt=duration, t=elapsed + duration, speed_limit=base)
            elapsed += duration
            index += 1
            continue
        end = index
        while end < len(events) and events[end]["turn"] == 0:
            end += 1
        batch = events[index:end]
        limits = [cap if e["known_straight"] else base for e in batch]
        accelerations = [dash if e["known_straight"] else accel for e in batch]
        velocities = [0.0 if index == 1 else base]
        for i in range(len(batch)):
            next_cap = limits[i + 1] if i + 1 < len(batch) else base
            velocities.append(min(limits[i], next_cap, math.sqrt(velocities[-1]**2 + 2 * accelerations[i] * cell)))
        if end == len(events):
            velocities[-1] = 0.0
        for i in reversed(range(len(batch))):
            velocities[i] = min(velocities[i], math.sqrt(velocities[i + 1]**2 + 2 * accelerations[i] * cell))
        for i, e in enumerate(batch):
            duration = linear_time(cell, velocities[i], velocities[i + 1], limits[i], accelerations[i])
            elapsed += duration
            e.update(t=elapsed, dt=duration, speed_limit=limits[i])
        index = end
    # Turn-ending sessions still need their final stop; deliberately explicit.
    if len(events) > 1 and events[-1]["turn"] != 0:
        stop = base / accel
        events[-1]["dt"] += stop
        events[-1]["t"] += stop
    delay = profile.get("sensor_delay_s", 0.0)
    for i, event in enumerate(events):
        event["t"] += i * delay
        if i:
            event["dt"] += delay
    return events[-1]["t"]


def finite_cost(solution):
    if solution is None:
        return None
    status = solution.get("status")
    if status not in ("ok", "OK", 0):
        if str(status).replace("-", "_") not in ("no_path", "no_feasible_terminal"):
            raise ValueError(f"Time planner: {status}: {solution.get('error', 'unsupported configuration')}")
        return None
    cost = solution.get("goal_entry_s")
    return float(cost) if cost is not None and math.isfinite(cost) else None


def canonical_edge(edge):
    x, y, d = edge[:3]
    if d == 2:
        return (x, y - 1, 0)
    if d == 3:
        return (x - 1, y, 1)
    return (x, y, d)


def known_goal_reachable(knowledge, start, goals):
    seen, pending = {tuple(start)}, [tuple(start)]
    goal_set = {tuple(g) for g in goals}
    while pending:
        x, y = pending.pop()
        if (x, y) in goal_set:
            return True
        for d in range(4):
            cell = (x + DX[d], y + DY[d])
            if knowledge.open(x, y, d) and cell not in seen and knowledge.inside(*cell):
                seen.add(cell)
                pending.append(cell)
    return False


def simulate(maze, profile, oracle, algorithm="relevant", epsilon=0.0,
             return_home=False, max_steps=12000, progress: Callable | None = None):
    knowledge = Knowledge(maze.width, maze.height)
    pos, heading = tuple(maze.start), 0
    phase = "goal"
    reached_goal, certificate, completion_reason = False, False, "step_limit"
    events = []
    lower_solution = upper_solution = None
    lower = upper = None
    upper_checked_count = -1000
    required_set = set()
    pending_replan = False
    goal_step = certificate_step = home_step = None
    target, relevant_count = None, 0
    incoming = {"turn": 0, "known_straight": False}
    for step in range(max_steps + 1):
        changes = knowledge.observe(maze.walls, pos, heading)
        if pos in maze.goals and not reached_goal:
            reached_goal, goal_step = True, step
        # A shortest optimistic route remains optimal when walls outside its
        # sufficient required-open set appear: its cost is unchanged while
        # alternatives can only disappear. This avoids thousands of replans.
        invalidated = any(wall and canonical_edge((x, y, d)) in required_set
                          for x, y, d, wall in changes)
        pending_replan = pending_replan or invalidated
        # Both policies use the same Adachi goal phase. Its decisions need no
        # time-route updates, so retain the (still admissible) old lower bound
        # until first goal, avoiding expensive unused open-maze solves.
        if lower_solution is None or (pending_replan and reached_goal):
            lower_solution = oracle.solve(knowledge.map_for(True), maze.goals)
            lower = finite_cost(lower_solution)
            required_set = {canonical_edge(e) for e in lower_solution.get("required_edges", [])}
            pending_replan = False
        all_required_known = (not pending_replan and lower is not None and
                              all(not knowledge.edge_unknown(e) for e in required_set))
        if all_required_known:
            # The adapter verifies this set suffices for the complete route,
            # including its stopping tail; therefore it is feasible on K.
            upper_solution, upper = lower_solution, lower
        elif knowledge.count - upper_checked_count >= 32 and known_goal_reachable(knowledge, maze.start, maze.goals):
            candidate = oracle.solve(knowledge.map_for(False), maze.goals)
            candidate_cost = finite_cost(candidate)
            if candidate_cost is not None and (upper is None or candidate_cost < upper):
                upper_solution, upper = candidate, candidate_cost
            upper_checked_count = knowledge.count
        if lower is not None and upper is not None and upper + 1e-5 < lower:
            raise RuntimeError("Route oracle violated optimistic/conservative monotonicity")
        certified_now = (lower is not None and upper is not None and upper <= lower * (1 + epsilon) + 1e-6)
        if certified_now and not certificate:
            certificate, certificate_step = True, step
        event = dict(step=step, x=pos[0], y=pos[1], heading=heading, changes=changes,
                     known_edges=knowledge.count, visited_cells=sum(sum(r) for r in knowledge.visited),
                     lower_s=lower, upper_s=upper, certified=certified_now,
                     phase=phase, target=target, relevant_edges=relevant_count,
                     dt=0.0, t=0.0, **incoming)
        events.append(event)
        if progress and (step % 10 == 0 or certified_now):
            progress({"algorithm": algorithm, "step": step, "known_edges": knowledge.count,
                      "lower_s": lower, "upper_s": upper})
        if algorithm == "relevant" and reached_goal and certified_now and phase != "home":
            phase = "home" if return_home else "done"
            completion_reason = "route_certified"
        if phase == "home" and pos == tuple(maze.start):
            phase, home_step = "done", step
        if phase == "done":
            event["phase"] = "done"
            break
        if step == max_steps:
            completion_reason = "step_limit"
            break
        decision = None
        if phase == "home":
            dist, parents = navigation(knowledge, pos, heading, profile)
            _, path = route_to_cell(dist, parents, maze.start)
            if path:
                decision = {"direction": path[0][2], "known_straight": len(path) > 1 and path[0][2] == heading == path[1][2]}
                target = list(maze.start)
        elif algorithm == "baseline" or (algorithm == "relevant" and not reached_goal):
            decision = next_baseline(knowledge.known, knowledge.walls, knowledge.visited,
                                     maze.goals, pos, heading, phase)
            phase = decision["phase"]
            if phase == "done":
                completion_reason = "all_reachable_visited"
                if return_home and pos != tuple(maze.start):
                    phase = "home"
                    dist, parents = navigation(knowledge, pos, heading, profile)
                    _, path = route_to_cell(dist, parents, maze.start)
                    decision = {"direction": path[0][2], "known_straight": False} if path else None
                else:
                    event["phase"] = "done"
                    break
        else:
            # First establish a complete start-goal route by exploring the
            # optimistic minimum-time route, then resolve competing shortcuts.
            required = lower_solution.get("required_edges", []) if lower_solution else []
            path, target, relevant_count = choose_probe(knowledge, required, pos, heading, profile)
            phase = "verify" if reached_goal else "goal"
            if certified_now and not reached_goal:
                dist, parents = navigation(knowledge, pos, heading, profile)
                options = [(cost, path) for goal in maze.goals for cost, path in [route_to_cell(dist, parents, goal)] if path]
                if options:
                    _, path = min(options, key=lambda item: item[0])
                    target = list(path[-1][:2])
            if path:
                nx, ny, d = path[0]
                eligible = (d == heading and knowledge.visited[ny][nx] and len(path) > 1 and path[1][2] == d)
                decision = {"direction": d, "known_straight": eligible}
            else:
                # An oracle with no feasible route (e.g. restricted primitives)
                # does not grant a certificate. Finish discovery so the result
                # explains infeasibility without an endless frontier loop.
                fallback = next_baseline(knowledge.known, knowledge.walls, knowledge.visited,
                                         maze.goals, pos, heading, "full")
                decision = fallback
                if fallback["phase"] == "done":
                    completion_reason = "no_feasible_route" if upper is None else "full_map_without_certificate"
                    if return_home and pos != tuple(maze.start):
                        phase = "home"
                        dist, parents = navigation(knowledge, pos, heading, profile)
                        _, home_path = route_to_cell(dist, parents, maze.start)
                        decision = {"direction": home_path[0][2], "known_straight": False} if home_path else None
                    else:
                        event["phase"] = "done"
                        break
        event["phase"], event["target"] = phase, target
        if not decision or decision.get("direction") is None:
            completion_reason = "no_reachable_target"
            break
        d = decision["direction"]
        x, y = pos
        if not knowledge.open(x, y, d):
            raise RuntimeError("Policy attempted an unobserved or blocked edge")
        if maze.walls[y][x] & (1 << d):
            raise RuntimeError("Collision with a ground-truth wall")
        nx, ny = x + DX[d], y + DY[d]
        incoming = {"turn": (d - heading) % 4, "known_straight": bool(decision.get("known_straight"))}
        pos, heading = (nx, ny), d
    duration = apply_timing(events, profile)
    def time_at(index):
        return events[index]["t"] if index is not None else None
    return {"algorithm": algorithm, "events": events,
            "summary": {"duration_s": duration, "steps": len(events) - 1,
                        "distance_m": (len(events) - 1) * profile["cell_mm"] / 1000,
                        "turns90": sum(e["turn"] in (1, 3) for e in events[1:]),
                        "uturns": sum(e["turn"] == 2 for e in events[1:]),
                        "empty_transit_steps": sum(not e["changes"] for e in events[1:]),
                        "known_edges": knowledge.count, "visited_cells": events[-1]["visited_cells"],
                        "first_goal_s": time_at(goal_step), "certificate_s": time_at(certificate_step),
                        "home_s": time_at(home_step), "certified": events[-1]["certified"],
                        "lower_s": lower, "upper_s": upper, "reason": completion_reason,
                        "completed": completion_reason not in ("step_limit", "no_reachable_target")},
            "final_known": knowledge.known, "final_walls": knowledge.walls,
            "optimistic_route": lower_solution.get("route_points", []) if lower_solution else [],
            "known_route": upper_solution.get("route_points", []) if upper_solution else []}
