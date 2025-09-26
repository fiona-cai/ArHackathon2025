"""
Amazon Robotics Hackathon - Routing API

This module defines the routing API for the Amazon Robotics Hackathon.
Students will implement the route_package function in this module.

*****IMPORTANT*****
Team name: Kaibo and Friends
Email address: fiona.cai899@gmail.com
*******************
"""

from typing import Optional, Dict, List, Tuple
from ar_hackathon.models.game_state import GameState
from ar_hackathon.models.package import Package


def _safe_int(x, default=0):
    try:
        return int(x)
    except Exception:
        return default


class SimpleHeap:
    """Ultra-simple heap for Dijkstra."""

    def __init__(self):
        self._data = []

    def push(self, cost, node):
        self._data.append((cost, node))
        i = len(self._data) - 1
        while i > 0:
            p = (i - 1) >> 1
            if self._data[p][0] <= self._data[i][0]:
                break
            self._data[p], self._data[i] = self._data[i], self._data[p]
            i = p

    def pop(self):
        if not self._data:
            return None
        if len(self._data) == 1:
            return self._data.pop()

        result = self._data[0]
        self._data[0] = self._data.pop()
        i = 0
        n = len(self._data)
        while True:
            l = 2 * i + 1
            r = l + 1
            smallest = i
            if l < n and self._data[l][0] < self._data[smallest][0]:
                smallest = l
            if r < n and self._data[r][0] < self._data[smallest][0]:
                smallest = r
            if smallest == i:
                break
            self._data[i], self._data[smallest] = self._data[smallest], self._data[i]
            i = smallest
        return result

    def __bool__(self):
        return bool(self._data)


class FastRouter:
    """Simple, fast router that handles both bandwidth and no-bandwidth cases efficiently."""

    def __init__(self):
        self.path_cache = {}
        self.last_topology_hash = None

    def _get_topology_hash(self, connections):
        """Simple hash for topology change detection."""
        h = 0
        for conn in connections:
            h ^= hash((conn.from_fc, conn.to_fc, int(conn.weight * 1000)))
        return h

    def _check_bandwidth_available(self, conn):
        """Quick bandwidth check."""
        if conn.available_bandwidth is None:
            return True  # Unlimited bandwidth
        return conn.available_bandwidth > 0

    def _simple_dijkstra(self, state, start, end):
        """Fast Dijkstra optimized for both bandwidth and no-bandwidth cases."""
        if start == end:
            return end

        # Build simple adjacency list
        graph = {}
        has_bandwidth_limits = False

        for conn in state.connections:
            if conn.from_fc not in graph:
                graph[conn.from_fc] = []

            # Check if this connection is available
            if conn.available_bandwidth is not None:
                has_bandwidth_limits = True
                if conn.available_bandwidth <= 0:
                    continue  # Skip unavailable connections

            graph[conn.from_fc].append((conn.to_fc, conn.weight))

        # Cache key includes bandwidth state only if there are limits
        if has_bandwidth_limits:
            bandwidth_state = tuple(sorted(
                (conn.from_fc, conn.to_fc, conn.available_bandwidth)
                for conn in state.connections
                if conn.available_bandwidth is not None
            ))
            cache_key = (start, end, bandwidth_state)
        else:
            cache_key = (start, end, 'unlimited')

        # Check cache
        if cache_key in self.path_cache:
            return self.path_cache[cache_key]

        # Dijkstra's algorithm
        distances = {start: 0.0}
        previous = {}
        heap = SimpleHeap()
        heap.push(0.0, start)
        visited = set()

        while heap:
            item = heap.pop()
            if not item:
                break

            dist, node = item

            if node in visited:
                continue
            visited.add(node)

            if node == end:
                # Reconstruct first hop
                current = end
                while current in previous and previous[current] != start:
                    current = previous[current]

                result = current if current != start else end
                self.path_cache[cache_key] = result
                return result

            for neighbor, weight in graph.get(node, []):
                if neighbor in visited:
                    continue

                new_dist = dist + weight
                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = node
                    heap.push(new_dist, neighbor)

        # No path found
        self.path_cache[cache_key] = None
        return None

    def route_package_fast(self, state, package):
        """Ultra-fast routing for both bandwidth and no-bandwidth scenarios."""
        if package.current_fc == package.destination_fc:
            return None

        # Clear cache if topology changed
        current_hash = self._get_topology_hash(state.connections)
        if self.last_topology_hash != current_hash:
            self.path_cache.clear()
            self.last_topology_hash = current_hash

        # Find next hop
        next_fc = self._simple_dijkstra(state, package.current_fc, package.destination_fc)

        if not next_fc or next_fc == package.current_fc:
            return None

        # Final validation
        conn = state.get_connection(package.current_fc, next_fc)
        if not conn or not self._check_bandwidth_available(conn):
            # Remove from cache as it's no longer valid
            for key in list(self.path_cache.keys()):
                if key[0] == package.current_fc:
                    del self.path_cache[key]
            return None

        return next_fc


class LoadBalancer:
    """Simple load balancer for when bandwidth limits exist."""

    def __init__(self):
        self.usage_counts = {}

    def select_best_option(self, state, from_fc, candidates):
        """Select best connection considering load."""
        if not candidates:
            return None

        if len(candidates) == 1:
            return candidates[0]

        # Score each candidate
        best_candidate = None
        best_score = float('inf')

        for next_fc in candidates:
            conn = state.get_connection(from_fc, next_fc)
            if not conn:
                continue

            # Base score is connection weight
            score = conn.weight

            # Add usage penalty if bandwidth is limited
            if conn.available_bandwidth is not None:
                usage_key = (from_fc, next_fc)
                usage_count = self.usage_counts.get(usage_key, 0)
                score += usage_count * 0.1

                # Heavy penalty for low bandwidth
                if conn.available_bandwidth <= 1:
                    score += 5.0

            if score < best_score:
                best_score = score
                best_candidate = next_fc

        return best_candidate

    def record_usage(self, from_fc, to_fc):
        """Record connection usage."""
        usage_key = (from_fc, to_fc)
        self.usage_counts[usage_key] = self.usage_counts.get(usage_key, 0) + 1

        # Prevent overflow
        if self.usage_counts[usage_key] > 100:
            # Decay all counts
            for key in self.usage_counts:
                self.usage_counts[key] = max(0, self.usage_counts[key] - 50)


# Global instances
_router = FastRouter()
_load_balancer = LoadBalancer()


def route_package(state: GameState, package: Package) -> Optional[str]:
    """
    Fast routing optimized for both bandwidth-limited and unlimited scenarios.

    - When bandwidth is None: Uses simple shortest path with aggressive caching
    - When bandwidth has limits: Considers availability and load balancing
    """
    # Fast exits
    if package.current_fc == package.destination_fc:
        return None

    current_time = _safe_int(state.current_time_step, 0)
    if package.in_transit:
        return None

    entry_time = _safe_int(package.entry_time, current_time)
    if entry_time > current_time:
        return None

    # Route the package
    next_fc = _router.route_package_fast(state, package)

    if not next_fc:
        return None

    # Verify and update connection
    conn = state.get_connection(package.current_fc, next_fc)
    if not conn:
        return None

    # Check bandwidth availability one more time
    if conn.available_bandwidth is not None and conn.available_bandwidth <= 0:
        return None

    # Update package state
    travel_time = max(1, int(conn.weight))
    package.in_transit = True
    package.transit_destination = next_fc
    package.transit_remaining_time = travel_time

    # Update bandwidth if limited
    if conn.available_bandwidth is not None:
        conn.available_bandwidth = max(0, conn.available_bandwidth - 1)
        _load_balancer.record_usage(package.current_fc, next_fc)

    return next_fc


def route_batch_fast(state: GameState) -> Dict[str, str]:
    """
    Fast batch routing that handles bandwidth efficiently.
    """
    routing_decisions = {}

    # Get all ready packages
    ready_packages = []
    current_time = _safe_int(state.current_time_step, 0)

    for pkg in state.active_packages:
        if (not pkg.in_transit and
                pkg.current_fc != pkg.destination_fc and
                _safe_int(pkg.entry_time, current_time) <= current_time):
            ready_packages.append(pkg)

    # Sort by entry time (older first)
    ready_packages.sort(key=lambda p: p.entry_time)

    # Check if we have bandwidth limits
    has_bandwidth_limits = any(
        conn.available_bandwidth is not None
        for conn in state.connections
    )

    if not has_bandwidth_limits:
        # No bandwidth limits - route all packages independently
        for pkg in ready_packages:
            next_fc = _router.route_package_fast(state, pkg)
            if next_fc:
                routing_decisions[pkg.id] = next_fc
    else:
        # Has bandwidth limits - coordinate routing
        for pkg in ready_packages:
            next_fc = _router.route_package_fast(state, pkg)

            if next_fc:
                conn = state.get_connection(pkg.current_fc, next_fc)
                if conn and (conn.available_bandwidth is None or conn.available_bandwidth > 0):
                    routing_decisions[pkg.id] = next_fc

                    # Update bandwidth for subsequent packages
                    if conn.available_bandwidth is not None:
                        conn.available_bandwidth = max(0, conn.available_bandwidth - 1)

    return routing_decisions


def get_simple_preview(state: GameState, package: Package) -> Optional[str]:
    """Get next hop preview without state changes."""
    return _router._simple_dijkstra(state, package.current_fc, package.destination_fc)


def clear_caches():
    """Clear all caches."""
    _router.path_cache.clear()
    _router.last_topology_hash = None
    _load_balancer.usage_counts.clear()


def get_performance_stats() -> Dict:
    """Get performance statistics."""
    return {
        'path_cache_size': len(_router.path_cache),
        'usage_tracking': len(_load_balancer.usage_counts),
        'topology_cached': _router.last_topology_hash is not None
    }