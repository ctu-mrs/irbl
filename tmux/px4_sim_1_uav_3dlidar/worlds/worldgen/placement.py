"""Random placement with keep-out zones and minimum spacing (rejection sampling)."""
import math


class Placer:
    """Tracks occupied discs and keep-out regions, so independent scatter passes (trees, then bushes,
    then rocks, ...) never overlap each other, the UAV spawn point, or the goal."""

    def __init__(self, rng, keepouts=()):
        self.rng = rng
        self.circles = []  # (x, y, r)
        self.rects = []    # (x0, x1, y0, y1)
        for k in keepouts:
            self.keep_out(*k)

    def keep_out(self, *k):
        """keep_out(x, y, r) for a disc, keep_out(x0, x1, y0, y1) for an axis-aligned rectangle."""
        (self.circles if len(k) == 3 else self.rects).append(tuple(k))

    def free(self, x, y, r):
        if any(x0 - r < x < x1 + r and y0 - r < y < y1 + r for x0, x1, y0, y1 in self.rects):
            return False
        return all((x - cx) ** 2 + (y - cy) ** 2 >= (r + cr) ** 2 for cx, cy, cr in self.circles)

    def claim(self, x, y, r):
        self.circles.append((x, y, r))

    def try_place(self, x, y, r):
        if self.free(x, y, r):
            self.claim(x, y, r)
            return True
        return False

    def scatter(self, n, x_range, y_range, radius, tries=60):
        """Yield up to n positions (x, y, r); `radius` is a number or a callable rng -> number."""
        for _ in range(n):
            for _t in range(tries):
                r = radius(self.rng) if callable(radius) else radius
                x = self.rng.uniform(*x_range)
                y = self.rng.uniform(*y_range)
                if self.try_place(x, y, r):
                    yield x, y, r
                    break


def area(x_range, y_range):
    return (x_range[1] - x_range[0]) * (y_range[1] - y_range[0])


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])
