#!/usr/bin/env python3
"""Infinite goal loop + live stats dashboard for the px4_sim_10_uav session.

Waits until every UAV publishes odometry and hovers (z > --takeoff-z), sends everyone a random goal
and activates the controllers. As soon as ALL UAVs have reached their goal (or --timeout expires) a
new set of goals is drawn (each one at least MIN_TRAVEL from that UAV's *current* position) and sent
to everybody at the same time. Forever, until Ctrl+C.

The top of the terminal shows the state of the simulation (path length, collisions, min inter-agent
distance, velocities, round times, ...). Every finished round is also appended to
logs/infinite_<date>.csv.

  ./infinite.py [--n 10] [--tol 0.5] [--timeout 180] [--collision-dist 0.6] [--safety-dist 1.0]
"""
import argparse
import csv
import itertools
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rbl_msgs.srv import Vec4
from std_srvs.srv import Trigger

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_positions  # noqa: E402  (reuses its sampler and MIN_SEP / MIN_TRAVEL / GOAL_Z)

MIN_STEP = 0.02  # [m] odometry jitter below this is not counted as travelled distance


class Agent:
    def __init__(self):
        self.pos = None
        self.speed = 0.0
        self.length = 0.0          # total, all rounds
        self.round_length = 0.0
        self.last_pos = None
        self.goal = None
        self.arrived = False
        self.max_speed = 0.0
        self.collisions = 0


class Infinite(Node):
    def __init__(self, args):
        super().__init__("infinite_goals")
        self.a = args
        self.n = args.n
        self.ag = [Agent() for _ in range(self.n)]
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for i in range(self.n):
            self.create_subscription(Odometry, f"/uav{i+1}/odometry",
                                     lambda m, i=i: self.odom_cb(i, m), qos)
        self.goto = [self.create_client(Vec4, f"/uav{i+1}/rbl_controller/goto") for i in range(self.n)]
        self.act = [self.create_client(Trigger, f"/uav{i+1}/rbl_controller/activation") for i in range(self.n)]

        self.phase = "waiting for odometry"
        self.t0 = None              # start of the first round
        self.round = 0              # rounds started
        self.round_start = None
        self.round_times = []       # completed rounds only
        self.timeouts = 0
        self.arrive_time = {}       # i -> time of arrival in current round
        self.in_collision = set()   # pairs currently closer than collision_dist
        self.in_violation = set()   # pairs currently closer than safety_dist
        self.collisions = 0
        self.violations = 0
        self.min_dist_now = float("nan")
        self.min_dist_all = float("inf")
        self.min_dist_round = float("inf")
        self.min_pair_all = None
        self.speed_sum = 0.0        # time integral of mean speed, for the average
        self.speed_t = 0.0
        self.events = []
        self.pending = []           # service futures in flight

        os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
        self.csv_path = os.path.join(HERE, "logs", f"infinite_{datetime.now():%Y%m%d_%H%M%S}.csv")
        with open(self.csv_path, "w", newline="") as f:
            csv.writer(f).writerow(["round", "duration_s", "timeout", "round_length_m", "total_length_m",
                                    "min_dist_round_m", "collisions_total", "violations_total",
                                    "mean_speed_round_mps"])

        self.last_tick = time.monotonic()
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0 / args.rate, self.draw)

    # ------------------------------------------------------------------ callbacks
    def odom_cb(self, i, m):
        p = m.pose.pose.position
        v = m.twist.twist.linear
        a = self.ag[i]
        a.pos = (p.x, p.y, p.z)
        a.speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        if self.t0 is not None:  # only count once the experiment is running
            a.max_speed = max(a.max_speed, a.speed)
            if a.last_pos is None:
                a.last_pos = a.pos
            d = math.dist(a.pos, a.last_pos)
            if d >= MIN_STEP:
                a.length += d
                a.round_length += d
                a.last_pos = a.pos

    def log(self, msg):
        self.events.append(f"{datetime.now():%H:%M:%S} {msg}")
        self.events = self.events[-self.a.events:]

    # ------------------------------------------------------------------ logic
    def tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        if any(a.pos is None for a in self.ag):
            missing = [f"uav{i+1}" for i, a in enumerate(self.ag) if a.pos is None]
            self.phase = "waiting for odometry: " + " ".join(missing)
            return

        self.update_distances()

        if self.t0 is None:
            low = [f"uav{i+1}" for i, a in enumerate(self.ag) if a.pos[2] < self.a.takeoff_z]
            if low:
                self.phase = f"waiting for takeoff (z > {self.a.takeoff_z} m): " + " ".join(low)
                return
            if not all(c.service_is_ready() for c in self.goto + self.act):
                self.phase = "waiting for rbl_controller services"
                return
            self.t0 = now
            for a in self.ag:
                a.last_pos = a.pos
            self.new_round(now, activate=True)
            return

        self.speed_sum += dt * sum(a.speed for a in self.ag) / self.n
        self.speed_t += dt

        for i, a in enumerate(self.ag):
            if not a.arrived and math.dist(a.pos, a.goal) < self.a.tol:
                a.arrived = True
                self.arrive_time[i] = now - self.round_start

        elapsed = now - self.round_start
        if all(a.arrived for a in self.ag):
            self.round_times.append(elapsed)
            self.log(f"round {self.round} done in {elapsed:.1f} s")
            self.end_round(elapsed, timeout=False)
            self.new_round(now)
        elif self.a.timeout > 0 and elapsed > self.a.timeout:
            late = [f"uav{i+1}" for i, a in enumerate(self.ag) if not a.arrived]
            self.timeouts += 1
            self.log(f"round {self.round} TIMEOUT after {elapsed:.0f} s, not arrived: {' '.join(late)}")
            self.end_round(elapsed, timeout=True)
            self.new_round(now)

    def update_distances(self):
        dmin = float("inf")
        pmin = None
        for i, j in itertools.combinations(range(self.n), 2):
            d = math.dist(self.ag[i].pos, self.ag[j].pos)
            if d < dmin:
                dmin, pmin = d, (i, j)
            if self.t0 is None:
                continue
            pair = (i, j)
            # rising edge = one event; hysteresis of 10 % to avoid counting jitter as many events
            if d < self.a.collision_dist and pair not in self.in_collision:
                self.in_collision.add(pair)
                self.collisions += 1
                self.ag[i].collisions += 1
                self.ag[j].collisions += 1
                self.log(f"COLLISION uav{i+1} <-> uav{j+1}  d={d:.2f} m")
            elif d > 1.1 * self.a.collision_dist:
                self.in_collision.discard(pair)
            if d < self.a.safety_dist and pair not in self.in_violation:
                self.in_violation.add(pair)
                self.violations += 1
            elif d > 1.1 * self.a.safety_dist:
                self.in_violation.discard(pair)
        self.min_dist_now = dmin
        if self.t0 is not None:
            self.min_dist_round = min(self.min_dist_round, dmin)
            if dmin < self.min_dist_all:
                self.min_dist_all, self.min_pair_all = dmin, pmin

    def end_round(self, elapsed, timeout):
        rl = sum(a.round_length for a in self.ag)
        with open(self.csv_path, "a", newline="") as f:
            csv.writer(f).writerow([self.round, f"{elapsed:.2f}", int(timeout), f"{rl:.2f}",
                                    f"{self.total_length():.2f}", f"{self.min_dist_round:.3f}",
                                    self.collisions, self.violations,
                                    f"{rl / self.n / elapsed if elapsed > 0 else 0:.3f}"])

    def new_round(self, now, activate=False):
        current = [a.pos[:2] for a in self.ag]
        pts = gen_positions.sample(avoid=current)
        with open(os.path.join(HERE, "config", "goals.env"), "w") as f:
            f.write("\n".join(f"export UAV{i}_GOAL_X={x:.2f}; export UAV{i}_GOAL_Y={y:.2f}; "
                              f"export UAV{i}_GOAL_Z={gen_positions.GOAL_Z}; export UAV{i}_GOAL_YAW=0.0"
                              for i, (x, y) in enumerate(pts, 1)) + "\n")
        self.round += 1
        self.round_start = now
        self.arrive_time = {}
        self.min_dist_round = float("inf")
        for i, (a, (x, y)) in enumerate(zip(self.ag, pts)):
            a.goal = (x, y, gen_positions.GOAL_Z)
            a.arrived = False
            a.round_length = 0.0
            req = Vec4.Request()
            req.goal = [float(x), float(y), float(gen_positions.GOAL_Z), 0.0]
            self.pending.append(self.goto[i].call_async(req))
        if activate:
            for c in self.act:
                self.pending.append(c.call_async(Trigger.Request()))
            self.log(f"controllers activated, round {self.round} started")
        else:
            self.log(f"round {self.round} started, new goals sent")
        self.pending = [p for p in self.pending if not p.done()]
        self.phase = "running"

    def total_length(self):
        return sum(a.length for a in self.ag)

    # ------------------------------------------------------------------ display
    def draw(self):
        B, R, G, Y, C, D = "\033[1m", "\033[0m", "\033[32m", "\033[33m", "\033[36m", "\033[2m"
        RED = "\033[31m"
        now = time.monotonic()
        out = []
        out.append(f"{B}{C}=== RBL infinite goals - {self.n} UAVs ==={R}   {D}{datetime.now():%H:%M:%S}  "
                   f"csv: {os.path.relpath(self.csv_path, HERE)}{R}")
        if self.t0 is None:
            out.append(f"{Y}{self.phase}{R}")
            if not math.isnan(self.min_dist_now):
                out.append(f"min distance now: {self.min_dist_now:.2f} m")
        else:
            T = now - self.t0
            el = now - self.round_start
            done = sum(a.arrived for a in self.ag)
            mean_rt = sum(self.round_times) / len(self.round_times) if self.round_times else float("nan")
            last_rt = self.round_times[-1] if self.round_times else float("nan")
            avg_speed = self.speed_sum / self.speed_t if self.speed_t > 0 else 0.0
            cur_speed = sum(a.speed for a in self.ag) / self.n
            max_speed = max(a.max_speed for a in self.ag)
            tl = self.total_length()
            col = RED if self.collisions else G
            mdc = RED if self.min_dist_now < self.a.collision_dist else (
                Y if self.min_dist_now < self.a.safety_dist else G)
            pair = f"(uav{self.min_pair_all[0]+1}-uav{self.min_pair_all[1]+1})" if self.min_pair_all else ""
            out.append(f"elapsed {B}{fmt_t(T)}{R}   round {B}{self.round}{R} ({fmt_t(el)})   "
                       f"arrived {B}{done}/{self.n}{R}   completed {B}{len(self.round_times)}{R}   "
                       f"timeouts {B}{self.timeouts}{R}")
            out.append(f"round time   last {last_rt:6.1f} s   mean {mean_rt:6.1f} s   "
                       f"best {min(self.round_times) if self.round_times else float('nan'):6.1f} s")
            out.append(f"length       total {B}{tl:8.1f} m{R}   per UAV {tl/self.n:7.1f} m   "
                       f"this round {sum(a.round_length for a in self.ag):7.1f} m")
            out.append(f"velocity     now {cur_speed:5.2f} m/s   avg {B}{avg_speed:5.2f} m/s{R}   "
                       f"max {max_speed:5.2f} m/s")
            out.append(f"min distance now {mdc}{self.min_dist_now:5.2f} m{R}   round "
                       f"{self.min_dist_round:5.2f} m   all-time {B}{self.min_dist_all:5.2f} m{R} {pair}")
            out.append(f"collisions   {col}{B}{self.collisions}{R} (d < {self.a.collision_dist} m)   "
                       f"safety violations {self.violations} (d < {self.a.safety_dist} m)")
            out.append("")
            out.append(f"{D} uav   x      y      z     goal dist  speed  length  coll  state{R}")
            for i, a in enumerate(self.ag):
                gd = math.dist(a.pos, a.goal)
                st = f"{G}arrived {self.arrive_time.get(i, 0):5.1f}s{R}" if a.arrived else f"{Y}moving{R}"
                out.append(f" {i+1:>3} {a.pos[0]:6.1f} {a.pos[1]:6.1f} {a.pos[2]:5.1f} {gd:9.2f}  "
                           f"{a.speed:5.2f} {a.length:7.1f} {a.collisions:5d}  {st}")
        out.append("")
        out.append(f"{D}--- events ---{R}")
        out.extend(self.events)
        sys.stdout.write("\033[H\033[2J" + "\n".join(out) + "\n")
        sys.stdout.flush()


def fmt_t(s):
    s = int(s)
    return f"{s // 3600:d}:{s // 60 % 60:02d}:{s % 60:02d}"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--n", type=int, default=gen_positions.N)
    p.add_argument("--tol", type=float, default=0.5, help="[m] goal reached when closer than this")
    p.add_argument("--timeout", type=float, default=180.0, help="[s] max round time, 0 = never")
    p.add_argument("--collision-dist", type=float, default=0.6, help="[m] centre distance counted as collision")
    p.add_argument("--safety-dist", type=float, default=1.0, help="[m] 2*encumbrance, safety violation")
    p.add_argument("--takeoff-z", type=float, default=1.5, help="[m] all UAVs above this before starting")
    p.add_argument("--rate", type=float, default=2.0, help="[Hz] dashboard refresh")
    p.add_argument("--events", type=int, default=8, help="number of event lines shown")
    args = p.parse_args()
    rclpy.init()
    node = Infinite(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\nstopped, round log: {node.csv_path}")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
