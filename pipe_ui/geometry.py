# Migrated from hvwire/pipe-editor管路编辑器是这个！/route_editor/geometry.py.
# Source kept outside Git; this copy uses the air-conditioning settings and fixed GUADIAN bridges.
"""Standalone editor runtime; maintained independently of automatic routing."""
from __future__ import annotations
"""Local CAD-compatible editing: exact source curves and adaptive tangent biarcs.

Only the selected interval is replaced. No global control-point search is used.
All lengths are mm; source splines retain their existing sampled representation.
"""


import copy


import math
import time


import numpy as np

import settings as config


def unit(v):
    length = float(np.linalg.norm(v))
    if length < 1e-10:
        raise ValueError("局部重建遇到退化切向，请更换选点或扩大影响范围")
    return np.asarray(v, dtype=float) / length


class Piece:
    def __init__(self, segment):
        self.segment = segment
        self.kind = segment["type"]
        self.start = np.asarray(segment["start"], dtype=float)
        self.end = np.asarray(segment["end"], dtype=float)
        if self.kind == "arc":
            self.center = np.asarray(segment["center"], dtype=float)
            self.radius = float(segment["radius"])
            self.angle = math.radians(segment["angle_deg"])
            if not 0 < self.angle < math.pi - 1e-8 or self.radius <= 0:
                raise ValueError("编辑器暂不支持单段达到 180° 的圆弧，请先拆分圆弧")
            self.u = unit(self.start - self.center)
            v = unit(self.end - self.center)
            self.v = unit(v - np.dot(v, self.u) * self.u)
            self.length = self.radius * self.angle
        elif self.kind == "line":
            self.length = float(np.linalg.norm(self.end - self.start))
            self.direction = unit(self.end - self.start)
        elif self.kind == "spline":
            self.points = np.asarray(segment["points"], dtype=float)
            self.marks = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))]
            self.length = float(self.marks[-1])
        else:
            raise ValueError(f"编辑器不支持的线段类型：{self.kind}")
        if self.length <= 1e-8:
            raise ValueError("工程结果包含零长度线段")

    def at(self, distance):
        s = float(np.clip(distance, 0, self.length))
        if self.kind == "arc":
            angle = s / self.radius
            point = self.center + self.radius * (math.cos(angle)*self.u + math.sin(angle)*self.v)
            tangent = -math.sin(angle)*self.u + math.cos(angle)*self.v
            return point, tangent
        if self.kind == "line":
            return self.start + s*self.direction, self.direction
        i = min(len(self.points)-2, max(0, np.searchsorted(self.marks, s, side="right")-1))
        direction = unit(self.points[i+1]-self.points[i])
        return self.points[i] + (s-self.marks[i])*direction, direction

    def clipped(self, low, high):
        if low <= 1e-8 and high >= self.length-1e-8:
            return copy.deepcopy(self.segment)
        result = copy.deepcopy(self.segment)
        result.update(start=self.at(low)[0].tolist(), end=self.at(high)[0].tolist())
        if self.kind == "arc":
            result["angle_deg"] = math.degrees((high-low)/self.radius)
        elif self.kind == "spline":
            marks = [low, *self.marks[(self.marks > low) & (self.marks < high)], high]
            result["points"] = [self.at(s)[0].tolist() for s in marks]
        return result


class RouteCurve:
    def __init__(self, segments, fixed_intervals=()):
        self.fixed_intervals = fixed_intervals
        self.pieces = [Piece(s) for s in segments]
        if not self.pieces:
            raise ValueError("结果缺少工程线段")
        for a, b in zip(self.pieces, self.pieces[1:]):
            if np.linalg.norm(a.end-b.start) > 1e-5:
                raise ValueError("工程线段不连续，请重新生成工程结果")
        self.marks = np.r_[0., np.cumsum([p.length for p in self.pieces])]
        self.length = float(self.marks[-1])

    def at(self, station):
        i = min(len(self.pieces)-1, max(0, np.searchsorted(self.marks, station, side="right")-1))
        return self.pieces[i].at(station-self.marks[i])

    def limits(self):
        # Protected endpoint splines must remain exactly as imported.
        protection = float(config.JOINT_ENDPOINT_TANGENT_LENGTH)
        left, right = protection, self.length-protection
        for i, piece in enumerate(self.pieces):
            if piece.kind != "spline":
                break
            left = max(left, float(self.marks[i+1]))
        for i in range(len(self.pieces)-1, -1, -1):
            if self.pieces[i].kind != "spline":
                break
            right = min(right, float(self.marks[i]))
        return left, right

    def sample(self, step=5.):
        values = [0.]
        for i, piece in enumerate(self.pieces):
            count = max(1, int(math.ceil(piece.length/step)))
            # Keep endpoint attachment edges intact: their exit validation is
            # defined on the whole first/last edge, not a new 5 mm fragment.
            if piece.kind == "line" and i in (0,len(self.pieces)-1):
                count = 1
            if piece.kind == "arc":
                count = max(count, int(math.ceil(math.degrees(piece.angle)/2.)))
            local = np.linspace(0, piece.length, count+1)
            if piece.kind == "spline":
                local = piece.marks
            values.extend((local[1:]+self.marks[i]).tolist())
        return np.array([self.at(s)[0] for s in values]), np.asarray(values)

    def clip(self, low, high):
        segments = []
        for i, piece in enumerate(self.pieces):
            a, b = max(low, self.marks[i]), min(high, self.marks[i+1])
            if b-a > 1e-7:
                segments.append(piece.clipped(a-self.marks[i], b-self.marks[i]))
        return segments

    def display(self):
        path, stations = self.sample()
        return {"path": path.tolist(), "stations": stations.tolist(),
                "editLimits": list(self.limits()),
                "editGeometry": [{"startStation": float(self.marks[i]), "length": p.length,
                                  **p.segment} for i, p in enumerate(self.pieces)]}


def deformation(curve, station, delta, influence):
    delta = np.asarray(delta, dtype=float)
    if delta.shape != (3,) or not np.isfinite(delta).all() or np.linalg.norm(delta) > 500:
        raise ValueError("单次位移须为有限 XYZ 数值，长度不得超过 500 mm")
    if not np.isfinite(station) or not np.isfinite(influence) or not 30 <= influence <= 2000:
        raise ValueError("影响半径须在 30–2000 mm 内")
    left, right = curve.limits()
    if not left+5 < station < right-5:
        raise ValueError("端点和接入切向段固定，请选择更靠近中间的位置")
    low, high = max(left, station-influence), min(right, station+influence)
    # Preserve each existing GUADIAN straight bridge, including its tangent.
    for start, end in curve.fixed_intervals:
        if start - 1e-6 <= station <= end + 1e-6:
            raise ValueError("挂点及其两侧固定直线不可拖动，请选择其它位置")
        if end < station:
            low = max(low, end)
        elif start > station:
            high = min(high, start)
    if station-low <= 5 or high-station <= 5:
        raise ValueError("选点过于靠近固定区域，请选择更远的位置")
    if any(p.kind == "spline" and curve.marks[i] < high and curve.marks[i+1] > low
           for i, p in enumerate(curve.pieces)):
        raise ValueError("修改范围包含无法精确重建的中间样条，请缩小影响范围或选择直线、圆弧段")

    def evaluate(s):
        point, tangent = curve.at(s)
        span = station-low if s <= station else high-station
        t = float(np.clip(abs(s-station)/span, 0., 1.))
        weight = 1 - (10*t**3-15*t**4+6*t**5)
        derivative = -30*t*t*(1-t)**2 * (-1 if s <= station else 1)/span
        return point+weight*delta, unit(tangent+derivative*delta)
    return low, high, evaluate


def tangent_arc(start, tangent, end):
    v = end-start
    normal = v-np.dot(v, tangent)*tangent
    height = float(np.linalg.norm(normal))
    if height < 1e-8:
        if np.dot(v, tangent) <= 0:
            raise ValueError("局部曲线发生折返，请减小位移或扩大影响范围")
        return [{"type": "line", "start": start.tolist(), "end": end.tolist()}]
    normal /= height
    radius = float(np.dot(v, v)/(2*height))
    center = start+radius*normal
    u = -normal
    angle = math.atan2(np.dot(end-center, tangent), np.dot(end-center, u)) % (2*math.pi)
    if angle > math.pi + 1e-7:
        raise ValueError("局部圆弧发生折返，请减小位移或扩大影响范围")
    result = []
    angles = np.linspace(0., angle, max(1, math.ceil(angle/(math.pi/2)))+1)
    for a, b in zip(angles[:-1], angles[1:]):
        p = center+radius*(math.cos(a)*u+math.sin(a)*tangent)
        q = center+radius*(math.cos(b)*u+math.sin(b)*tangent)
        result.append({"type": "arc", "center": center.tolist(), "radius": radius,
                       "angle_deg": math.degrees(b-a), "start": p.tolist(), "end": q.tolist()})
    result[0]["start"], result[-1]["end"] = start.tolist(), end.tolist()
    return result


def biarc(p, t, q, u):
    v = q-p
    a, b, vv = 2*(1-np.dot(t,u)), 2*np.dot(v,t+u), np.dot(v,v)
    if abs(a) < 1e-10:
        if b <= 1e-10:
            raise ValueError("局部切向无法连接，请扩大影响范围")
        d = vv/b
    else:
        root = math.sqrt(max(0., b*b+4*a*vv))
        d = 2*vv/(b+root) if b >= 0 else (-b+root)/(2*a)
    middle = (p+q+d*(t-u))/2
    first = tangent_arc(p, t, middle)
    second = tangent_arc(q, -u, middle)
    for seg in second:
        seg["start"], seg["end"] = seg["end"], seg["start"]
    return first+list(reversed(second))


def point_distance(point, pieces):
    best = float("inf")
    for p in pieces:
        if p.kind == "line":
            s = np.clip(np.dot(point-p.start, p.direction), 0, p.length)
        else:
            v = point-p.center
            s = np.clip(math.atan2(np.dot(v,p.v), np.dot(v,p.u)), 0, p.angle)*p.radius
        best = min(best, float(np.linalg.norm(point-p.at(s)[0])),
                   float(np.linalg.norm(point-p.start)), float(np.linalg.norm(point-p.end)))
    return best


def rebuild_local(segments, station, delta, influence, tolerance=.1, fixed_intervals=()):
    curve = RouteCurve(segments, fixed_intervals)
    low, high, evaluate = deformation(curve, station, delta, influence)
    return fit_deformation(curve, low, high, evaluate, [station], tolerance)


def fit_deformation(curve, low, high, evaluate, targets, tolerance=.1, deadline=None):
    result = []
    max_error = 0.

    def fit(a, b, depth=0):
        nonlocal max_error
        if deadline is not None and time.monotonic() >= deadline:
            raise RuntimeError('局部优化达到时间上限，搜索未完成')
        try:
            candidate = biarc(*evaluate(a), *evaluate(b))
            pieces = [Piece(s) for s in candidate]
            samples = np.array([evaluate(s)[0] for s in np.linspace(a,b,17)])
            error = max(point_distance(p, pieces) for p in samples)
            # Also reject excursions of the fitted arcs away from the preview.
            starts, edges = samples[:-1], np.diff(samples,axis=0)
            lengths = np.einsum("ij,ij->i",edges,edges)
            for piece in pieces:
                for value in np.linspace(0,piece.length,9):
                    point = piece.at(value)[0]
                    t = np.clip(np.einsum("ij,ij->i",point-starts,edges)/np.maximum(lengths,1e-20),0,1)
                    error = max(error,float(np.min(np.linalg.norm(point-starts-t[:,None]*edges,axis=1))))
        except ValueError:
            candidate, error = [], float("inf")
        if error > tolerance or b-a > 40:
            if depth >= 16 or b-a < .05:
                raise ValueError(f"局部拟合未收敛：原路径弧长 {a:.1f}–{b:.1f} mm；请减小位移或扩大影响范围")
            fit(a, (a+b)/2, depth+1)
            fit((a+b)/2, b, depth+1)
        else:
            result.extend(candidate)
            max_error = max(max_error, error)

    # Prior local edits can contain many short arcs. Reusing every old arc end
    # as a mandatory split would double their count on each subsequent edit.
    # Retain spaced source boundaries; the error test refines between them.
    marks = [low]
    for value in curve.marks[(curve.marks > low) & (curve.marks < high)]:
        if value-marks[-1] >= 10 and high-value >= 10 and all(abs(value-station) >= 10 for station in targets):
            marks.append(float(value))
    marks = sorted(set([*marks,*targets,high]))
    for a, b in zip(marks[:-1], marks[1:]):
        fit(a,b)
    joined = curve.clip(0,low)+result+curve.clip(high,curve.length)
    # Check analytic tangents at every new join; rounded display samples alone
    # must not conceal a kink in the geometry exported to CAD.
    check = RouteCurve(joined)
    for i, (a,b) in enumerate(zip(check.pieces,check.pieces[1:])):
        if a.kind == "spline" or b.kind == "spline":
            continue  # existing sampled endpoint spline has no analytic tangent
        cosine = np.clip(np.dot(a.at(a.length)[1],b.at(0)[1]),-1.,1.)
        if math.degrees(math.acos(cosine)) > .1:
            raise ValueError(f"局部接头切向不连续：接头 {i+1}，请调整影响范围")
    return joined, {"interval_mm": [low,high], "fit_error_mm": max_error,
                    "fit_tolerance_mm": tolerance, "method": "local_tangent_biarc"}
