// Camera and XYZ gizmo migrated from the supplied hvwire pipe editor.
const add = (a, b) => a.map((v, i) => v + b[i]),
  sub = (a, b) => a.map((v, i) => v - b[i]);
const mul = (a, t) => a.map((v) => v * t),
  dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0),
  norm = (a) => Math.hypot(...a);
// A free orthonormal camera frame: no world-up lock or Euler-angle poles.
function presetBasis(view) {
  const [yaw, pitch] = {
    iso: [-0.7, 0.8],
    xy: [0, Math.PI / 2],
    xz: [0, 0],
    yz: [Math.PI / 2, 0],
  }[view];
  return {
    right: [Math.cos(yaw), Math.sin(yaw), 0],
    up: [-Math.sin(yaw) * Math.sin(pitch), Math.cos(yaw) * Math.sin(pitch), Math.cos(pitch)],
    depth: [Math.sin(yaw) * Math.cos(pitch), -Math.cos(yaw) * Math.cos(pitch), Math.sin(pitch)],
  };
}
function basis() {
  return cameraBasis;
}
function setView(view) {
  cameraBasis = presetBasis(view);
}
function cross(a, b) {
  return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
}
function orbitBy(dx, dy) {
  const distance = Math.hypot(dx, dy);
  if (!distance) return;
  const b = basis();
  // Rotate about screen-local axes, so the facing side follows the pointer
  // even after crossing a pole or turning world Z upside down.
  const axis = mul(add(mul(b.right, -dy), mul(b.up, -dx)), 1 / distance);
  const angle = distance * 0.008, c = Math.cos(angle), s = Math.sin(angle);
  const rotate = (v) => add(add(mul(v, c), mul(cross(axis, v), s)), mul(axis, dot(axis, v) * (1-c)));
  const r = rotate(b.right), right = mul(r, 1 / norm(r));
  const u = rotate(b.up), perpendicular = sub(u, mul(right, dot(u, right)));
  const up = mul(perpendicular, 1 / norm(perpendicular));
  // Re-orthogonalize to avoid accumulated scale/skew over long drag sessions.
  cameraBasis = {right, up, depth: cross(right, up)};
}
function project(p) {
  const d = sub(p, center),
    b = basis();
  return [
    width / 2 + pan[0] + dot(d, b.right) * scale,
    height / 2 + pan[1] - dot(d, b.up) * scale,
    dot(d, b.depth),
  ];
}
function schedule() {
  if (!frame)
    frame = requestAnimationFrame(() => {
      frame = 0;
      draw();
    });
}
function stations(path) {
  const route = state.routes.find(r => r.path === path);
  if (route?.stations) return route.stations;
  let values = [0];
  for (let i = 1; i < path.length; i++)
    values.push(values[i - 1] + norm(sub(path[i], path[i - 1])));
  return values;
}
function at(path, s, st = stations(path)) {
  const geometry = state.routes.find(r => r.path === path)?.editGeometry;
  if (geometry?.length) {
    let low = 0, high = geometry.length-1;
    while (low < high) {
      const mid = Math.ceil((low+high)/2);
      if (geometry[mid].startStation <= s) low = mid;
      else high = mid-1;
    }
    const seg = geometry[low];
    const local = Math.max(0, Math.min(seg.length, s-seg.startStation));
    if (seg.type === "line") return add(seg.start, mul(sub(seg.end,seg.start),local/seg.length));
    if (seg.type === "arc") {
      const radial = sub(seg.start,seg.center), u = mul(radial,1/norm(radial));
      const end = sub(seg.end,seg.center), raw = sub(end,mul(u,dot(end,u))), v = mul(raw,1/norm(raw));
      const angle = local/seg.radius;
      return add(seg.center,mul(add(mul(u,Math.cos(angle)),mul(v,Math.sin(angle))),seg.radius));
    }
  }
  let lo = 1,
    hi = st.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (st[mid] < s) lo = mid + 1;
    else hi = mid;
  }
  const t = Math.max(0, Math.min(1, (s - st[lo - 1]) / (st[lo] - st[lo - 1])));
  return add(path[lo - 1], mul(sub(path[lo], path[lo - 1]), t));
}
function bump(path, s, delta, radius) {
  const st = stations(path),
    total = st.at(-1);
  const limits = state.routes.find(r => r.path === path)?.editLimits || [40,total-40];
  let low = Math.max(limits[0], s - radius),
    high = Math.min(limits[1], s + radius);
  for (const [a,b] of (state.routes.find(r => r.path === path)?.fixedIntervals || [])) {
    if (s >= a && s <= b) throw Error("挂点及其两侧直线固定");
    if (b < s) low = Math.max(low,b);
    if (a > s) high = Math.min(high,a);
  }
  if (s <= low + 5 || s >= high - 5)
    throw Error("端点和接入切向段固定，请选择中间的位置");
  const marks = [...st, s, low, high];
  for (let x = low; x < high; x += 10) marks.push(x);
  return [...new Set(marks)]
    .sort((a, b) => a - b)
    .map((x) => {
      const t = Math.min(
        1,
        Math.max(0, x <= s ? (s - x) / (s - low) : (x - s) / (high - s)),
      );
      const w = 1 - (10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5);
      return add(at(path, x, st), mul(delta, w));
    });
}

const moveAxes = [
  {name: "X", vector: [1, 0, 0], color: "#ff6868"},
  {name: "Y", vector: [0, 1, 0], color: "#63e68b"},
  {name: "Z", vector: [0, 0, 1], color: "#65a8ff"},
];
function axisHandles() {
  if (!editMode || !picked) return [];
  const origin = project(selectedPointPosition()), b = basis();
  return moveAxes.map(axis => {
    const direction = [dot(axis.vector, b.right), -dot(axis.vector, b.up)];
    const length = Math.hypot(...direction);
    return {...axis, origin, direction, disabled: length < .12,
      end: [origin[0]+direction[0]*90, origin[1]+direction[1]*90]};
  });
}
function drawMoveAxes() {
  for (const axis of axisHandles()) {
    const [x,y] = axis.origin, [ex,ey] = axis.end;
    ctx.save();
    ctx.strokeStyle = ctx.fillStyle = axis.color;
    ctx.lineWidth = gesture?.axis === axis.name ? 5 : 3;
    ctx.shadowColor = "#071018"; ctx.shadowBlur = 4;
    if (axis.disabled) {
      ctx.globalAlpha = .55;
      ctx.beginPath();ctx.arc(x,y,5,0,Math.PI*2);ctx.stroke();
      ctx.font = "bold 12px system-ui";
      ctx.fillText(axis.name+" · 旋转视图后拖动",x+16,y+25);
    } else {
      const length = Math.hypot(ex-x,ey-y), ux=(ex-x)/length, uy=(ey-y)/length;
      ctx.beginPath();ctx.moveTo(x+ux*15,y+uy*15);ctx.lineTo(ex,ey);ctx.stroke();
      ctx.beginPath();ctx.moveTo(ex,ey);
      ctx.lineTo(ex-ux*13-uy*6,ey-uy*13+ux*6);
      ctx.lineTo(ex-ux*13+uy*6,ey-uy*13-ux*6);ctx.closePath();ctx.fill();
      ctx.font = "bold 14px system-ui";
      ctx.fillText(axis.name,ex+ux*12-4,ey+uy*12+5);
    }
    ctx.restore();
  }
}
function pickAxis(x,y) {
  let best = null;
  for (const axis of axisHandles()) {
    if (axis.disabled) continue;
    const [ox,oy]=axis.origin, dx=axis.end[0]-ox, dy=axis.end[1]-oy;
    const length=Math.hypot(dx,dy), along=((x-ox)*dx+(y-oy)*dy)/length;
    if (along < 18 || along > length+20) continue;
    const distance=Math.hypot(x-ox-dx*along/length,y-oy-dy*along/length);
    if (distance <= 10 && (!best || distance < best.distance)) best={...axis,distance};
  }
  return best;
}
function dragDelta(dx, dy, axisName) {
  const axis = moveAxes.find(a => a.name === axisName);
  if (!axis) throw Error("请拖动 X、Y 或 Z 坐标轴");
  const b=basis(), x=dot(axis.vector,b.right), y=-dot(axis.vector,b.up);
  const denominator=x*x+y*y;
  if (denominator < .12**2) throw Error("该轴接近垂直屏幕，请旋转视图后拖动");
  return mul(axis.vector,(dx*x+dy*y)/(scale*denominator));
}
