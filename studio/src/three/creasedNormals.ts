// Normals for a mesh that came as loose triangles (an STL): shared across
// the edges a surface is meant to be smooth over, kept apart across the
// ones it is meant to break at.
//
// A triangle soup shaded with one normal a face shows every facet — fine
// for a dense scan, and what makes a maker's simplified model (a few
// hundred triangles a link) read as cut glass. Here each corner takes the
// normals of the faces that meet at its point and lie within the crease
// angle of its own face, weighted by the angle each of them subtends
// there: a turned surface shades round, a corner stays a corner, and how a
// flat face happens to be cut into triangles changes nothing.
//
// Corners are matched by their coordinates exactly — the same three floats
// — which is how the triangles of one STL share a point. No tolerance: a
// tolerance is a length, and a mesh does not say its unit.

/** One normal per corner (`xyz`, in corner order) for the loose triangles
 * `position` (nine numbers a face). Faces meeting at a point within
 * `crease` radians of each other share their normals there. */
export function creasedNormals(position: Float32Array, crease: number): Float32Array {
  const faces = Math.floor(position.length / 9);
  const corners = faces * 3;
  const unit = new Float32Array(faces * 3);
  const angle = new Float32Array(corners);
  for (let f = 0; f < faces; f++) {
    const p = 9 * f;
    const e = [0, 1, 2].map((k) => [
      position[p + 3 * ((k + 1) % 3)] - position[p + 3 * k],
      position[p + 3 * ((k + 1) % 3) + 1] - position[p + 3 * k + 1],
      position[p + 3 * ((k + 1) % 3) + 2] - position[p + 3 * k + 2],
    ]);
    // The face normal: edge 0 x edge 1.
    let nx = e[0][1] * e[1][2] - e[0][2] * e[1][1];
    let ny = e[0][2] * e[1][0] - e[0][0] * e[1][2];
    let nz = e[0][0] * e[1][1] - e[0][1] * e[1][0];
    const length = Math.hypot(nx, ny, nz);
    if (length > 0) {
      nx /= length;
      ny /= length;
      nz /= length;
    }
    unit.set([nx, ny, nz], 3 * f);
    // The angle at corner k: between the edge leaving it and the reverse
    // of the edge arriving. A face with no area subtends none.
    for (let k = 0; k < 3; k++) {
      const out = e[k];
      const back = e[(k + 2) % 3];
      const lengths = Math.hypot(...out) * Math.hypot(...back);
      const cos = lengths > 0 ? -(out[0] * back[0] + out[1] * back[1] + out[2] * back[2]) / lengths : 1;
      angle[3 * f + k] = length > 0 ? Math.acos(Math.min(1, Math.max(-1, cos))) : 0;
    }
  }

  // Which corners stand on the same point: an id per distinct point, then
  // the corners of each point side by side.
  const bits = new Uint32Array(position.buffer, position.byteOffset, corners * 3);
  const SIGN = 0x80000000;
  const ids = new Map<string, number>();
  const point = new Int32Array(corners);
  for (let c = 0; c < corners; c++) {
    // (Minus zero is zero.)
    const [x, y, z] = [bits[3 * c], bits[3 * c + 1], bits[3 * c + 2]].map((b) => (b === SIGN ? 0 : b));
    const key = `${x},${y},${z}`;
    let id = ids.get(key);
    if (id === undefined) {
      id = ids.size;
      ids.set(key, id);
    }
    point[c] = id;
  }
  const first = new Int32Array(ids.size + 1);
  for (let c = 0; c < corners; c++) first[point[c] + 1]++;
  for (let i = 0; i < ids.size; i++) first[i + 1] += first[i];
  const around = new Int32Array(corners);
  const next = first.slice(0, ids.size);
  for (let c = 0; c < corners; c++) around[next[point[c]]++] = c;

  const limit = Math.cos(crease);
  const normals = new Float32Array(corners * 3);
  for (let c = 0; c < corners; c++) {
    const f = Math.floor(c / 3);
    const [ux, uy, uz] = [unit[3 * f], unit[3 * f + 1], unit[3 * f + 2]];
    let [x, y, z] = [0, 0, 0];
    for (let i = first[point[c]]; i < first[point[c] + 1]; i++) {
      const g = Math.floor(around[i] / 3);
      const [vx, vy, vz] = [unit[3 * g], unit[3 * g + 1], unit[3 * g + 2]];
      if (g === f || ux * vx + uy * vy + uz * vz > limit) {
        const w = angle[around[i]];
        x += w * vx;
        y += w * vy;
        z += w * vz;
      }
    }
    const length = Math.hypot(x, y, z);
    // Nothing to average (a face with no area): its own normal, such as it is.
    normals.set(length > 0 ? [x / length, y / length, z / length] : [ux, uy, uz], 3 * c);
  }
  return normals;
}
