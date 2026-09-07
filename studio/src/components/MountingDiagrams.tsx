import { useId } from "react";
import {
  checkFor,
  engagementFor,
  formatRange,
  pairedHole,
  projectHole,
  range,
  selectionHole,
  type Connection,
  type Hole,
  type Inspection,
  type Side,
} from "../mounting";

const COLORS = { flange: "#267eb1", mount: "#b87538" };

export function MountingHoles({
  inspection,
  connection: c,
  selected,
  onSelect,
}: {
  inspection: Inspection;
  connection: Connection;
  selected: string;
  onSelect: (key: string) => void;
}) {
  const overlay = c.flange.mapped && c.mount.mapped;
  const chosen = selectionHole(c, selected),
    paired = pairedHole(inspection, c, selected);
  const radius = Math.max(
    12,
    ...[...c.flange.holes, ...c.mount.holes].map(
      (h) =>
        Math.hypot(...h.position) +
        (h.diameter?.max ?? h.thread?.diameter ?? 0),
    ),
  );
  const scale = 165 / radius;
  const centers: Record<Side, [number, number]> = overlay
    ? { flange: [245, 285], mount: [245, 285] }
    : { flange: [245, 285], mount: [650, 285] };
  const position = (side: Side, h: Hole) =>
    overlay ? projectHole(c, side, h) : [...h.position, 0];
  const draw = (
    side: Side,
    h: Hole,
    i: number,
    origin: [number, number],
    zoom = scale,
    local = [0, 0],
    label = true,
  ) => {
    const p = position(side, h),
      x = origin[0] + (p[0] - local[0]) * zoom,
      y = origin[1] - (p[1] - local[1]) * zoom;
    const r = ((h.diameter?.min ?? h.thread?.diameter ?? 0) * zoom) / 2;
    const active =
      `${side}:${h.id}` === selected ||
      (paired?.side === side && paired.index === i);
    return (
      <g
        key={`${side}:${h.id}`}
        role="button"
        tabIndex={0}
        aria-label={`${side} hole ${h.id}`}
        onClick={() => onSelect(`${side}:${h.id}`)}
        onKeyDown={(e) => {
          if (e.key === "Enter" || e.key === " ") {
            e.preventDefault();
            onSelect(`${side}:${h.id}`);
          }
        }}
      >
        <circle cx={x} cy={y} r={Math.max(r + 8, 15)} fill="transparent" />
        {active && (
          <circle
            cx={x}
            cy={y}
            r={r + 7}
            fill="none"
            stroke="#9558b3"
            strokeWidth={2}
          />
        )}
        {r > 0 && (
          <circle
            cx={x}
            cy={y}
            r={r}
            fill="none"
            stroke={COLORS[side]}
            strokeWidth={2}
            strokeDasharray={side === "mount" ? "5 3" : undefined}
          />
        )}
        <path d={`M${x - 4} ${y}h8M${x} ${y - 4}v8`} stroke={COLORS[side]} />
        {label && (
          <text
            x={side === "flange" ? x - r - 10 : x + r + 10}
            y={y - 10}
            textAnchor={side === "flange" ? "end" : "start"}
            fontSize={12}
          >
            {side === "flange" ? "F" : "M"}:{h.id}
          </text>
        )}
      </g>
    );
  };
  if (!c.flange.holes.length && !c.mount.holes.length)
    return (
      <div className="mounting-empty">
        <b>Hole dimensions are not documented</b>
        <p>Select a finding to see the drawings needed for this connection.</p>
      </div>
    );
  const center = chosen ? position(chosen.side, chosen.hole) : [0, 0, 0];
  const delta =
    chosen && paired
      ? projectHole(c, chosen.side, chosen.hole).map(
          (x, i) => x - projectHole(c, paired.side, paired.hole)[i],
        )
      : null;
  return (
    <svg
      className="mounting-diagram"
      viewBox="0 0 900 550"
      aria-label="Mounting hole drawing"
      role="img"
    >
      <text x={35} y={42} fontSize={17}>
        {overlay
          ? "Both faces in the flange coordinate system"
          : "Separate drawing coordinates · model mapping unconfirmed"}
      </text>
      <text x={35} y={68} fontSize={12}>
        Blue: flange / solid · Brown: mount / dashed · Click a hole to inspect
        it
      </text>
      {(overlay ? (["flange"] as const) : (["flange", "mount"] as const)).map(
        (side) => {
          const [x, y] = centers[side];
          return (
            <g key={side}>
              <path
                d={`M${x - 185} ${y}h370M${x} ${y - 185}v370`}
                stroke="#b0c0cc"
                strokeDasharray="5 5"
              />
              <text x={x - 170} y={105} fontSize={13}>
                {overlay
                  ? "Face projection (mm)"
                  : `${side} · ${c[side].frame}`}
              </text>
            </g>
          );
        },
      )}
      {(["flange", "mount"] as const).flatMap((side) =>
        c[side].holes.map((h, i) => draw(side, h, i, centers[side])),
      )}
      {(["flange", "mount"] as const).flatMap((side) =>
        c[side].locators.map((l) => {
          const p = overlay ? projectHole(c, side, l) : [...l.position, 0];
          return (
            <g key={`${side}:locator:${l.id}`}>
              <circle
                cx={centers[side][0] + p[0] * scale}
                cy={centers[side][1] - p[1] * scale}
                r={((l.diameter?.min ?? 0) * scale) / 2}
                fill="none"
                stroke={COLORS[side]}
                strokeDasharray="2 3"
              />
              <title>{`${side} ${l.kind}: ${l.id}`}</title>
            </g>
          );
        }),
      )}
      {overlay && chosen && (
        <>
          <rect
            x={475}
            y={115}
            width={380}
            height={338}
            rx={12}
            fill="#f7f9fb"
            stroke="#c4d0d9"
          />
          <text x={495} y={145} fontSize={14}>
            {paired ? "Paired holes" : "Local detail · pairing not established"}
          </text>
          {(["flange", "mount"] as const).flatMap((side) =>
            c[side].holes.flatMap((h, i) => {
              const p = position(side, h);
              return Math.hypot(p[0] - center[0], p[1] - center[1]) < 8
                ? [draw(side, h, i, [650, 280], 12, center, false)]
                : [];
            }),
          )}
          <text x={495} y={385} fontSize={16}>
            {delta
              ? `Center distance: ${Math.hypot(...delta).toFixed(3)} mm`
              : `${chosen.side} ${chosen.hole.id}: (${chosen.hole.position.join(", ")}) mm`}
          </text>
          <text x={495} y={417} fontSize={11}>
            {delta
              ? `ΔXYZ: ${delta.map((x) => x.toFixed(3)).join(", ")} mm`
              : "Circles show declared locations; they do not establish a match."}
          </text>
        </>
      )}
      <text x={35} y={505} fontSize={12}>
        {overlay
          ? "Nominal thread diameter and minimum clearance diameter; tolerances are listed in the inspector."
          : "Drawing values remain useful before their location on the 3D model has been confirmed."}
      </text>
      <text x={35} y={530} fontSize={11}>
        {checkFor(inspection, c, "mating_plane")?.message ??
          "Face position and orientation need their own evidence."}
      </text>
    </svg>
  );
}

export function MountingSection({
  inspection,
  connection,
  selected,
}: {
  inspection: Inspection;
  connection: Connection;
  selected: string;
}) {
  const hatch = useId().replace(/:/g, ""),
    check = engagementFor(inspection, connection, selected);
  if (!check)
    return (
      <div className="mounting-empty">
        <b>No evaluated screw stack for this hole</b>
        <p>
          Select a clearance hole with a documented fastener. Hole pairing,
          frame correspondence and bearing conditions must be established before
          screw engagement can be checked.
        </p>
      </div>
    );
  const v = check.inputs,
    length = range(v.length_mm),
    grip = range(v.grip_mm),
    washer = range(v.washer_mm),
    start = range(v.thread_start_mm),
    depth = range(v.usable_depth_mm),
    engagement = range(v.engagement_mm),
    tip = range(v.tip_penetration_mm),
    minimum =
      typeof v.min_engagement_mm === "number" ? v.min_engagement_mm : null;
  if (!length || !grip || !washer)
    return (
      <div className="mounting-empty">
        <b>Bearing-stack dimensions are incomplete</b>
        <p>{check.message}</p>
      </div>
    );
  const scale =
    320 /
    Math.max(length.max + 3, grip.max + washer.max + (depth?.max ?? 6) + 2);
  const y0 = 108,
    face = y0 + (grip.max + washer.max) * scale,
    tipY = y0 + length.max * scale;
  const color =
    check.status === "fail"
      ? "#b64c47"
      : check.status === "pass"
        ? "#287d62"
        : "#93672f";
  const allowedEnd = face + (depth?.max ?? 7) * scale;
  return (
    <svg
      className="mounting-diagram"
      viewBox="0 0 900 550"
      role="img"
      aria-label="Screw engagement schematic section"
    >
      <defs>
        <pattern
          id={hatch}
          width={9}
          height={9}
          patternUnits="userSpaceOnUse"
          patternTransform="rotate(45)"
        >
          <path d="M0 0V9" stroke="#94b0c3" strokeWidth={2} />
        </pattern>
      </defs>
      <text x={35} y={42} fontSize={17}>
        Screw engagement · schematic section
      </text>
      <text x={35} y={68} fontSize={12}>
        Dimensions from the checker. Head shape and physical bore bottom are not
        inferred.
      </text>
      <rect
        x={205}
        y={y0 + washer.max * scale}
        width={100}
        height={grip.max * scale}
        fill="#e0bd99"
        stroke="#b9875b"
      />
      <rect
        x={359}
        y={y0 + washer.max * scale}
        width={100}
        height={grip.max * scale}
        fill="#e0bd99"
        stroke="#b9875b"
      />
      <rect
        x={205}
        y={face}
        width={100}
        height={Math.max(0, allowedEnd - face)}
        fill={`url(#${hatch})`}
        stroke="#849ead"
      />
      <rect
        x={359}
        y={face}
        width={100}
        height={Math.max(0, allowedEnd - face)}
        fill={`url(#${hatch})`}
        stroke="#849ead"
      />
      <rect
        x={285}
        y={y0}
        width={94}
        height={washer.max * scale}
        fill="#a3b5c2"
      />
      <rect
        x={311}
        y={y0}
        width={42}
        height={length.min * scale}
        fill="#c2ccd5"
        stroke="#61798d"
      />
      {length.max !== length.min && (
        <rect
          x={311}
          y={y0 + length.min * scale}
          width={42}
          height={(length.max - length.min) * scale}
          fill="#c2ccd560"
          stroke="#61798d"
          strokeDasharray="4 3"
        />
      )}
      <path d={`M285 ${y0}H379`} stroke="#425d72" strokeWidth={3} />
      <path d={`M195 ${face}H465`} stroke="#b9875b" strokeDasharray="5 3" />
      {start && start.max > 0 && (
        <rect
          x={305}
          y={face}
          width={54}
          height={start.max * scale}
          fill="#e7c58960"
          stroke="#b58a50"
          strokeDasharray="4 3"
        />
      )}
      {depth && (
        <>
          <path
            d={`M291 ${face + depth.min * scale}H475M291 ${allowedEnd}H475`}
            stroke="#b66b45"
            strokeDasharray="5 3"
          />
          {depth.max !== depth.min && (
            <rect
              x={291}
              y={face + depth.min * scale}
              width={184}
              height={(depth.max - depth.min) * scale}
              fill="#d28d6233"
            />
          )}
        </>
      )}
      {minimum !== null && (
        <path
          d={`M300 ${face + ((start?.max ?? 0) + minimum) * scale}H470`}
          stroke="#438a77"
          strokeDasharray="3 3"
        />
      )}
      <path
        d={`M414 ${face + (start?.max ?? 0) * scale}V${tipY}`}
        stroke={color}
        strokeWidth={4}
      />
      <text x={490} y={122} fontSize={13}>
        Under-head length: {formatRange(length)}
      </text>
      <text x={490} y={152} fontSize={13}>
        Washer: {formatRange(washer)}
      </text>
      <text x={490} y={182} fontSize={13}>
        Bearing stack: {formatRange(grip)}
      </text>
      <text x={490} y={226} fontSize={13}>
        Unthreaded lead: {formatRange(start)}
      </text>
      <text x={490} y={263} fontSize={18} style={{ fill: color }}>
        Engagement: {formatRange(engagement)}
      </text>
      <text x={490} y={296} fontSize={13}>
        Minimum engagement:{" "}
        {minimum === null ? "Not documented" : `${minimum} mm`}
      </text>
      <text x={490} y={333} fontSize={13}>
        Tip penetration: {formatRange(tip)}
      </text>
      <text x={490} y={366} fontSize={13}>
        Allowed tip depth: {formatRange(depth)}
      </text>
      <text x={35} y={502} fontSize={12}>
        Dashed lines and bands denote limits or intervals, not an inferred
        physical hole bottom.
      </text>
      <text x={35} y={530} fontSize={11}>
        The drawing shows the selected bearing stack. Plate width and screw head
        dimensions are schematic.
      </text>
    </svg>
  );
}
