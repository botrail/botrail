import { useMemo, useState } from "react";

import { backendSupportsHttp } from "../backend";
import {
  NO_FILTER,
  attributeCell,
  bomCaption,
  bomRows,
  catalogCell,
  namesCell,
  rowSelected,
  totalColumns,
  type BomFilter,
} from "../bomTable";
import { useDockReserve } from "../dockReserve";
import { partTitle } from "../partIdentity";
import { useStudioStore } from "../store";
import { downloadBlob } from "./Header";
import { OverlayTabs } from "./OverlayTabs";

/**
 * The bill of materials over the viewport: every line the host derives
 * (`Scene::bom`) — identified or not, merged by product, with the
 * residents and derived lines it stands for — in the columns the CSV
 * writes, plus the attributes that have a total. A click on a row
 * selects what it stands for, so the PART card follows; the filters cut
 * it to the purchasing to-do (`unidentified`) or to the lines nothing in
 * the scene draws (`derived`); `⤓ csv` is `Bom::to_csv`, the same bytes
 * `export_bom` writes.
 */
export function BomOverlay() {
  const open = useStudioStore((s) => s.bomOpen);
  const setOpen = useStudioStore((s) => s.setBomOpen);
  const lines = useStudioStore((s) => s.bom);
  const totals = useStudioStore((s) => s.bomTotals);
  const selection = useStudioStore((s) => s.selection);
  const selectTarget = useStudioStore((s) => s.selectTarget);
  const docked = useStudioStore((s) => s.playback !== null);
  const [panel, setPanel] = useState<HTMLDivElement | null>(null);
  useDockReserve(panel, docked);
  const [filter, setFilter] = useState<BomFilter>(NO_FILTER);
  const [csvError, setCsvError] = useState<string | null>(null);

  const rows = useMemo(() => bomRows(lines, filter), [lines, filter]);
  const columns = useMemo(() => totalColumns(totals), [totals]);
  const derived = lines.filter((l) => l.targets.some((t) => t.derived)).length;
  const unidentified = lines.filter((l) => !l.identified).length;

  if (!open) return null;

  const onCsv = async () => {
    try {
      const res = await fetch("/api/bom.csv");
      if (!res.ok) throw new Error(await res.text());
      downloadBlob(await res.blob(), "bom.csv");
      setCsvError(null);
    } catch (e) {
      setCsvError(`csv failed: ${String(e)}`);
    }
  };

  return (
    <div className="sfc-overlay io-overlay bom-overlay" ref={setPanel}>
      <div className="sfc-head">
        <OverlayTabs active="bom" />
        <span className="sfc-caption">{bomCaption(lines, totals)}</span>
        {csvError && (
          <span className="header-error" title={csvError}>
            {csvError}
          </span>
        )}
        <label className="io-filter" title="only lines nobody has identified — bom.unidentified()">
          <input
            type="checkbox"
            checked={filter.unidentified}
            onChange={(e) => setFilter({ ...filter, unidentified: e.target.checked })}
          />
          unidentified{unidentified > 0 ? ` (${unidentified})` : ""}
        </label>
        {derived > 0 && (
          <label className="io-filter" title="only lines the bill made up with no scene object — tools, controllers">
            <input
              type="checkbox"
              checked={filter.derived}
              onChange={(e) => setFilter({ ...filter, derived: e.target.checked })}
            />
            derived ({derived})
          </label>
        )}
        {backendSupportsHttp() && (
          <button
            className="timeline-button"
            onClick={onCsv}
            disabled={lines.length === 0}
            title="download the bill as CSV — the same bytes Scene.export_bom writes"
          >
            ⤓ csv
          </button>
        )}
        <button
          className="timeline-button"
          onClick={() => setOpen(false)}
          title="close (▤ on the scene tree reopens it)"
        >
          ×
        </button>
      </div>
      <div className="sfc-scroll io-scroll">
        {rows.length === 0 ? (
          <div className="sfc-empty hint">
            {lines.length === 0 ? "nothing on the bill yet" : "no lines match the filter"}
          </div>
        ) : (
          <table className="io-table bom-table">
            <thead>
              <tr>
                <th>#</th>
                <th>category</th>
                <th>manufacturer</th>
                <th>model</th>
                <th>catalog</th>
                <th className="num">qty</th>
                <th>names</th>
                {columns.map((c) => (
                  <th key={c} className="num" title={c}>
                    {c}
                  </th>
                ))}
              </tr>
            </thead>
            <tbody>
              {rows.map((line, i) => {
                const target = line.targets[0];
                const picked = rowSelected(line, selection);
                return (
                  <tr
                    key={`${line.category}/${namesCell(line)}`}
                    className={`io-row io-clickable${line.identified ? "" : " bom-unidentified"}${
                      picked ? " bom-picked" : ""
                    }`}
                    onClick={() => target && selectTarget(target)}
                    title={`${partTitle(line)} — select in the scene tree`}
                  >
                    <td className="io-muted">{lines.indexOf(line) + 1}</td>
                    <td>{line.category}</td>
                    <td>{line.manufacturer ?? (line.identified ? "" : "?")}</td>
                    <td>{line.model ?? (line.identified ? "" : "?")}</td>
                    <td className="io-muted">{catalogCell(line)}</td>
                    <td className="num">{line.qty}</td>
                    <td className="io-muted">{namesCell(line)}</td>
                    {columns.map((c) => (
                      <td key={c} className="num">
                        {attributeCell(line, c)}
                      </td>
                    ))}
                    {i < 0 ? null : null}
                  </tr>
                );
              })}
            </tbody>
            {columns.length > 0 && (
              <tfoot>
                <tr>
                  <td colSpan={7} className="io-muted">
                    totals
                  </td>
                  {columns.map((c) => (
                    <td key={c} className="num">
                      {attributeCell({ ...rows[0], attributes: totals }, c)}
                    </td>
                  ))}
                </tr>
              </tfoot>
            )}
          </table>
        )}
      </div>
    </div>
  );
}
