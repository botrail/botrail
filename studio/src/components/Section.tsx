import { useState, type ReactNode } from "react";

const storageKey = (id: string) => `botrail-studio.section.${id}`;

function initialCollapsed(id: string): boolean {
  try {
    return localStorage.getItem(storageKey(id)) === "1";
  } catch {
    return false;
  }
}

/**
 * Collapsible sidebar section. The head toggles; which sections are
 * collapsed persists per section id, so the sidebar keeps the user's
 * shape across sessions.
 */
export function Section({
  id,
  title,
  badge,
  children,
}: {
  id: string;
  title: string;
  /** Right-side head content (badges, small controls); clicking it never toggles. */
  badge?: ReactNode;
  children: ReactNode;
}) {
  const [collapsed, setCollapsed] = useState(() => initialCollapsed(id));
  const toggle = () =>
    setCollapsed((prev) => {
      const next = !prev;
      try {
        localStorage.setItem(storageKey(id), next ? "1" : "0");
      } catch {
        // Private-mode storage failures only cost persistence.
      }
      return next;
    });

  return (
    <section className="panel-section">
      <div className="panel-head" onClick={toggle}>
        <h2>
          <span className="panel-twist">{collapsed ? "▸" : "▾"}</span>
          {title}
        </h2>
        {badge && (
          <div className="panel-head-right" onClick={(e) => e.stopPropagation()}>
            {badge}
          </div>
        )}
      </div>
      {!collapsed && children}
    </section>
  );
}
