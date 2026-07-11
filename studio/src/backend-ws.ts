// WebSocket backend: connects to `/ws` on the same origin and auto-
// reconnects with exponential backoff (capped at 5s).

import type { BackendHandlers, SessionBackend } from "./backend";

const MIN_BACKOFF_MS = 500;
const MAX_BACKOFF_MS = 5000;

export class WsBackend implements SessionBackend {
  private socket: WebSocket | null = null;
  private backoff = MIN_BACKOFF_MS;
  private reconnectTimer: number | null = null;
  private handlers: BackendHandlers | null = null;

  start(handlers: BackendHandlers): void {
    if (this.handlers) return;
    this.handlers = handlers;
    this.open();
  }

  send(text: string): void {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(text);
    }
  }

  private url(): string {
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    return `${proto}//${location.host}/ws`;
  }

  private open(): void {
    const handlers = this.handlers;
    if (!handlers) return;
    handlers.onStatus("connecting");
    const socket = new WebSocket(this.url());
    this.socket = socket;

    socket.onopen = () => {
      this.backoff = MIN_BACKOFF_MS;
      handlers.onStatus("connected");
    };
    socket.onmessage = (ev) => handlers.onMessage(ev.data as string);
    socket.onclose = () => {
      if (this.socket === socket) this.socket = null;
      handlers.onStatus("disconnected");
      this.scheduleReconnect();
    };
    // An error is followed by a close event; let onclose drive reconnection.
    socket.onerror = () => socket.close();
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer !== null) return;
    const delay = this.backoff;
    this.backoff = Math.min(this.backoff * 2, MAX_BACKOFF_MS);
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.open();
    }, delay);
  }
}
