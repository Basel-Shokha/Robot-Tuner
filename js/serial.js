// serial.js - Web Serial (Chrome / Edge on desktop): line-based connection to the robot.
export const serialSupported = () => typeof navigator !== 'undefined' && 'serial' in navigator;

export class RobotLink {
  constructor() { this.port = null; this.listeners = new Set(); this.buf = ''; this.lines = []; }
  get connected() { return !!this.port; }
  async connect(baud = 115200) {
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: baud });
    this.writer = this.port.writable.getWriter();
    this._read();
  }
  async _read() {
    const dec = new TextDecoder();
    while (this.port && this.port.readable) {
      this.reader = this.port.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          this.buf += dec.decode(value, { stream: true });
          let i;
          while ((i = this.buf.search(/\r?\n/)) >= 0) {
            const line = this.buf.slice(0, i); this.buf = this.buf.slice(i + (this.buf[i] === '\r' ? 2 : 1));
            this.lines.push(line); if (this.lines.length > 2000) this.lines.shift();
            for (const f of this.listeners) f(line);
          }
        }
      } catch (e) { /* port lost */ break; } finally { this.reader.releaseLock(); }
    }
  }
  async disconnect() {
    try { if (this.reader) await this.reader.cancel(); if (this.writer) { this.writer.releaseLock(); } if (this.port) await this.port.close(); } catch (e) { /* ignore */ }
    this.port = null;
  }
  onLine(f) { this.listeners.add(f); return () => this.listeners.delete(f); }
  async send(cmd) { if (!this.writer) throw new Error('not connected'); await this.writer.write(new TextEncoder().encode(cmd + '\n')); for (const f of this.listeners) f('> ' + cmd); }
  /** wait until `test(line)` returns a value (not undefined/false); rejects after timeout ms */
  waitFor(test, timeout = 30000) {
    return new Promise((res, rej) => {
      const off = this.onLine((l) => { const v = test(l); if (v) { off(); clearTimeout(tm); res(v); } });
      const tm = setTimeout(() => { off(); rej(new Error('timeout')); }, timeout);
    });
  }
  /** collect lines until `endTest(line)`; returns all lines */
  collect(endTest, timeout = 30000) {
    const got = [];
    return new Promise((res, rej) => {
      const off = this.onLine((l) => { got.push(l); if (endTest(l)) { off(); clearTimeout(tm); res(got); } });
      const tm = setTimeout(() => { off(); rej(Object.assign(new Error('timeout'), { lines: got })); }, timeout);
    });
  }
}
