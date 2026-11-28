// serial.js - line-based connection to the robot: USB (Web Serial) or Bluetooth Low Energy (Web Bluetooth, Nordic UART
// Service: the generated ESP32 sketch). Both feed the same line handling, PING status and command code.
export const serialSupported = () => typeof navigator !== 'undefined' && 'serial' in navigator;
export const bleSupported = () => typeof navigator !== 'undefined' && 'bluetooth' in navigator;
const NUS = '6e400001-b5a3-f393-e0a9-e50e24dcca9e', NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e', NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';

export class RobotLink {
  constructor() { this.port = null; this.listeners = new Set(); this.buf = ''; this.lines = []; this.lastRx = 0; this.lastPong = 0; this.openedAt = 0; }
  get connected() { return !!this.port || !!this.bleDev; }
  get mode() { return this.bleDev ? 'ble' : this.port ? 'usb' : ''; }
  /** port: an already chosen SerialPort (else the browser asks the student to pick one) */
  async connect(baud = 115200, port = null) {
    this.port = port || await navigator.serial.requestPort();
    await this.port.open({ baudRate: baud });
    // ESP32 boards wire DTR / RTS to EN (reset) and IO0 (boot): Windows switches both on when a port opens, which can hold
    // the ESP32 in reset (silent: "no answer"). Switch them off like the Arduino Serial Monitor does: the robot starts normally.
    // (Uno / Mega already reset on opening; Bluetooth ports have no such lines.)
    try { await this.port.setSignals({ dataTerminalReady: false, requestToSend: false }); } catch (e) { /* not supported (e.g. Bluetooth) */ }
    this.writer = this.port.writable.getWriter();
    this.openedAt = Date.now(); this.lastRx = 0; this.lastPong = 0; this.lastPingAt = 0; this.lost = false; this.lostReason = ''; this.errors = 0;
    this.closing = false;
    this.readDone = this._read();
  }
  /** text from the robot -> lines -> listeners (PONG and LINK are handled here) */
  _feed(text) {
    this.buf += text;
    let i;
    while ((i = this.buf.search(/\r?\n/)) >= 0) {
      const line = this.buf.slice(0, i); this.buf = this.buf.slice(i + (this.buf[i] === '\r' ? 2 : 1));
      this.lastRx = Date.now();
      const pong = /^PONG\b\s*(\w*)/.exec(line.trim());
      if (pong) { this.lastPong = this.lastRx; this.via = pong[1] || ''; continue; }   // keep-alive answers stay out of the log
      const lk = /^LINK bluetooth (connected|disconnected)/.exec(line.trim());
      if (lk) for (const f of this.linkListeners || []) f(lk[1]);
      this.lines.push(line); if (this.lines.length > 2000) this.lines.shift();
      for (const f of this.listeners) f(line);
    }
  }
  /** Bluetooth Low Energy: the browser shows the robots nearby (no pairing in Windows, no COM port) */
  async connectBle() {
    const dev = await navigator.bluetooth.requestDevice({ filters: [{ services: [NUS] }, { namePrefix: 'RobotTuner' }], optionalServices: [NUS] });
    const gatt = await dev.gatt.connect();
    const svc = await gatt.getPrimaryService(NUS);
    this.bleRx = await svc.getCharacteristic(NUS_RX);
    const tx = await svc.getCharacteristic(NUS_TX);
    const dec = new TextDecoder();
    tx.addEventListener('characteristicvaluechanged', (e) => this._feed(dec.decode(e.target.value, { stream: true })));
    await tx.startNotifications();
    dev.addEventListener('gattserverdisconnected', () => {
      if (!this.bleDev || this.closing) return;
      this.bleDev = null; this.lost = true; this.lostReason = 'Bluetooth link lost';
      for (const f of this.listeners) f('# connection lost (Bluetooth)');
    });
    this.bleDev = dev; this.bleQueue = Promise.resolve();
    this.openedAt = Date.now(); this.lastRx = 0; this.lastPong = 0; this.lastPingAt = 0; this.lost = false; this.lostReason = ''; this.closing = false;
  }
  /** BLE writes go one at a time, 20 bytes each (the smallest packet every phone / PC supports) */
  _bleWrite(text) {
    const bytes = new TextEncoder().encode(text);
    this.bleQueue = this.bleQueue.then(async () => {
      for (let i = 0; i < bytes.length; i += 20) {
        const part = bytes.slice(i, i + 20);
        if (this.bleRx.writeValueWithoutResponse) await this.bleRx.writeValueWithoutResponse(part); else await this.bleRx.writeValue(part);
      }
    }).catch(() => { /* link gone: gattserverdisconnected reports it */ });
    return this.bleQueue;
  }
  async _read() {
    const dec = new TextDecoder();
    while (this.port && this.port.readable && !this.closing) {
      this.reader = this.port.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          this._feed(dec.decode(value, { stream: true }));
        }
      } catch (e) {
        // Recoverable line errors (common on Bluetooth COM ports): the browser gives a fresh readable stream, keep reading.
        // Anything else (the device or the Bluetooth link is gone): the port is lost.
        this.errors = (this.errors || 0) + 1;
        if (!['BufferOverrunError', 'FramingError', 'BreakError', 'ParityError'].includes(e && e.name)) { this.lostReason = (e && (e.name + ': ' + e.message)) || 'lost'; break; }
      } finally { try { this.reader.releaseLock(); } catch (e) { /* ignore */ } }
    }
    // the read loop ended without disconnect(): the robot is gone (unplugged, out of Bluetooth range, powered off)
    if (this.port && !this.closing) { const p = this.port; this.port = null; this.lost = true; try { await p.close(); } catch (e) { /* ignore */ } for (const f of this.listeners) f('# connection lost' + (this.lostReason ? ' (' + this.lostReason + ')' : '')); }
  }
  /** close the port for real: stop the read loop first (a still-locked stream would make port.close() fail) */
  async disconnect() {
    if (this.bleDev) { this.closing = true; const d = this.bleDev; this.bleDev = null; try { d.gatt.disconnect(); } catch (e) { /* ignore */ } this.closing = false; this.lost = false; return; }
    if (!this.port) return;
    this.closing = true;
    const port = this.port;
    try { if (this.reader) await this.reader.cancel(); } catch (e) { /* already done */ }
    try { await this.readDone; } catch (e) { /* ignore */ }
    try { if (this.writer) { this.writer.releaseLock(); this.writer = null; } } catch (e) { /* a write still pending */ }
    try { await port.close(); } catch (e) { console.warn('serial close:', e); }
    this.port = null; this.lost = false;
  }
  onLink(f) { (this.linkListeners = this.linkListeners || []).push(f); }
  onLine(f) { this.listeners.add(f); return () => this.listeners.delete(f); }
  /** keep-alive: PING without echo in the log */
  async ping() { if (this.bleDev) { this._bleWrite('PING\n'); return; } if (this.writer) { try { await this.writer.write(new TextEncoder().encode('PING\n')); } catch (e) { /* port lost */ } } }
  async send(cmd) { if (this.bleDev) { await this._bleWrite(cmd + '\n'); for (const f of this.listeners) f('> ' + cmd); return; } if (!this.writer) throw new Error('not connected'); await this.writer.write(new TextEncoder().encode(cmd + '\n')); for (const f of this.listeners) f('> ' + cmd); }
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
