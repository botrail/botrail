import assert from "node:assert/strict";
import { test } from "node:test";
import {
  PANEL_ACTIONS,
  PANEL_PX,
  PANEL_SIZE,
  buttonCenter,
  buttonLabel,
  buttonRect,
  hitButton,
  nextSpeed,
  panelText,
  panelTitle,
} from "../src/xr/panel.ts";

const idle = { hasPlayback: false, playing: false, speed: 1, canTeach: false, time: null, duration: null };
const playing = { hasPlayback: true, playing: true, speed: 2, canTeach: true, time: 12.34, duration: 15.6 };

test("five buttons share the lower half without overlapping, inside the quad", () => {
  assert.equal(PANEL_ACTIONS.length, 5);
  assert.equal(PANEL_PX.width / PANEL_PX.height, PANEL_SIZE.width / PANEL_SIZE.height, "canvas and quad agree");
  let right = 0;
  for (let i = 0; i < PANEL_ACTIONS.length; i++) {
    const r = buttonRect(i);
    assert.ok(r.x > right, `button ${i} starts after the previous one`);
    assert.ok(r.x + r.w < 1 && r.y > 0 && r.y + r.h < 1, "inside the quad");
    assert.ok(r.y + r.h <= 0.6, "leaves the title line free");
    right = r.x + r.w;
  }
});

test("a hit on a button's centre names it; the title line and the gaps name nothing", () => {
  for (const action of PANEL_ACTIONS) assert.equal(hitButton(buttonCenter(action)), action);
  assert.equal(hitButton({ x: 0.5, y: 0.9 }), null, "title");
  assert.equal(hitButton({ x: 0.01, y: 0.3 }), null, "left margin");
  const a = buttonRect(0);
  const b = buttonRect(1);
  assert.equal(hitButton({ x: (a.x + a.w + b.x) / 2, y: 0.3 }), null, "the gap between two buttons");
  assert.equal(hitButton({ x: 1.2, y: 0.3 }), null, "off the quad");
});

test("labels follow the playback state", () => {
  assert.equal(buttonLabel("play", idle), "▶ —");
  assert.equal(buttonLabel("play", { ...playing, playing: false }), "▶ play");
  assert.equal(buttonLabel("play", playing), "❚❚ pause");
  assert.equal(buttonLabel("speed", playing), "2×");
  assert.equal(buttonLabel("waypoint", idle), "+ —", "nothing to teach without a robot");
  assert.equal(buttonLabel("waypoint", playing), "+ point");
  assert.equal(buttonLabel("recenter", idle), "⟲ center");
  assert.equal(buttonLabel("exit", idle), "✕ exit");
  assert.equal(panelTitle(playing), "12.3 / 15.6 s");
  assert.match(panelTitle(idle), /no playback/);
  assert.equal(panelText(playing).split("\n").length, 6, "title + five buttons");
  assert.notEqual(panelText(playing), panelText({ ...playing, time: 12.5 }), "the playhead redraws");
});

test("the speed cycles like the dock's button", () => {
  assert.deepEqual([1, 2, 4, 8].map(nextSpeed), [2, 4, 8, 1]);
  assert.equal(nextSpeed(0.5), 1);
});
