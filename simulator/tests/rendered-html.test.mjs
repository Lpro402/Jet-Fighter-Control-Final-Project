import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

async function render() {
  const workerUrl = new URL("../dist/server/index.js", import.meta.url);
  workerUrl.searchParams.set("test", `${process.pid}-${Date.now()}`);
  const { default: worker } = await import(workerUrl.href);

  return worker.fetch(
    new Request("http://localhost/", {
      headers: { accept: "text/html" },
    }),
    {
      ASSETS: {
        fetch: async () => new Response("Not found", { status: 404 }),
      },
    },
    {
      waitUntil() {},
      passThroughOnException() {},
    },
  );
}

test("server-renders the aircraft control simulator", async () => {
  const response = await render();
  assert.equal(response.status, 200);
  assert.match(response.headers.get("content-type") ?? "", /^text\/html\b/i);

  const html = await response.text();
  assert.match(html, /<title>Advanced Autopilot Testbed<\/title>/i);
  assert.match(html, /Advanced Autopilot/);
  assert.match(html, /מכת רוח רגילה/);
  assert.match(html, /טורבולנציה חזקה/);
  assert.match(html, /דיאגרמת חוג הבקרה הפעיל/);
  assert.match(html, /LQ Servo \(Optimal\)/);
  assert.doesNotMatch(html, /codex-preview|Building your site|SkeletonPreview/i);
});

test("preserves the submitted controllers and isolates the gust", async () => {
  const page = await readFile(new URL("../app/page.jsx", import.meta.url), "utf8");

  assert.match(page, /const P_GAIN = 0\.047953;/);
  assert.match(page, /const RATE_PHI_GAIN = 0\.07408;/);
  assert.match(page, /const RATE_P_GAIN = 0\.01;/);
  assert.match(page, /const LEAD_GAIN = 0\.39697;/);
  assert.match(
    page,
    /const STATE_K = \[9\.29011, -0\.002137, -1\.91889, 0\.088362, -0\.23100\];/,
  );
  assert.match(page, /const STATE_NR = 0\.00551719;/);
  assert.doesNotMatch(page, /MAX_AILERON_RAD|LQ_ANTI_WINDUP_GAIN/);

  assert.match(page, /if \(previousGust\?\.active\) return previousGust;/);
  assert.match(page, /gust\.betaRate = 0;/);
  assert.match(page, /gust\.type === 'turbulence'/);
  assert.match(page, /rollAcceleration: 4\.25/);
  assert.match(page, /profile\.rollAcceleration/);
  assert.match(page, /gust\.yawAcceleration = 0;/);

  assert.match(page, /event\.code === 'KeyR'/);
  assert.match(page, /event\.code === 'KeyG'/);
  assert.match(page, /event\.code === 'KeyD'/);
  assert.match(page, /triggerTurbulence/);
  assert.match(page, /if \(event\.repeat\) return;/);
  assert.doesNotMatch(page, /event\.key\.toLowerCase\(\)/);

  assert.match(page, /const airframeMotion = new THREE\.Group\(\);/);
  assert.match(page, /sim\.current\.X\[4\] \* 4/);
  assert.match(page, /const createWingTrail = colorHex =>/);
  assert.match(page, /leftWingTrail\.group/);
  assert.match(page, /rightWingTrail\.group/);
  assert.match(page, /appendWingTrail/);
  assert.match(page, /clearWingTrail/);
});
