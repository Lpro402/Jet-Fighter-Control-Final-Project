"use client";

import React, { useRef, useEffect, useState } from 'react';
import * as THREE from 'three';
import {
  Activity,
  Compass,
  Cpu,
  Crosshair,
  Download,
  Gauge,
  Monitor,
  Pause,
  Play,
  RotateCcw,
  ShieldAlert,
  Wind,
  Zap,
} from 'lucide-react';

const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;

const CONTROLLERS = {
  P: { name: 'P Controller', desc: 'בקרת שגיאה. השארת שגיאת מצב מתמיד.', color: '#94a3b8', glow: 'rgba(148, 163, 184, 0.5)' },
  PD: { name: 'Roll-rate feedback (p + ϕ)', desc: 'משוב קצב גלגול פנימי. דמוי-PD.', color: '#a78bfa', glow: 'rgba(167, 139, 250, 0.5)' },
  Lead: { name: 'Phase Lead', desc: 'רשת קידום פאזה. תגובה מהירה.', color: '#fb923c', glow: 'rgba(251, 146, 60, 0.5)' },
  FullState: { name: 'Full-State Feedback', desc: 'השמת קטבים עם קדם-מסנן N_r.', color: '#38bdf8', glow: 'rgba(56, 189, 248, 0.5)' },
  LQServo: { name: 'LQ Servo (Optimal)', desc: 'בקר אופטימלי משולב אינטגרטור.', color: '#10b981', glow: 'rgba(16, 185, 129, 0.5)' }
};

// Final controller constants from the submitted design
const P_GAIN = 0.047953;
const RATE_PHI_GAIN = 0.07408;
const RATE_P_GAIN = 0.01;

const LEAD_GAIN = 0.39697;
const LEAD_ZERO = 3;
const LEAD_POLE = 15;
// Realisation:
// C(s) = K(s+z)/(s+p)
//      = K - K(p-z)/(s+p)
const LEAD_STATE_GAIN = LEAD_GAIN * (LEAD_POLE - LEAD_ZERO); // 4.76364

const STATE_K = [9.29011, -0.002137, -1.91889, 0.088362, -0.23100];
const STATE_NR = 0.00551719;
const MAX_AILERON_RAD = 5 * D2R;
const LQ_ANTI_WINDUP_GAIN = 3;

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

const CONTROL_ARCHITECTURES = {
  P: {
    formula: 'δc = Kφ(φc − φ)',
    controllerLabel: 'P GAIN',
    controllerValue: `Kφ = ${P_GAIN.toFixed(5)}`,
    summary: 'חוג משוב יחידה פשוט על זווית הגלגול.',
    feedback: [
      { label: 'Angle feedback', signal: 'φ', valueKey: 'phi', unit: '°' },
    ],
  },
  PD: {
    formula: 'δc = Kφ(φc − φ) − Kp·p',
    controllerLabel: 'ANGLE + RATE',
    controllerValue: `Kφ=${RATE_PHI_GAIN.toFixed(5)}  Kp=${RATE_P_GAIN.toFixed(3)}`,
    summary: 'חוג זווית חיצוני עם משוב קצב גלגול פנימי.',
    feedback: [
      { label: 'Angle feedback', signal: 'φ', valueKey: 'phi', unit: '°' },
      { label: 'Rate feedback', signal: 'p', valueKey: 'p', unit: '°/s' },
    ],
  },
  Lead: {
    formula: 'Clead(s) = 0.39697(s+3)/(s+15)',
    controllerLabel: 'PHASE LEAD',
    controllerValue: 'zero −3  |  pole −15',
    summary: 'רשת קידום פאזה דינמית שמאיצה את תגובת החוג.',
    feedback: [
      { label: 'Angle feedback', signal: 'φ', valueKey: 'phi', unit: '°' },
      { label: 'Lead state', signal: 'xc', valueKey: 'xc', unit: '' },
    ],
  },
  FullState: {
    formula: 'δc = Nr·φc − Kx',
    controllerLabel: 'STATE FEEDBACK',
    controllerValue: `Nr = ${STATE_NR.toFixed(6)}`,
    summary: 'משוב מכל מצבי המטוס עם קדם־מסנן לפקודת הייחוס.',
    feedback: [
      { label: 'Sideslip', signal: 'β', valueKey: 'beta', unit: '°' },
      { label: 'Roll rate', signal: 'p', valueKey: 'p', unit: '°/s' },
      { label: 'Roll angle', signal: 'φ', valueKey: 'phi', unit: '°' },
      { label: 'Aileron', signal: 'δa', valueKey: 'da', unit: '°' },
    ],
  },
  LQServo: {
    formula: 'δc = −Kx + Ki∫(φc − φ)dt',
    controllerLabel: 'LQ SERVO / LQI',
    controllerValue: 'optimal state + integral feedback',
    summary: 'בקר אופטימלי עם אינטגרטור שמבטל שגיאת מצב מתמיד.',
    feedback: [
      { label: 'State vector', signal: 'x', valueKey: 'phi', unit: '° φ' },
      { label: 'Error integral', signal: 'ξ', valueKey: 'xi', unit: '' },
    ],
  },
};

const FLIGHT_MODEL = {
  massKg: 12000,
  wingAreaM2: 27.87,
  gravity: 9.80665,
  targetAirspeed: 220,
  targetAltitude: 3500,
  maxThrustN: 85000,
  cd0: 0.022,
  inducedDrag: 0.075,
};

const wrapRadians = angle => Math.atan2(Math.sin(angle), Math.cos(angle));

const quaternionFromFlightAngles = (bank, pitch, heading) => {
  // Aircraft axes in the scene: pitch=X, yaw=Y, roll=Z, forward=+Z.
  // Compose yaw * pitch * roll from the authoritative Euler angles instead
  // of integrating a second, drifting attitude state.
  const halfPitch = pitch / 2;
  const halfHeading = heading / 2;
  const halfRoll = -bank / 2;
  const sp = Math.sin(halfPitch);
  const cp = Math.cos(halfPitch);
  const sh = Math.sin(halfHeading);
  const ch = Math.cos(halfHeading);
  const sr = Math.sin(halfRoll);
  const cr = Math.cos(halfRoll);

  const yawPitch = {
    x: ch * sp,
    y: sh * cp,
    z: -sh * sp,
    w: ch * cp,
  };

  return normaliseQuaternion([
    yawPitch.x * cr + yawPitch.y * sr,
    -yawPitch.x * sr + yawPitch.y * cr,
    yawPitch.w * sr + yawPitch.z * cr,
    yawPitch.w * cr - yawPitch.z * sr,
  ]);
};

const createFlightState = (bank = 0, heading = 0) => {
  const pitch = 2.2 * D2R;
  const safeBank = Number.isFinite(bank) ? wrapRadians(bank) : 0;
  const safeHeading = Number.isFinite(heading) ? wrapRadians(heading) : 0;

  return {
  quaternion: quaternionFromFlightAngles(safeBank, pitch, safeHeading),
  pitch,
  pitchRate: 0,
  heading: safeHeading,
  altitude: FLIGHT_MODEL.targetAltitude,
  north: 0,
  east: 0,
  verticalSpeed: 0,
  airspeed: FLIGHT_MODEL.targetAirspeed,
  mach: 0.66,
  dynamicPressure: 0,
  loadFactor: 1,
  aoa: 2.2,
  turnRate: 0,
  thrust: 0,
  };
};

const normaliseQuaternion = quaternion => {
  const magnitude = Math.hypot(...quaternion) || 1;
  return quaternion.map(value => value / magnitude);
};

const integrateFlightPhysics = (flight, X, controlCommand, dt, gustInput) => {
  const [beta, p, r, phi, da] = X;
  const model = FLIGHT_MODEL;
  const safePhi = Number.isFinite(phi)
    ? clamp(wrapRadians(phi), -85 * D2R, 85 * D2R)
    : 0;

  const density = 1.225 * Math.exp(-Math.max(0, flight.altitude) / 8500);
  const speedOfSound = 340.3 - Math.min(45, flight.altitude * 0.003);
  const dynamicPressure = 0.5 * density * flight.airspeed * flight.airspeed;
  const bankCosine = Math.max(0.24, Math.cos(safePhi));
  const commandedLoad = clamp(1 / bankCosine, 0.35, 8.5);
  const liftCoefficient = clamp(
    commandedLoad * model.massKg * model.gravity /
      Math.max(1, dynamicPressure * model.wingAreaM2),
    -0.5,
    1.45
  );
  const dragCoefficient =
    model.cd0 +
    model.inducedDrag * liftCoefficient * liftCoefficient +
    0.004 * Math.abs(beta / D2R) +
    0.0015 * Math.abs(da / D2R);
  const drag = dynamicPressure * model.wingAreaM2 * dragCoefficient;
  const speedHold = model.massKg * 0.42 * (model.targetAirspeed - flight.airspeed);
  const thrust = clamp(drag + speedHold, 0, model.maxThrustN);
  const airspeedDot = (thrust - drag) / model.massKg;

  const altitudeError = model.targetAltitude - flight.altitude;
  const targetPitch = clamp(
    2.2 * D2R + altitudeError * 0.00016 - flight.verticalSpeed * 0.0025,
    -10 * D2R,
    12 * D2R
  );
  const pitchAcceleration =
    2.8 * (targetPitch - flight.pitch) -
    2.1 * flight.pitchRate +
    0.00004 * gustInput;
  const nextPitchRate = clamp(
    flight.pitchRate + pitchAcceleration * dt,
    -30 * D2R,
    30 * D2R
  );
  const nextPitch = clamp(
    flight.pitch + nextPitchRate * dt,
    -15 * D2R,
    15 * D2R
  );

  const coordinatedTurnRate =
    model.gravity * Math.tan(clamp(safePhi, -78 * D2R, 78 * D2R)) /
    Math.max(80, flight.airspeed);
  const yawRate = coordinatedTurnRate + 0.16 * r;
  const nextHeading = wrapRadians(flight.heading + yawRate * dt);
  const verticalSpeed =
    flight.airspeed * Math.sin(nextPitch) -
    0.12 * Math.abs(Math.sin(safePhi)) * flight.airspeed;
  const horizontalSpeed = flight.airspeed * Math.cos(nextPitch);

  // Keep the rendered attitude locked to the project model's roll angle.
  // This prevents quaternion drift, gimbal-looking flips and mode-switch jumps.
  flight.quaternion = quaternionFromFlightAngles(
    safePhi,
    nextPitch,
    nextHeading
  );
  flight.pitchRate = nextPitchRate;
  flight.pitch = nextPitch;
  flight.heading = nextHeading;
  flight.altitude = Math.max(80, flight.altitude + verticalSpeed * dt);
  flight.verticalSpeed = verticalSpeed;
  flight.airspeed = clamp(flight.airspeed + airspeedDot * dt, 95, 340);
  flight.north += horizontalSpeed * Math.cos(nextHeading) * dt;
  flight.east += horizontalSpeed * Math.sin(nextHeading) * dt;
  flight.mach = flight.airspeed / Math.max(280, speedOfSound);
  flight.dynamicPressure = dynamicPressure;
  flight.loadFactor = clamp(
    commandedLoad + 0.025 * Math.abs(controlCommand / D2R),
    0,
    9
  );
  flight.aoa = clamp(
    liftCoefficient * 7.6 + 0.16 * Math.abs(beta / D2R),
    -4,
    18
  );
  flight.turnRate = yawRate * R2D;
  flight.thrust = thrust;

  if (
    !flight.quaternion.every(Number.isFinite) ||
    !Number.isFinite(flight.pitch) ||
    !Number.isFinite(flight.heading) ||
    !Number.isFinite(flight.altitude) ||
    !Number.isFinite(flight.airspeed)
  ) {
    return createFlightState(safePhi, nextHeading);
  }

  return flight;
};

const computeRawControl = (X, phiCmdRad, ctrlType, measuredPhi = X[3]) => {
  const [beta, p, r, , da, xc, xi] = X;
  const error = phiCmdRad - measuredPhi;

  switch (ctrlType) {
    case 'P':
      return P_GAIN * error;

    case 'PD':
      return RATE_PHI_GAIN * error - RATE_P_GAIN * p;

    case 'Lead':
      return LEAD_GAIN * error - LEAD_STATE_GAIN * xc;

    case 'FullState':
      return STATE_NR * phiCmdRad - (
        STATE_K[0] * beta +
        STATE_K[1] * p +
        STATE_K[2] * r +
        STATE_K[3] * measuredPhi +
        STATE_K[4] * da
      );

    case 'LQServo':
      return (
        2.61682 * beta -
        0.13735 * p -
        0.54175 * r -
        0.70817 * measuredPhi -
        1.38814 * da +
        xi
      );

    default:
      return 0;
  }
};

const computeControl = (X, phiCmdRad, ctrlType, measuredPhi = X[3]) =>
  clamp(
    computeRawControl(X, phiCmdRad, ctrlType, measuredPhi),
    -MAX_AILERON_RAD,
    MAX_AILERON_RAD
  );

// Physics derivatives based on the final five-state aircraft model.
// X = [beta, p, r, phi, delta_a, x_lead, x_i]
const getDerivatives = (
  X,
  phiCmdRad,
  ctrlType,
  measurementNoiseRad,
  windGust
) => {
  const [beta, p, r, phi, da, xc] = X;

  // The same sampled measurement must be used throughout one RK4 step.
  const measuredPhi = phi + measurementNoiseRad;
  const error = phiCmdRad - measuredPhi;
  const rawDc = computeRawControl(X, phiCmdRad, ctrlType, measuredPhi);
  const dc = clamp(rawDc, -MAX_AILERON_RAD, MAX_AILERON_RAD);

  const dBeta = -0.575 * beta - r + 0.0536 * phi - 0.078 * da;
  const dP = -300 * beta - 3.03 * p + 2 * r + 64.4 * da + windGust;
  const dR = 68 * beta + 0.045 * p - 2.4 * r + 5 * da;
  const dPhi = p;
  const dDa = -5 * da + 5 * dc;

  // Controller internal states are active only for their own architecture.
  const dXc = ctrlType === 'Lead' ? -LEAD_POLE * xc + error : 0;
  const dXi =
    ctrlType === 'LQServo'
      ? error + LQ_ANTI_WINDUP_GAIN * (dc - rawDc)
      : 0;

  return [dBeta, dP, dR, dPhi, dDa, dXc, dXi];
};

const addVectors = (v1, v2) => v1.map((value, index) => value + v2[index]);
const scaleVector = (vector, scale) => vector.map(value => value * scale);

const rk4Step = (
  X,
  phiCmdRad,
  ctrlType,
  dt,
  noiseAmplitudeRad,
  windGust
) => {
  // Sample-and-hold sensor noise over the complete RK4 integration step.
  // Re-sampling at k1...k4 would inject artificial high-frequency energy.
  const noiseSample = (Math.random() - 0.5) * noiseAmplitudeRad;

  const derivative = state =>
    getDerivatives(
      state,
      phiCmdRad,
      ctrlType,
      noiseSample,
      windGust
    );

  const k1 = derivative(X);
  const k2 = derivative(addVectors(X, scaleVector(k1, dt / 2)));
  const k3 = derivative(addVectors(X, scaleVector(k2, dt / 2)));
  const k4 = derivative(addVectors(X, scaleVector(k3, dt)));

  return X.map(
    (value, index) =>
      value +
      (dt / 6) *
        (k1[index] + 2 * k2[index] + 2 * k3[index] + k4[index])
  );
};

const getControlEffort = (X, phiCmdRad, ctrlType) =>
  computeControl(X, phiCmdRad, ctrlType, X[3]);

const buildAdvancedAirplane = () => {
  const group = new THREE.Group();

  // Premium Materials
  const matFuselage = new THREE.MeshStandardMaterial({ 
    color: 0x475569, // Brighter stealth grey for clearer silhouette
    roughness: 0.28, 
    metalness: 0.72,
    emissive: 0x07111f,
    emissiveIntensity: 0.32,
  });
  
  const matNose = new THREE.MeshStandardMaterial({ 
    color: 0x263449, 
    roughness: 0.52,
    metalness: 0.45,
  });
  
  const matCanopy = new THREE.MeshPhysicalMaterial({ 
    color: 0xf59e0b, // Amber/Gold tinted canopy
    metalness: 0.9, 
    roughness: 0.05, 
    transparent: true, 
    opacity: 0.6,
    clearcoat: 1.0,
    clearcoatRoughness: 0.1
  });
  
  const matControlSurface = new THREE.MeshStandardMaterial({ 
    color: 0x64748b, 
    roughness: 0.3,
    metalness: 0.65,
    emissive: 0x071523,
    emissiveIntensity: 0.25,
  });

  // Main Fuselage
  const fuseGeom = new THREE.CylinderGeometry(0.2, 0.45, 4.5, 32);
  fuseGeom.rotateX(Math.PI / 2);
  const fuselage = new THREE.Mesh(fuseGeom, matFuselage);
  group.add(fuselage);

  // Nose Cone
  const noseGeom = new THREE.ConeGeometry(0.2, 1.4, 32);
  noseGeom.rotateX(Math.PI / 2);
  noseGeom.translate(0, 0, 2.9);
  const nose = new THREE.Mesh(noseGeom, matNose);
  group.add(nose);

  // Canopy
  const canopyGeom = new THREE.CapsuleGeometry(0.25, 1.0, 16, 16);
  canopyGeom.rotateX(Math.PI / 2);
  canopyGeom.translate(0, 0.35, 0.8);
  const canopy = new THREE.Mesh(canopyGeom, matCanopy);
  group.add(canopy);

  // Swept Main Wings
  const wingShape = new THREE.Shape();
  wingShape.moveTo(0, 0);
  wingShape.lineTo(3.2, -1.2);
  wingShape.lineTo(3.2, -2.0);
  wingShape.lineTo(0, -1.4);
  
  const extrudeSettings = { depth: 0.06, bevelEnabled: true, bevelSegments: 2, steps: 1, bevelSize: 0.02, bevelThickness: 0.02 };
  
  const rightWingGeom = new THREE.ExtrudeGeometry(wingShape, extrudeSettings);
  rightWingGeom.rotateX(Math.PI / 2);
  const rightWing = new THREE.Mesh(rightWingGeom, matFuselage);
  group.add(rightWing);

  const leftWingGeom = rightWingGeom.clone();
  leftWingGeom.rotateZ(Math.PI);
  const leftWing = new THREE.Mesh(leftWingGeom, matFuselage);
  group.add(leftWing);

  // Ailerons (Animated parts)
  const aileronGeom = new THREE.BoxGeometry(1.4, 0.08, 0.5);
  aileronGeom.translate(0, 0, -0.25); // pivot point
  
  const rightAileron = new THREE.Mesh(aileronGeom, matControlSurface);
  rightAileron.position.set(2.0, 0, -2.0);
  group.add(rightAileron);

  const leftAileron = new THREE.Mesh(aileronGeom, matControlSurface);
  leftAileron.position.set(-2.0, 0, -2.0);
  group.add(leftAileron);

  // Horizontal stabilators
  const stabilatorGeom = new THREE.BoxGeometry(1.65, 0.07, 0.58);
  const rightStabilator = new THREE.Mesh(stabilatorGeom, matControlSurface);
  rightStabilator.position.set(0.95, 0.02, -2.05);
  rightStabilator.rotation.y = -0.14;
  group.add(rightStabilator);

  const leftStabilator = rightStabilator.clone();
  leftStabilator.position.x = -0.95;
  leftStabilator.rotation.y = 0.14;
  group.add(leftStabilator);

  // Twin side intakes give the model a more credible modern-fighter silhouette.
  const intakeGeom = new THREE.BoxGeometry(0.34, 0.38, 1.25);
  const intakeMaterial = new THREE.MeshStandardMaterial({
    color: 0x172033,
    roughness: 0.42,
    metalness: 0.55,
  });
  const rightIntake = new THREE.Mesh(intakeGeom, intakeMaterial);
  rightIntake.position.set(0.46, -0.18, 0.35);
  group.add(rightIntake);
  const leftIntake = rightIntake.clone();
  leftIntake.position.x = -0.46;
  group.add(leftIntake);

  // Vertical Tail
  const vTailShape = new THREE.Shape();
  vTailShape.moveTo(0,0);
  vTailShape.lineTo(0, 1.4);
  vTailShape.lineTo(-0.8, 1.4);
  vTailShape.lineTo(-1.5, 0);
  const vTailGeom = new THREE.ExtrudeGeometry(vTailShape, extrudeSettings);
  vTailGeom.rotateY(-Math.PI/2);
  vTailGeom.translate(-0.03, 0.25, -1.2);
  const vTail = new THREE.Mesh(vTailGeom, matFuselage);
  group.add(vTail);

  // Engine exhaust glow
  const engineGeom = new THREE.CylinderGeometry(0.35, 0.25, 0.6, 16);
  engineGeom.rotateX(Math.PI/2);
  engineGeom.translate(0, 0, -2.5);
  const matEngine = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });
  const engine = new THREE.Mesh(engineGeom, matEngine);
  group.add(engine);
  
  // Inner Afterburner
  const abGeom = new THREE.CylinderGeometry(0.2, 0.05, 1.5, 16);
  abGeom.rotateX(Math.PI/2);
  abGeom.translate(0, 0, -3.0);
  const matAb = new THREE.MeshBasicMaterial({ color: 0xbae6fd, transparent: true, opacity: 0.8 });
  const afterburner = new THREE.Mesh(abGeom, matAb);
  group.add(afterburner);

  // Navigation lights and wing-tip strobes.
  const navRed = new THREE.PointLight(0xff334f, 2.4, 5);
  navRed.position.set(-3.15, 0.08, -1.45);
  group.add(navRed);
  const navGreen = new THREE.PointLight(0x22c55e, 2.4, 5);
  navGreen.position.set(3.15, 0.08, -1.45);
  group.add(navGreen);

  const wingTipGeometry = new THREE.SphereGeometry(0.055, 12, 12);
  const redTip = new THREE.Mesh(
    wingTipGeometry,
    new THREE.MeshBasicMaterial({ color: 0xff334f })
  );
  redTip.position.copy(navRed.position);
  group.add(redTip);
  const greenTip = new THREE.Mesh(
    wingTipGeometry,
    new THREE.MeshBasicMaterial({ color: 0x22c55e })
  );
  greenTip.position.copy(navGreen.position);
  group.add(greenTip);

  return {
    airplaneGroup: group,
    leftAileron,
    rightAileron,
    rightStabilator,
    leftStabilator,
    engine,
    afterburner,
    navRed,
    navGreen,
  };
};

const drawOscilloscope = (canvas, history, key1, key2, color1, color2, min, max, label1, label2, showLimits = false) => {
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  
  // Clear with transparent background to let CSS show through
  ctx.clearRect(0, 0, w, h);
  // The app shell is RTL, but telemetry labels and axes are technical LTR data.
  ctx.direction = 'ltr';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'alphabetic';
  
  // Cyber grid
  ctx.strokeStyle = 'rgba(30, 41, 59, 0.5)'; // slate-800
  ctx.lineWidth = 1;
  ctx.beginPath();
  for(let i=1; i<5; i++) {
    const y = i * (h / 5);
    ctx.moveTo(0, y); ctx.lineTo(w, y);
  }
  for(let i=1; i<10; i++) {
    const x = i * (w / 10);
    ctx.moveTo(x, 0); ctx.lineTo(x, h);
  }
  ctx.stroke();

  // Zero line
  const zeroY = h - ((0 - min) / (max - min)) * h;
  ctx.strokeStyle = 'rgba(100, 116, 139, 0.6)'; // slate-500
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(0, zeroY); ctx.lineTo(w, zeroY);
  ctx.stroke();

  // Limits
  if (showLimits) {
    const limitUp = h - ((5 - min) / (max - min)) * h;
    const limitDown = h - ((-5 - min) / (max - min)) * h;
    ctx.strokeStyle = 'rgba(244, 63, 94, 0.6)'; // rose-500
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(0, limitUp); ctx.lineTo(w, limitUp);
    ctx.moveTo(0, limitDown); ctx.lineTo(w, limitDown);
    ctx.stroke();
    ctx.setLineDash([]);
    
    ctx.fillStyle = 'rgba(244, 63, 94, 0.9)';
    ctx.font = '10px monospace';
    ctx.fillText('MAX +5°', w - 60, limitUp - 5);
    ctx.fillText('MIN -5°', w - 60, limitDown + 12);
  }

  if (history.length < 2) return;

  const dx = w / Math.max(1, history.length - 1); 

  const drawLine = (key, color, glow, isDash, fillGradient = false) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.shadowBlur = glow ? 12 : 0;
    ctx.shadowColor = color;
    if(isDash) ctx.setLineDash([4, 4]);
    
    ctx.beginPath();
    history.forEach((pt, i) => {
      const x = i * dx;
      const val = pt[key] !== undefined ? pt[key] : 0;
      const y = h - ((val - min) / (max - min)) * h;
      if(i===0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
    
    // Add gradient fill under the curve
    if (fillGradient) {
      ctx.lineTo((history.length - 1) * dx, zeroY);
      ctx.lineTo(0, zeroY);
      ctx.closePath();
      
      const grad = ctx.createLinearGradient(0, 0, 0, h);
      // Extract RGB from hex/rgb string roughly (assuming standard hex)
      grad.addColorStop(0, color + '40'); // 25% opacity
      grad.addColorStop(1, color + '00'); // 0% opacity
      
      ctx.fillStyle = grad;
      ctx.shadowBlur = 0; // Turn off shadow for fill
      ctx.fill();
    }
    
    ctx.setLineDash([]);
    ctx.shadowBlur = 0;
  };

  // Draw lines
  drawLine(key2, color2, false, true, false); // Command (dashed)
  drawLine(key1, color1, true, false, true); // Actual (solid, glowing, filled)

  // Legends
  ctx.shadowBlur = 8;
  ctx.shadowColor = color1;
  ctx.fillStyle = '#f8fafc'; // slate-50
  ctx.font = 'bold 12px Inter, sans-serif';
  ctx.fillText(label1, 10, 20);
  
  ctx.shadowBlur = 0;
  ctx.fillStyle = color2;
  ctx.fillText(label2, 10, 36);
};

const AttitudeIndicator = ({ roll, pitch = 0 }) => {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const radius = Math.min(w, h) / 2 - 4; // slight padding

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, w, h);
    
    // Outer Bezel Shadow
    ctx.shadowBlur = 15;
    ctx.shadowColor = 'rgba(0,0,0,0.8)';
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, 2*Math.PI);
    ctx.fillStyle = '#020617';
    ctx.fill();
    ctx.shadowBlur = 0;

    ctx.save();
    
    // Clip to circle
    ctx.beginPath();
    ctx.arc(cx, cy, radius - 2, 0, 2*Math.PI);
    ctx.clip();
    
    // Translate and rotate for roll
    ctx.translate(cx, cy);
    ctx.rotate(-roll * D2R); 

    // Sky Gradient
    const skyGrad = ctx.createLinearGradient(0, -h, 0, pitch * 2);
    skyGrad.addColorStop(0, '#0284c7'); // sky-600
    skyGrad.addColorStop(1, '#7dd3fc'); // sky-300
    ctx.fillStyle = skyGrad;
    ctx.fillRect(-w, -h, w*2, h + pitch * 2);

    // Ground Gradient
    const groundGrad = ctx.createLinearGradient(0, pitch * 2, 0, h);
    groundGrad.addColorStop(0, '#854d0e'); // yellow-800
    groundGrad.addColorStop(1, '#422006'); // deeply dark brown
    ctx.fillStyle = groundGrad;
    ctx.fillRect(-w, pitch * 2, w*2, h);

    // Horizon line
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(-w, pitch * 2);
    ctx.lineTo(w, pitch * 2);
    ctx.stroke();

    // Pitch ladder (more realistic)
    ctx.strokeStyle = 'rgba(255,255,255,0.8)';
    ctx.fillStyle = 'rgba(255,255,255,0.8)';
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    
    for(let i=10; i<=40; i+=10) {
      const pyUp = pitch * 2 - i * 3;
      const pyDown = pitch * 2 + i * 3;
      
      // Up pitch (sky)
      ctx.beginPath(); ctx.moveTo(-15, pyUp); ctx.lineTo(15, pyUp); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(-15, pyUp); ctx.lineTo(-15, pyUp+4); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(15, pyUp); ctx.lineTo(15, pyUp+4); ctx.stroke();
      ctx.fillText(i, -25, pyUp); ctx.fillText(i, 25, pyUp);
      
      // Down pitch (ground) - dashed
      ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(-15, pyDown); ctx.lineTo(15, pyDown); ctx.stroke();
      ctx.setLineDash([]);
      ctx.beginPath(); ctx.moveTo(-15, pyDown); ctx.lineTo(-15, pyDown-4); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(15, pyDown); ctx.lineTo(15, pyDown-4); ctx.stroke();
      ctx.fillText(i, -25, pyDown); ctx.fillText(i, 25, pyDown);
    }

    ctx.restore();

    // Inner Bezel highlight
    ctx.beginPath();
    ctx.arc(cx, cy, radius - 2, 0, 2*Math.PI);
    ctx.lineWidth = 4;
    const bezelGrad = ctx.createLinearGradient(0,0,w,h);
    bezelGrad.addColorStop(0, '#334155');
    bezelGrad.addColorStop(1, '#0f172a');
    ctx.strokeStyle = bezelGrad;
    ctx.stroke();

    // Fixed aircraft reference (amber)
    ctx.strokeStyle = '#f59e0b'; // Amber-500
    ctx.lineWidth = 3.5;
    ctx.beginPath();
    // Left wing
    ctx.moveTo(cx - 50, cy); ctx.lineTo(cx - 15, cy); ctx.lineTo(cx-15, cy+8);
    // Right wing
    ctx.moveTo(cx + 50, cy); ctx.lineTo(cx + 15, cy); ctx.lineTo(cx+15, cy+8);
    ctx.stroke();
    // Center dot
    ctx.fillStyle = '#f59e0b';
    ctx.fillRect(cx-2, cy-2, 4, 4);

    // Roll markings (top arc)
    ctx.save();
    ctx.translate(cx, cy);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    const angles = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60];
    angles.forEach(deg => {
      ctx.save();
      ctx.rotate(deg * D2R);
      ctx.beginPath();
      ctx.moveTo(0, -radius + 4);
      // Thicker/longer line for 0, 30, 60
      const isMajor = deg % 30 === 0 || deg === 0;
      ctx.lineTo(0, -radius + (isMajor ? 14 : 8));
      ctx.stroke();
      ctx.restore();
    });
    
    // Top Roll Pointer (Fixed triangle)
    ctx.fillStyle = '#f59e0b';
    ctx.beginPath();
    ctx.moveTo(0, -radius + 14);
    ctx.lineTo(-6, -radius + 24);
    ctx.lineTo(6, -radius + 24);
    ctx.fill();
    ctx.restore();

  }, [roll, pitch]);

  return (
    <div className="relative flex flex-col items-center">
      <canvas ref={canvasRef} width="180" height="180" className="rounded-full shadow-[0_0_25px_rgba(0,0,0,0.6)] border-4 border-slate-900" />
      <div className="absolute -bottom-3 bg-slate-900 border border-slate-700 px-3 py-1 rounded-md font-mono text-sm text-sky-400 shadow-lg">
        {Math.abs(roll).toFixed(1)}° {roll > 0 ? 'R' : roll < 0 ? 'L' : ''}
      </div>
    </div>
  );
};

const FlowLink = ({ label, color, paused, reverse = false }) => (
  <div
    className={`control-flow-link ${reverse ? 'is-reverse' : ''} ${paused ? 'is-paused' : ''}`}
    style={{ '--flow-color': color }}
    aria-hidden="true"
  >
    {label && <span className="control-flow-label">{label}</span>}
    <span className="control-flow-pulse" />
  </div>
);

const DiagramNode = ({ eyebrow, title, value, color, compact = false }) => (
  <div
    className={`control-diagram-node ${compact ? 'is-compact' : ''}`}
    style={{ '--node-color': color }}
  >
    <span className="control-node-eyebrow">{eyebrow}</span>
    <strong>{title}</strong>
    <span className="control-node-value" dir="ltr">{value}</span>
  </div>
);

const LiveControlArchitecture = ({ controller, cmdDeg, stats, paused }) => {
  const info = CONTROLLERS[controller];
  const architecture = CONTROL_ARCHITECTURES[controller];
  const activity = clamp(Math.abs(stats.dc) / 5, 0.18, 1);

  return (
    <section
      className="control-architecture-panel"
      style={{
        '--controller-color': info.color,
        '--controller-glow': info.glow,
        '--flow-duration': `${(1.65 - activity).toFixed(2)}s`,
      }}
      aria-labelledby="control-architecture-title"
    >
      <div className="control-architecture-header">
        <div>
          <span className="control-section-kicker">LIVE CONTROL ARCHITECTURE</span>
          <h2 id="control-architecture-title">דיאגרמת חוג הבקרה הפעיל</h2>
          <p>{architecture.summary}</p>
        </div>
        <div className="control-formula-card" dir="ltr">
          <span className={`control-live-dot ${paused ? 'is-paused' : ''}`} />
          <div>
            <span>{paused ? 'FLOW PAUSED' : 'LIVE SIGNAL FLOW'}</span>
            <strong>{architecture.formula}</strong>
          </div>
        </div>
      </div>

      <div className="control-diagram-scroll" dir="ltr">
        <div className="control-diagram-stage">
          <div className="control-scan-line" />
          <div className="control-main-loop">
            <DiagramNode
              eyebrow="REFERENCE"
              title="Roll command"
              value={`φc = ${cmdDeg.toFixed(1)}°`}
              color={info.color}
              compact
            />
            <FlowLink label="φc" color={info.color} paused={paused} />
            <DiagramNode
              eyebrow="ERROR JUNCTION"
              title="Σ"
              value={`e = ${stats.ess.toFixed(2)}°`}
              color="#fbbf24"
              compact
            />
            <FlowLink label="e(t)" color="#fbbf24" paused={paused} />
            <DiagramNode
              eyebrow={architecture.controllerLabel}
              title={info.name}
              value={architecture.controllerValue}
              color={info.color}
            />
            <FlowLink label={`δc ${stats.dc.toFixed(2)}°`} color="#e879f9" paused={paused} />
            <DiagramNode
              eyebrow="ACTUATOR"
              title="Aileron servo"
              value={`δa = ${stats.da.toFixed(2)}°`}
              color="#fb7185"
            />
            <FlowLink label="δa" color="#fb7185" paused={paused} />
            <DiagramNode
              eyebrow="AIRCRAFT"
              title="Lateral dynamics"
              value={`β ${stats.beta.toFixed(2)}° · p ${stats.p.toFixed(2)}°/s`}
              color="#38bdf8"
            />
            <FlowLink label="φ" color="#38bdf8" paused={paused} />
            <DiagramNode
              eyebrow="OUTPUT"
              title="Roll angle"
              value={`φ = ${stats.phi.toFixed(2)}°`}
              color="#34d399"
              compact
            />
          </div>

          <div className="control-feedback-zone">
            <div className="control-feedback-return">
              <span className="control-return-corner" />
              <FlowLink label="measured feedback" color={info.color} paused={paused} reverse />
              <span className="control-return-corner is-left" />
            </div>
            <div className="control-feedback-cards">
              {architecture.feedback.map(item => (
                <div className="control-feedback-chip" key={`${controller}-${item.signal}`}>
                  <span>{item.label}</span>
                  <strong dir="ltr">
                    {item.signal} = {Number(stats[item.valueKey] || 0).toFixed(2)}{item.unit}
                  </strong>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>

      <div className="control-architecture-footer">
        <span><i style={{ background: info.color }} /> Active controller: <strong>{info.name}</strong></span>
        <span dir="ltr">|e| {Math.abs(stats.ess).toFixed(2)}°</span>
        <span dir="ltr">|δa|max {stats.maxDa.toFixed(2)}°</span>
        <span dir="ltr">t = {stats.time.toFixed(2)} s</span>
      </div>
    </section>
  );
};

export default function AdvancedAircraftSimulation() {
  const mountRef = useRef(null);
  const chartRollRef = useRef(null);
  const chartAileronRef = useRef(null);
  
  const [controller, setController] = useState('LQServo');
  const [cmdDeg, setCmdDeg] = useState(30);
  const [paused, setPaused] = useState(false);
  const [stats, setStats] = useState({
    phi: 0,
    p: 0,
    beta: 0,
    da: 0,
    dc: 0,
    ess: 30,
    time: 0,
    maxDa: 0,
    maxDc: 0,
    xc: 0,
    xi: 0,
    airspeed: FLIGHT_MODEL.targetAirspeed,
    mach: 0.66,
    altitude: FLIGHT_MODEL.targetAltitude,
    heading: 0,
    gLoad: 1,
    aoa: 2.2,
    pitch: 2.2,
    verticalSpeed: 0,
    turnRate: 0,
  });
  const [noiseLvl, setNoiseLvl] = useState(0);
  const [simSpeed, setSimSpeed] = useState(1);
  const [physicsMode, setPhysicsMode] = useState('advanced');
  const [showAdvancedHud, setShowAdvancedHud] = useState(true);
  const [showEnvironmentFx, setShowEnvironmentFx] = useState(true);
  const [showControlDiagram, setShowControlDiagram] = useState(true);

  const sim = useRef({
    X: [0, 0, 0, 0, 0, 0, 0], 
    phi_cmd_rad: 30 * D2R,
    ctrlType: 'LQServo',
    history: [], 
    time: 0,
    windGust: 0,
    maxDa: 0,
    maxDc: 0,
    flight: createFlightState(),
  });

  const sceneRefs = useRef({});
  const requestRef = useRef();
  const lastTimeRef = useRef();

  // Smooth camera control
  const targetRotation = useRef({ x: 0, y: Math.PI / 8 }); // slight isometric start
  const currentRotation = useRef({ x: 0, y: Math.PI / 8 });
  const [isDragging, setIsDragging] = useState(false);
  const dragStart = useRef({ x: 0, y: 0 });

  useEffect(() => {
    if (!mountRef.current) return;

    const width = Math.max(1, mountRef.current.clientWidth);
    const height = Math.max(1, mountRef.current.clientHeight);

    const scene = new THREE.Scene();
    // Transparent background to let React UI show through
    scene.background = null; 
    scene.fog = new THREE.FogExp2(0x020617, 0.03);

    const camera = new THREE.PerspectiveCamera(45, width/height, 0.1, 100);
    camera.position.set(0, 0, 10.4);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    mountRef.current.appendChild(renderer.domElement);

    // Advanced Lighting
    const hemiLight = new THREE.HemisphereLight(0xdbeafe, 0x172033, 1.15);
    scene.add(hemiLight);
    
    const dirLight = new THREE.DirectionalLight(0xffffff, 2.7);
    dirLight.position.set(5, 10, 5);
    scene.add(dirLight);

    const fillLight = new THREE.DirectionalLight(0x93c5fd, 1.5);
    fillLight.position.set(-5, 2, 7);
    scene.add(fillLight);
    
    // Tech-blue rim light
    const rimLight = new THREE.SpotLight(0x38bdf8, 7);
    rimLight.position.set(-10, 5, -10);
    rimLight.lookAt(0,0,0);
    scene.add(rimLight);

    // Subtle 3D Grid
    const grid = new THREE.GridHelper(60, 60, 0x0ea5e9, 0x1e293b);
    grid.position.y = -3.5;
    // Fade out grid in distance
    grid.material.transparent = true;
    grid.material.opacity = 0.3;
    scene.add(grid);

    // Speed Lines (Particles)
    const particleCount = 400;
    const posArray = new Float32Array(particleCount * 3);
    for(let i=0; i<particleCount*3; i+=3) {
       posArray[i] = (Math.random() - 0.5) * 30; // x
       posArray[i+1] = (Math.random() - 0.5) * 20; // y
       posArray[i+2] = (Math.random() - 0.5) * 40; // z
    }
    const particlesGeo = new THREE.BufferGeometry();
    particlesGeo.setAttribute('position', new THREE.BufferAttribute(posArray, 3));
    // Streaked points
    const particlesMat = new THREE.PointsMaterial({
      color: 0x38bdf8, 
      size: 0.08, 
      transparent: true, 
      opacity: 0.5,
      blending: THREE.AdditiveBlending
    });
    const particles = new THREE.Points(particlesGeo, particlesMat);
    scene.add(particles);

    const {
      airplaneGroup,
      leftAileron,
      rightAileron,
      rightStabilator,
      leftStabilator,
      engine,
      afterburner,
      navRed,
      navGreen,
    } = buildAdvancedAirplane();
    
    const rotationWrapper = new THREE.Group();
    rotationWrapper.add(airplaneGroup);
    scene.add(rotationWrapper);

    sceneRefs.current = {
      scene,
      camera,
      renderer,
      airplaneGroup,
      leftAileron,
      rightAileron,
      rightStabilator,
      leftStabilator,
      engine,
      afterburner,
      navRed,
      navGreen,
      grid,
      particles,
      rotationWrapper,
    };

    const handleResize = () => {
      if(!mountRef.current) return;
      const w = Math.max(1, mountRef.current.clientWidth);
      const h = Math.max(1, mountRef.current.clientHeight);
      renderer.setSize(w, h);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    window.addEventListener('resize', handleResize);

    return () => {
      window.removeEventListener('resize', handleResize);

      scene.traverse(object => {
        const mesh = object;
        if (mesh.geometry) mesh.geometry.dispose?.();

        const materials = Array.isArray(mesh.material)
          ? mesh.material
          : mesh.material
            ? [mesh.material]
            : [];

        materials.forEach(material => {
          Object.values(material).forEach(value => {
            if (value && typeof value === 'object' && value.isTexture) {
              value.dispose?.();
            }
          });
          material.dispose?.();
        });
      });

      if (mountRef.current?.contains(renderer.domElement)) {
        mountRef.current.removeChild(renderer.domElement);
      }

      renderer.dispose();
      renderer.forceContextLoss();
    };
  }, []);

  useEffect(() => {
    let lastUiUpdate = 0;
    const fixedDt = 0.01; 

    const animate = (time) => {
      if (lastTimeRef.current != undefined && !paused) {
        let dt = ((time - lastTimeRef.current) / 1000) * simSpeed;
        dt = Math.min(dt, 0.05); 
        
        const steps = Math.max(1, Math.ceil(dt / fixedDt));
        const integrationDt = dt / steps;

        for (let i = 0; i < steps; i++) {
          sim.current.X = rk4Step(
            sim.current.X,
            sim.current.phi_cmd_rad,
            sim.current.ctrlType,
            integrationDt,
            noiseLvl * D2R,
            sim.current.windGust
          );

          if (physicsMode === 'advanced') {
            sim.current.flight = integrateFlightPhysics(
              sim.current.flight,
              sim.current.X,
              getControlEffort(
                sim.current.X,
                sim.current.phi_cmd_rad,
                sim.current.ctrlType
              ),
              integrationDt,
              sim.current.windGust
            );
          }

          // Time-based gust decay, independent of monitor frame rate.
          sim.current.windGust *= Math.exp(-3 * integrationDt);
          sim.current.time += integrationDt;
        }

        const currentPhi = sim.current.X[3] * R2D;
        const currentDa = sim.current.X[4] * R2D;
        const currentDc = getControlEffort(sim.current.X, sim.current.phi_cmd_rad, sim.current.ctrlType) * R2D;
        const targetPhi = sim.current.phi_cmd_rad * R2D;
        const currentP = sim.current.X[1] * R2D;
        const currentBeta = sim.current.X[0] * R2D;

        sim.current.maxDa = Math.max(sim.current.maxDa, Math.abs(currentDa));
        sim.current.maxDc = Math.max(sim.current.maxDc, Math.abs(currentDc));
        sim.current.history.push({
          t: sim.current.time,
          phi: currentPhi,
          phic: targetPhi,
          p: currentP,
          beta: currentBeta,
          da: currentDa,
          dc: currentDc,
          airspeed: sim.current.flight.airspeed,
          altitude: sim.current.flight.altitude,
          mach: sim.current.flight.mach,
          gLoad: sim.current.flight.loadFactor,
        });
        if (sim.current.history.length > 600) sim.current.history.shift();

        const {
          airplaneGroup,
          leftAileron,
          rightAileron,
          rightStabilator,
          leftStabilator,
          engine,
          afterburner,
          navRed,
          navGreen,
          grid,
          particles,
          renderer,
          scene,
          camera,
          rotationWrapper,
        } = sceneRefs.current;
        
        if (airplaneGroup && leftAileron && rightAileron && engine && afterburner && grid && particles) {
          if (physicsMode === 'advanced') {
            const [qx, qy, qz, qw] = sim.current.flight.quaternion;
            airplaneGroup.quaternion.set(qx, qy, qz, qw);
          } else {
            // Project mode mirrors the submitted five-state model directly.
            airplaneGroup.rotation.set(
              Math.abs(sim.current.X[3]) * 0.05,
              sim.current.X[0] * 3,
              -sim.current.X[3]
            );
          }
          
          leftAileron.rotation.x = sim.current.X[4];
          rightAileron.rotation.x = -sim.current.X[4];
          if (leftStabilator && rightStabilator) {
            const stabilatorAngle =
              physicsMode === 'advanced'
                ? clamp(-sim.current.flight.pitchRate * 0.35, -0.28, 0.28)
                : 0;
            leftStabilator.rotation.x = stabilatorAngle;
            rightStabilator.rotation.x = stabilatorAngle;
          }

          // Engine flicker based on time and noise
          const flicker = 0.8 + Math.random() * 0.2;
          engine.material.color.setHex(Math.random() > 0.5 ? 0x0ea5e9 : 0x38bdf8);
          const thrustRatio =
            physicsMode === 'advanced'
              ? sim.current.flight.thrust / FLIGHT_MODEL.maxThrustN
              : 0.72 + Math.abs(currentDa) * 0.02;
          afterburner.scale.set(1, 1, 0.65 + flicker * clamp(thrustRatio, 0.2, 1.15));
          afterburner.material.opacity = 0.34 + 0.42 * clamp(thrustRatio, 0, 1);
          if (navRed && navGreen) {
            const strobe = Math.sin(sim.current.time * 7.5) > 0.82 ? 4.6 : 1.6;
            navRed.intensity = showEnvironmentFx ? strobe : 0;
            navGreen.intensity = showEnvironmentFx ? strobe : 0;
          }

          // Move Grid
          grid.position.z = (sim.current.time * 15) % 2;
          grid.visible = showEnvironmentFx;
          particles.visible = showEnvironmentFx;
          
          // Animate Speed Lines (Particles)
          const positions = particles.geometry.attributes.position.array;
          for(let i=2; i<positions.length; i+=3) {
              positions[i] += 1.5; // move Z fast
              if(positions[i] > 20) {
                  positions[i] = -20; // wrap around
                  positions[i-1] = (Math.random() - 0.5) * 20; // new Y
                  positions[i-2] = (Math.random() - 0.5) * 30; // new X
              }
          }
          particles.geometry.attributes.position.needsUpdate = true;
        }

        // Smooth Camera Lerping
        currentRotation.current.x += (targetRotation.current.x - currentRotation.current.x) * 0.1;
        currentRotation.current.y += (targetRotation.current.y - currentRotation.current.y) * 0.1;
        
        if(rotationWrapper) {
          rotationWrapper.rotation.y = currentRotation.current.x;
          rotationWrapper.rotation.x = currentRotation.current.y;
        }

        if(renderer && scene && camera) renderer.render(scene, camera);

        // Update Canvas Charts
        if(chartRollRef.current) {
          const color = CONTROLLERS[sim.current.ctrlType].color;
          drawOscilloscope(chartRollRef.current, sim.current.history, 'phi', 'phic', color, '#cbd5e1', -15, 45, 'ROLL ANGLE (ACTUAL)', 'COMMAND (\u03D5_c)');
        }
        if(chartAileronRef.current) {
          drawOscilloscope(chartAileronRef.current, sim.current.history, 'da', 'dc', '#f43f5e', '#fbbf24', -8, 8, 'AILERON DEFLECTION', 'SERVO CMD', true);
        }

        // Throttle React State updates
        if (time - lastUiUpdate > 100) {
          setStats({
            phi: currentPhi,
            p: currentP,
            beta: currentBeta,
            da: currentDa,
            dc: currentDc,
            ess: targetPhi - currentPhi,
            time: sim.current.time,
            maxDa: sim.current.maxDa,
            maxDc: sim.current.maxDc,
            xc: sim.current.X[5],
            xi: sim.current.X[6],
            airspeed: sim.current.flight.airspeed,
            mach: sim.current.flight.mach,
            altitude: sim.current.flight.altitude,
            heading: ((sim.current.flight.heading * R2D) % 360 + 360) % 360,
            gLoad: sim.current.flight.loadFactor,
            aoa: sim.current.flight.aoa,
            pitch: sim.current.flight.pitch * R2D,
            verticalSpeed: sim.current.flight.verticalSpeed,
            turnRate: sim.current.flight.turnRate,
          });
          lastUiUpdate = time;
        }
      }
      
      lastTimeRef.current = time;
      requestRef.current = requestAnimationFrame(animate);
    };
    
    requestRef.current = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(requestRef.current);
  }, [paused, noiseLvl, simSpeed, physicsMode, showEnvironmentFx]);

  const triggerGust = () => {
    sim.current.windGust = 80; // Short stress-test disturbance in p-dot
  };

  const resetSim = () => {
    sim.current.X = [0,0,0,0,0,0,0];
    sim.current.history = [];
    sim.current.time = 0;
    sim.current.windGust = 0;
    sim.current.maxDa = 0;
    sim.current.maxDc = 0;
    sim.current.flight = createFlightState();
    setStats({
      phi: 0,
      p: 0,
      beta: 0,
      da: 0,
      dc: 0,
      ess: sim.current.phi_cmd_rad * R2D,
      time: 0,
      maxDa: 0,
      maxDc: 0,
      xc: 0,
      xi: 0,
      airspeed: FLIGHT_MODEL.targetAirspeed,
      mach: 0.66,
      altitude: FLIGHT_MODEL.targetAltitude,
      heading: 0,
      gLoad: 1,
      aoa: 2.2,
      pitch: 2.2,
      verticalSpeed: 0,
      turnRate: 0,
    });
    targetRotation.current = { x: 0, y: Math.PI / 8 };
    currentRotation.current = { x: 0, y: Math.PI / 8 };
    lastTimeRef.current = undefined;
  };

  const handleControllerChange = (c) => {
    setController(c);
    sim.current.ctrlType = c;
    // Each architecture starts from the same clean initial condition.
    // This avoids hidden controller memory and non-physical switching spikes.
    resetSim();
  };

  // 3D Canvas Mouse Events
  const handleMouseDown = (e) => { setIsDragging(true); dragStart.current = { x: e.clientX, y: e.clientY }; };
  const handleMouseUp = () => { setIsDragging(false); };
  const handleMouseMove = (e) => {
    if (!isDragging) return;
    const dx = e.clientX - dragStart.current.x;
    const dy = e.clientY - dragStart.current.y;
    targetRotation.current = {
      x: targetRotation.current.x + dx * 0.005,
      y: Math.max(-Math.PI/3, Math.min(Math.PI/3, targetRotation.current.y + dy * 0.005))
    };
    dragStart.current = { x: e.clientX, y: e.clientY };
  };
  const handleCommandChange = (deg) => {
    sim.current.phi_cmd_rad = deg * D2R;
  };

  const handlePhysicsModeChange = mode => {
    setPhysicsMode(mode);
    sim.current.flight = createFlightState(
      sim.current.X[3],
      sim.current.flight?.heading ?? 0
    );
    lastTimeRef.current = undefined;
  };

  const exportTelemetry = () => {
    if (!sim.current.history.length) return;

    const rows = [
      [
        'time_s',
        'controller',
        'physics_mode',
        'phi_cmd_deg',
        'phi_deg',
        'p_deg_s',
        'beta_deg',
        'delta_a_deg',
        'delta_c_deg',
        'airspeed_m_s',
        'altitude_m',
        'mach',
        'g_load',
      ],
      ...sim.current.history.map(point => [
        point.t.toFixed(4),
        sim.current.ctrlType,
        physicsMode,
        point.phic.toFixed(5),
        point.phi.toFixed(5),
        point.p.toFixed(5),
        point.beta.toFixed(5),
        point.da.toFixed(5),
        point.dc.toFixed(5),
        point.airspeed.toFixed(4),
        point.altitude.toFixed(4),
        point.mach.toFixed(5),
        point.gLoad.toFixed(5),
      ]),
    ];
    const csv = rows.map(row => row.join(',')).join('\n');
    const url = URL.createObjectURL(new Blob([csv], { type: 'text/csv;charset=utf-8' }));
    const anchor = document.createElement('a');
    anchor.href = url;
    anchor.download = `aircraft-${sim.current.ctrlType}-${Date.now()}.csv`;
    anchor.click();
    URL.revokeObjectURL(url);
  };

  useEffect(() => {
    const onKeyDown = event => {
      if (event.target instanceof HTMLInputElement) return;
      if (event.code === 'Space') {
        event.preventDefault();
        setPaused(value => !value);
      }
      if (event.key.toLowerCase() === 'r') resetSim();
      if (event.key.toLowerCase() === 'g') triggerGust();
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, []);

  return (
    // Blueprint / Cyber background pattern
    <div className="flex min-h-screen flex-col bg-slate-950 text-slate-200 font-sans p-3 md:p-4 gap-4 overflow-x-hidden relative selection:bg-sky-500/30" dir="rtl"
         style={{ backgroundImage: 'radial-gradient(circle at 50% 0%, #0f172a 0%, #020617 100%)' }}>
      
      {/* Subtle Grid Overlay over the entire app */}
      <div className="absolute inset-0 pointer-events-none opacity-[0.03]" style={{ backgroundImage: 'linear-gradient(#38bdf8 1px, transparent 1px), linear-gradient(90deg, #38bdf8 1px, transparent 1px)', backgroundSize: '40px 40px' }}></div>

      {/* Top Glass HUD Header */}
      <header className="flex items-center justify-between bg-slate-900/40 backdrop-blur-xl border border-slate-700/50 p-4 rounded-2xl shadow-[0_8px_32px_rgba(0,0,0,0.4)] z-10">
        <div className="flex items-center gap-4">
          <div className="p-3 bg-sky-500/10 rounded-xl border border-sky-400/30 shadow-[0_0_15px_rgba(56,189,248,0.2)]">
            <Cpu className="text-sky-400" size={28} />
          </div>
          <div>
            <h1 className="text-2xl font-bold text-white tracking-wide uppercase">Advanced Autopilot <span className="text-sky-400 font-light">Testbed</span></h1>
            <p className="text-slate-400 text-sm mt-0.5 flex items-center gap-2 font-mono">
              <Zap size={14} className="text-amber-400" />
              STATUS: <span className={paused ? 'text-amber-400' : 'text-emerald-400'}>{paused ? 'STANDBY' : 'ACTIVE_SIMULATION'}</span>
            </p>
          </div>
        </div>
        <div className="flex gap-2 md:gap-3">
          <button onClick={exportTelemetry} disabled={!sim.current.history.length} className="p-3 bg-slate-800/80 hover:bg-slate-700 disabled:opacity-40 disabled:cursor-not-allowed rounded-xl border border-slate-600 transition-all shadow-lg backdrop-blur-sm group" title="ייצוא טלמטריה ל-CSV" aria-label="ייצוא טלמטריה">
            <Download className="text-sky-400 group-hover:-translate-y-0.5 transition-transform" />
          </button>
          <button onClick={() => setPaused(!paused)} className="p-3 bg-slate-800/80 hover:bg-slate-700 rounded-xl border border-slate-600 transition-all shadow-lg backdrop-blur-sm group" aria-label={paused ? 'המשך סימולציה' : 'השהיית סימולציה'}>
            {paused ? <Play className="text-emerald-400 group-hover:scale-110 transition-transform" /> : <Pause className="text-amber-400 group-hover:scale-110 transition-transform" />}
          </button>
          <button onClick={resetSim} className="p-3 bg-slate-800/80 hover:bg-slate-700 rounded-xl border border-slate-600 transition-all shadow-lg backdrop-blur-sm group" title="איפוס מצבים" aria-label="איפוס הסימולציה">
            <RotateCcw className="text-rose-400 group-hover:-rotate-90 transition-transform" />
          </button>
        </div>
      </header>

      <div className="flex flex-col lg:flex-row gap-5 z-10">
        
        {/* Left Col: Viewport & Primary Controls */}
        <div className="flex flex-col gap-5 lg:w-8/12 relative">
          
          {/* 3D Canvas (Interactive) */}
          <div 
            className="h-[440px] lg:h-[560px] bg-slate-950 border border-slate-700/50 rounded-2xl overflow-hidden relative cursor-move shadow-[inset_0_0_100px_rgba(0,0,0,0.8),0_10px_30px_rgba(0,0,0,0.5)]"
            onMouseDown={handleMouseDown} onMouseUp={handleMouseUp} onMouseLeave={handleMouseUp} onMouseMove={handleMouseMove}
            onTouchStart={(e) => handleMouseDown(e.touches[0])} onTouchEnd={handleMouseUp} onTouchMove={(e) => handleMouseMove(e.touches[0])}
          >
            {/* Attitude Indicator Overlay */}
            <div className="absolute top-6 left-6 flex gap-4 z-10 pointer-events-none">
              <AttitudeIndicator
                roll={stats.phi}
                pitch={physicsMode === 'advanced' ? stats.pitch : Math.abs(stats.phi) * 0.05}
              />
            </div>
            
            {/* Time Overlay */}
            <div className="absolute top-6 right-6 bg-slate-900/80 border border-slate-600 px-4 py-2 rounded-lg text-sm font-mono text-sky-400 backdrop-blur-md pointer-events-none shadow-[0_0_15px_rgba(56,189,248,0.2)]" dir="ltr">
              TIME: {stats.time.toFixed(2)}s
            </div>

            {showAdvancedHud && (
              <>
                <div className="flight-hud flight-hud-left" dir="ltr">
                  <span>IAS</span>
                  <strong>{stats.airspeed.toFixed(0)}</strong>
                  <small>m/s · M {stats.mach.toFixed(2)}</small>
                  <span>α {stats.aoa.toFixed(1)}°</span>
                </div>
                <div className="flight-hud flight-hud-right" dir="ltr">
                  <span>ALT</span>
                  <strong>{stats.altitude.toFixed(0)}</strong>
                  <small>m · VS {stats.verticalSpeed.toFixed(1)} m/s</small>
                  <span>HDG {stats.heading.toFixed(0).padStart(3, '0')}°</span>
                </div>
                <div className="flight-path-marker" aria-hidden="true">
                  <span />
                </div>
                <div className="flight-load-strip" dir="ltr">
                  <span>G {stats.gLoad.toFixed(2)}</span>
                  <span>TURN {stats.turnRate.toFixed(2)}°/s</span>
                  <span>{physicsMode === 'advanced' ? '6-DOF FLIGHT SHELL' : '5-STATE PROJECT MODEL'}</span>
                </div>
              </>
            )}

            <div ref={mountRef} className="w-full h-full" />
            
            <div className="absolute bottom-4 left-0 w-full text-center pointer-events-none text-slate-500 text-xs font-mono tracking-widest">
              <Compass size={14} className="inline mr-2 opacity-50"/>
              DRAG VIEWPORT TO ROTATE CAMERA
            </div>
          </div>

          {/* Controller Selection Panel */}
          <div className="bg-slate-900/60 backdrop-blur-xl border border-slate-700/50 rounded-2xl p-5 shadow-xl shrink-0">
            <div className="flex justify-between items-center mb-4">
              <h2 className="text-lg font-bold text-white flex items-center gap-2"><Monitor size={18} className="text-sky-400"/> ארכיטקטורת בקרה פעילה</h2>
            </div>
            
            <div className="grid grid-cols-2 md:grid-cols-5 gap-3">
              {Object.entries(CONTROLLERS).map(([key, info]) => (
                <button
                  key={key}
                  onClick={() => handleControllerChange(key)}
                  className={`text-right p-3 rounded-xl border transition-all duration-300 flex flex-col gap-1 relative overflow-hidden group
                    ${controller === key 
                      ? 'bg-slate-800 scale-[1.02] z-10' 
                      : 'bg-slate-900/50 border-slate-700 text-slate-400 hover:bg-slate-800'
                  }`}
                  style={{
                    borderColor: controller === key ? info.color : '',
                    boxShadow: controller === key ? `0 0 20px ${info.glow}` : 'none'
                  }}
                >
                  <div className={`font-bold text-sm transition-colors ${controller === key ? 'text-white' : ''}`} style={{ color: controller === key ? info.color : '' }}>{info.name}</div>
                  <div className="text-[10px] opacity-80 leading-tight">{info.desc}</div>
                </button>
              ))}
            </div>
          </div>

          <div className="simulation-mode-panel">
            <div className="simulation-mode-copy">
              <span>SIMULATION FIDELITY</span>
              <strong>מצב פיזיקה ותצוגה</strong>
              <small>
                מצב הפרויקט שומר על המודל הליניארי המקורי; המצב המתקדם מוסיף
                קינמטיקה תלת־ממדית, אטמוספרה ואווירודינמיקה.
              </small>
            </div>
            <div className="simulation-mode-actions">
              <div className="simulation-segmented" dir="ltr">
                <button
                  type="button"
                  onClick={() => handlePhysicsModeChange('project')}
                  className={physicsMode === 'project' ? 'is-active' : ''}
                >
                  PROJECT MODEL
                  <span>5-state</span>
                </button>
                <button
                  type="button"
                  onClick={() => handlePhysicsModeChange('advanced')}
                  className={physicsMode === 'advanced' ? 'is-active is-advanced' : ''}
                >
                  ADVANCED PHYSICS
                  <span>6-DOF shell</span>
                </button>
              </div>
              <div className="simulation-switches">
                <label>
                  <input
                    type="checkbox"
                    checked={showAdvancedHud}
                    onChange={event => setShowAdvancedHud(event.target.checked)}
                  />
                  <span>HUD מתקדם</span>
                </label>
                <label>
                  <input
                    type="checkbox"
                    checked={showEnvironmentFx}
                    onChange={event => setShowEnvironmentFx(event.target.checked)}
                  />
                  <span>אפקטי סביבה</span>
                </label>
                <label>
                  <input
                    type="checkbox"
                    checked={showControlDiagram}
                    onChange={event => setShowControlDiagram(event.target.checked)}
                  />
                  <span>דיאגרמה חיה</span>
                </label>
              </div>
            </div>
          </div>

        </div>

        {/* Right Col: Environment & Charts */}
        <div className="flex flex-col gap-5 lg:w-4/12">
          
          {/* Environment Controls */}
          <div className="bg-slate-900/60 backdrop-blur-xl border border-slate-700/50 rounded-2xl p-5 shadow-xl shrink-0 grid grid-cols-1 sm:grid-cols-2 gap-6">
            
            {/* Command Generation */}
            <div>
              <label className="text-sm font-bold text-slate-300 mb-3 block flex items-center gap-2">
                <Crosshair size={16} className="text-sky-400"/>
                פקודת זווית (φ_c)
              </label>
              <div className="flex items-center gap-3 bg-slate-950 p-2.5 rounded-xl border border-slate-700 shadow-inner">
                <input 
                  type="range" min="-60" max="60" step="5"
                  value={cmdDeg} 
                  onChange={(e) => {
                    const nextCommand = Number(e.target.value);
                    setCmdDeg(nextCommand);
                    handleCommandChange(nextCommand);
                  }}
                  className="flex-1 accent-sky-500 h-1.5 bg-slate-800 rounded-lg appearance-none cursor-pointer"
                  dir="ltr"
                />
                <span className="font-mono font-bold text-white min-w-[45px] text-center text-sm bg-slate-800 py-1 rounded">{cmdDeg}°</span>
              </div>
              <div className="flex gap-2 mt-3">
                <button onClick={() => {setCmdDeg(30); handleCommandChange(30);}} className="flex-1 bg-slate-800 hover:bg-slate-700 text-xs font-bold py-2 rounded-lg border border-slate-600 transition">30°</button>
                <button onClick={() => {setCmdDeg(-30); handleCommandChange(-30);}} className="flex-1 bg-slate-800 hover:bg-slate-700 text-xs font-bold py-2 rounded-lg border border-slate-600 transition">-30°</button>
                <button onClick={() => {setCmdDeg(0); handleCommandChange(0);}} className="flex-1 bg-slate-800 hover:bg-slate-700 text-xs font-bold py-2 rounded-lg border border-slate-600 transition">0°</button>
              </div>
            </div>

            {/* Disturbances */}
            <div className="flex flex-col gap-3 justify-end">
              <button 
                onClick={triggerGust}
                className="w-full flex items-center justify-center gap-2 bg-indigo-600/20 hover:bg-indigo-600/40 border border-indigo-500/50 text-indigo-300 font-bold py-2.5 rounded-xl transition-all shadow-[0_0_15px_rgba(79,70,229,0.2)] active:scale-95"
              >
                <Wind size={18} />
                הזרקת מכת רוח (Gust)
              </button>
              
              <div className="bg-slate-950 p-3 rounded-xl border border-slate-700 shadow-inner">
                 <div className="flex justify-between items-center text-xs text-slate-300 mb-2 font-bold">
                   <span className="flex items-center gap-1"><ShieldAlert size={14} className="text-amber-400"/> רעש חיישן</span>
                   <span className="font-mono bg-slate-800 px-2 py-0.5 rounded">{noiseLvl.toFixed(1)}°</span>
                 </div>
                 <input 
                  type="range" min="0" max="10" step="0.5"
                  value={noiseLvl} 
                  onChange={(e) => setNoiseLvl(Number(e.target.value))}
                  className="w-full accent-amber-500 h-1.5 bg-slate-800 rounded-lg appearance-none cursor-pointer"
                  dir="ltr"
                />
              </div>
              <div className="flex items-center justify-between bg-slate-950 p-3 rounded-xl border border-slate-700 shadow-inner">
                <span className="flex items-center gap-2 text-xs font-bold text-slate-300">
                  <Gauge size={14} className="text-emerald-400" />
                  קצב סימולציה
                </span>
                <select
                  value={simSpeed}
                  onChange={event => setSimSpeed(Number(event.target.value))}
                  className="rounded-lg border border-slate-600 bg-slate-800 px-2 py-1 text-xs font-mono text-white"
                  dir="ltr"
                >
                  <option value={0.25}>0.25×</option>
                  <option value={0.5}>0.5×</option>
                  <option value={1}>1×</option>
                  <option value={2}>2×</option>
                </select>
              </div>
            </div>
          </div>

          {/* Telemetry Stats */}
          <div className="grid grid-cols-2 xl:grid-cols-5 gap-3 shrink-0">
            <div className="bg-slate-900/60 backdrop-blur-xl border border-slate-700/50 rounded-2xl p-4 flex flex-col items-center justify-center text-center shadow-lg">
              <span className="text-slate-400 text-[10px] mb-1 font-bold uppercase tracking-widest">ACTUAL ROLL</span>
              <span className="text-2xl font-mono text-white font-bold" dir="ltr">{stats.phi.toFixed(2)}°</span>
            </div>
            <div className="bg-slate-900/60 backdrop-blur-xl border border-slate-700/50 rounded-2xl p-4 flex flex-col items-center justify-center text-center relative overflow-hidden shadow-lg">
              {Math.abs(stats.da) > 4.9 && <div className="absolute inset-0 bg-rose-500/20 animate-pulse" />}
              <span className="text-slate-400 text-[10px] mb-1 font-bold uppercase tracking-widest relative z-10">AILERON DEFLECTION</span>
              <span className={`text-2xl font-mono font-bold relative z-10 ${Math.abs(stats.da) > 4.9 ? 'text-rose-400 drop-shadow-[0_0_8px_rgba(244,63,94,0.8)]' : 'text-emerald-400 drop-shadow-[0_0_8px_rgba(16,185,129,0.5)]'}`} dir="ltr">{stats.da.toFixed(2)}°</span>
            </div>
            <div className="bg-slate-900/60 backdrop-blur-xl border border-slate-700/50 rounded-2xl p-4 flex flex-col items-center justify-center text-center shadow-lg">
              <span className="text-slate-400 text-[10px] mb-1 font-bold uppercase tracking-widest">TRACKING ERROR</span>
              <span className="text-2xl font-mono text-amber-400 font-bold drop-shadow-[0_0_8px_rgba(251,191,36,0.5)]" dir="ltr">{Math.abs(stats.ess).toFixed(2)}°</span>
            </div>
            <div className="bg-slate-900/60 backdrop-blur-xl border border-slate-700/50 rounded-2xl p-4 flex flex-col items-center justify-center text-center shadow-lg">
              <span className="text-slate-400 text-[10px] mb-1 font-bold uppercase tracking-widest">ROLL RATE</span>
              <span className="text-xl font-mono text-sky-300 font-bold" dir="ltr">{stats.p.toFixed(2)}°/s</span>
            </div>
            <div className="bg-slate-900/60 backdrop-blur-xl border border-slate-700/50 rounded-2xl p-4 flex flex-col items-center justify-center text-center shadow-lg">
              <span className="text-slate-400 text-[10px] mb-1 font-bold uppercase tracking-widest">SERVO CMD</span>
              <span className="text-xl font-mono text-fuchsia-300 font-bold" dir="ltr">{stats.dc.toFixed(2)}°</span>
            </div>
          </div>

          <div className="flex items-center justify-between rounded-xl border border-slate-700/60 bg-slate-900/40 px-4 py-2 text-[11px] font-mono text-slate-400" dir="ltr">
            <span className="flex items-center gap-1"><Activity size={13} /> MAX |δa|: <strong className={stats.maxDa > 5 ? 'text-rose-400' : 'text-emerald-400'}>{stats.maxDa.toFixed(2)}°</strong></span>
            <span>MAX |δc|: <strong className="text-amber-300">{stats.maxDc.toFixed(2)}°</strong></span>
            <span className="hidden 2xl:inline">SPACE pause · R reset · G gust</span>
          </div>

          {/* Oscilloscopes */}
          <div className="flex flex-col gap-4 min-h-[420px]">
            <div className="bg-slate-950 backdrop-blur-xl border border-slate-700/80 rounded-2xl p-1.5 flex-1 relative overflow-hidden shadow-[inset_0_0_20px_rgba(0,0,0,0.5),0_10px_20px_rgba(0,0,0,0.4)]">
              <canvas ref={chartRollRef} width="800" height="250" className="w-full h-full rounded-xl" />
            </div>
            <div className="bg-slate-950 backdrop-blur-xl border border-slate-700/80 rounded-2xl p-1.5 flex-1 relative overflow-hidden shadow-[inset_0_0_20px_rgba(0,0,0,0.5),0_10px_20px_rgba(0,0,0,0.4)]">
              <canvas ref={chartAileronRef} width="800" height="250" className="w-full h-full rounded-xl" />
            </div>
          </div>

        </div>
      </div>

      {showControlDiagram && (
        <LiveControlArchitecture
          controller={controller}
          cmdDeg={cmdDeg}
          stats={stats}
          paused={paused}
        />
      )}
    </div>
  );
}
