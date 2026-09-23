#!/usr/bin/env node
/**
 * Command-line text HUD.
 *
 * A headless library needs a way to be looked at, and this is it: the same
 * scenarios the test suite asserts on, rendered as a fixed-width telemetry
 * trace plus the measured metrics and the model's own notes. Nothing here
 * computes physics — every number comes from the library, which is the point.
 * If the CLI and the tests ever disagreed, the tests would be right.
 *
 * Usage:
 *   node src/cli.ts list
 *   node src/cli.ts run <scenario> [--hud] [--every <s>] [--max-rows <n>]
 *   node src/cli.ts trim <aircraft> <altitudeM> <tasMps> [--throttle <0..1>]
 *   node src/cli.ts envelope <aircraft>
 *   node src/cli.ts info <aircraft>
 *
 * Run with no arguments for the help text.
 */

import {
  bestLiftToDrag,
  bestLiftToDragCoefficient,
  envelopeSweep,
  isaAtGeometricAltitude,
  maxLevelSpeed,
  minimumLevelTrimSpeed,
  PRESET_NAMES,
  PRESETS,
  runScenario,
  SCENARIO_NAMES,
  speedForLiftCoefficient,
  stallSpeedMps,
  trim,
  weightN,
  wingLoading,
  type AircraftSpec,
  type ScenarioName,
  type Telemetry,
} from './index.ts';

const KT = 1.943844;

/* ---------------------------------------------------------------- formatting */

const pad = (value: string, width: number, right = false): string =>
  right ? value.padStart(width) : value.padEnd(width);

const num = (value: number, dp: number, width: number): string => {
  if (!Number.isFinite(value)) return pad('n/a', width, true);
  return pad(value.toFixed(dp), width, true);
};

const rule = (width = 74): string => '-'.repeat(width);

const title = (text: string): string => `\n${text}\n${'='.repeat(Math.min(text.length, 74))}`;

/* ------------------------------------------------------------------- helpers */

function resolveAircraft(name: string): AircraftSpec {
  const upper = name.toUpperCase();
  const spec = PRESETS[upper];
  if (!spec) {
    throw new Error(`unknown aircraft "${name}"; known: ${PRESET_NAMES.join(', ')}`);
  }
  return spec;
}

function resolveScenario(name: string): ScenarioName {
  const match = SCENARIO_NAMES.find((candidate) => candidate === name.toLowerCase());
  if (!match) {
    throw new Error(`unknown scenario "${name}"; known: ${SCENARIO_NAMES.join(', ')}`);
  }
  return match;
}

/** Evenly-spaced subset of the samples, always including the last one. */
function decimate(samples: readonly Telemetry[], maxRows: number): Telemetry[] {
  if (samples.length <= maxRows) return [...samples];
  const step = Math.ceil(samples.length / maxRows);
  const out: Telemetry[] = [];
  for (let i = 0; i < samples.length; i += step) out.push(samples[i]!);
  const last = samples[samples.length - 1]!;
  if (out[out.length - 1] !== last) out.push(last);
  return out;
}

interface ParsedArgs {
  readonly command: string;
  readonly positional: readonly string[];
  readonly flags: ReadonlyMap<string, string | true>;
}

function parseArgs(argv: readonly string[]): ParsedArgs {
  const positional: string[] = [];
  const flags = new Map<string, string | true>();
  for (let i = 0; i < argv.length; i++) {
    const token = argv[i]!;
    if (token.startsWith('--')) {
      const key = token.slice(2);
      const next = argv[i + 1];
      if (next !== undefined && !next.startsWith('--')) {
        flags.set(key, next);
        i += 1;
      } else {
        flags.set(key, true);
      }
    } else {
      positional.push(token);
    }
  }
  return { command: positional[0] ?? 'help', positional: positional.slice(1), flags };
}

const flagNumber = (flags: ReadonlyMap<string, string | true>, key: string, fallback: number): number => {
  const raw = flags.get(key);
  if (raw === undefined || raw === true) return fallback;
  const value = Number(raw);
  if (!Number.isFinite(value)) throw new Error(`--${key} expects a number, got "${raw}"`);
  return value;
};

/* ------------------------------------------------------------------ commands */

function printHelp(): void {
  const lines = [
    'AirCraft — headless 6-DOF flight dynamics sandbox',
    '',
    'Usage: node src/cli.ts <command> [arguments] [options]',
    '',
    'Commands:',
    '  list                                   List aircraft and scenarios',
    '  run <scenario> [--hud] [--every <s>]   Run a scripted scenario',
    '  trim <aircraft> <altM> <tasMps>        Solve for a trimmed flight condition',
    '       [--throttle <0..1>]               ...fixed throttle (a glide) instead of level',
    '  envelope <aircraft>                    Trim sweep across the usable envelope',
    '  info <aircraft>                        Derived performance figures',
    '',
    `Aircraft:  ${PRESET_NAMES.join(', ')}`,
    `Scenarios: ${SCENARIO_NAMES.join(', ')}`,
    '',
    'Options:',
    '  --hud               Print the telemetry trace (run only)',
    '  --every <seconds>   Trace sampling interval (default 5)',
    '  --json              Emit raw JSON instead of a table',
    '',
    'Examples:',
    '  node src/cli.ts run takeoff --hud',
    '  node src/cli.ts run glide --hud --every 20',
    '  node src/cli.ts trim C172 1500 60',
    '  node src/cli.ts trim GLIDER 600 29 --throttle 0',
    '  node src/cli.ts info C172',
  ];
  process.stdout.write(`${lines.join('\n')}\n`);
}

function printList(): void {
  process.stdout.write(`${title('Aircraft')}\n`);
  for (const name of PRESET_NAMES) {
    const spec = PRESETS[name]!;
    const vs = stallSpeedMps(spec, isaAtGeometricAltitude(0));
    process.stdout.write(
      `  ${pad(name, 8)} ${pad(spec.name, 34)} ${num(spec.mass.massKg, 0, 5)} kg  ` +
        `Vs ${num(vs * KT, 1, 5)} kt  L/D ${num(bestLiftToDrag(spec), 1, 5)}\n`,
    );
  }
  process.stdout.write(
    `\nRun a scenario with:  node src/cli.ts run <scenario> --hud\n` +
      `Scenarios: ${SCENARIO_NAMES.join(', ')}\n`,
  );
}

function printMetrics(metrics: Readonly<Record<string, number>>): void {
  process.stdout.write(`${title('Metrics')}\n`);
  const width = Math.max(...Object.keys(metrics).map((k) => k.length));
  for (const [key, value] of Object.entries(metrics)) {
    const shown = Number.isFinite(value) ? (Math.abs(value) >= 1e5 || (Math.abs(value) < 1e-3 && value !== 0) ? value.toExponential(3) : value.toFixed(4)) : 'n/a';
    process.stdout.write(`  ${pad(key, width)}  ${shown.padStart(14)}\n`);
  }
}

function printNotes(notes: readonly string[]): void {
  if (notes.length === 0) return;
  process.stdout.write(`${title('Notes')}\n`);
  for (const note of notes) {
    for (const [index, line] of wrap(note, 70).entries()) {
      process.stdout.write(`${index === 0 ? '  - ' : '    '}${line}\n`);
    }
  }
}

function wrap(text: string, width: number): string[] {
  const words = text.split(/\s+/);
  const lines: string[] = [];
  let current = '';
  for (const word of words) {
    if (current.length === 0) current = word;
    else if (current.length + 1 + word.length <= width) current += ` ${word}`;
    else {
      lines.push(current);
      current = word;
    }
  }
  if (current.length > 0) lines.push(current);
  return lines;
}

function printTrace(samples: readonly Telemetry[], dt: number): void {
  const header =
    pad('t/s', 7, true) +
    pad('alt/m', 9, true) +
    pad('TAS/kt', 9, true) +
    pad('AOA/o', 8, true) +
    pad('pitch/o', 9, true) +
    pad('roll/o', 8, true) +
    pad('VS/m/s', 9, true) +
    pad('n', 7, true) +
    pad('thr', 6, true) +
    pad('flags', 12, true);
  process.stdout.write(`${title('Telemetry')}\n`);
  process.stdout.write(`  ${header}\n`);
  process.stdout.write(`  ${rule(header.length)}\n`);
  for (const s of samples) {
    const flags = s.stalled ? 'STALL' : s.speedMarginMps < 0 ? 'SLOW' : '';
    process.stdout.write(
      `  ` +
        num(s.timeS, 1, 7) +
        num(s.altitudeM, 0, 9) +
        num(s.trueAirspeedKt, 1, 9) +
        num(s.alphaDeg, 1, 8) +
        num(s.pitchDeg, 1, 9) +
        num(s.rollDeg, 1, 8) +
        num(s.verticalSpeedMps, 1, 9) +
        num(s.loadFactor, 2, 7) +
        num(s.throttle, 2, 6) +
        pad(flags, 12, true) +
        '\n',
    );
  }
  process.stdout.write(`  (sampled every ${dt.toFixed(1)} s; full rate is 5-50 Hz)\n`);
}

function commandRun(args: ParsedArgs): void {
  const scenarioArg = args.positional[0];
  if (scenarioArg === undefined) throw new Error('run needs a scenario name; see `list`');
  const name = resolveScenario(scenarioArg);
  const outcome = runScenario(name);

  if (args.flags.has('json')) {
    process.stdout.write(
      `${JSON.stringify(
        {
          name: outcome.name,
          description: outcome.description,
          aircraft: outcome.spec.name,
          metrics: outcome.metrics,
          notes: outcome.notes,
          events: outcome.result.events,
          durationS: outcome.result.durationS,
          completed: outcome.result.completed,
        },
        null,
        2,
      )}\n`,
    );
    return;
  }

  process.stdout.write(`${title(`${outcome.name}  ·  ${outcome.spec.name}`)}\n`);
  process.stdout.write(`  ${outcome.description}\n`);

  if (args.flags.has('hud')) {
    const everyS = flagNumber(args.flags, 'every', 5);
    const lastTime = outcome.result.samples[outcome.result.samples.length - 1]?.timeS ?? 0;
    const maxRows = Math.max(2, Math.min(60, Math.ceil(lastTime / everyS) + 1));
    printTrace(decimate(outcome.result.samples, maxRows), everyS);
  }

  process.stdout.write(`${title('Events')}\n`);
  if (outcome.result.events.length === 0) process.stdout.write('  (none)\n');
  for (const event of outcome.result.events) {
    process.stdout.write(`  ${num(event.timeS, 2, 7)} s  ${pad(event.type, 15)} ${event.detail}\n`);
  }

  printMetrics(outcome.metrics);
  printNotes(outcome.notes);
}

function commandTrim(args: ParsedArgs): void {
  const [aircraftName, altitudeRaw, speedRaw] = args.positional;
  if (aircraftName === undefined || altitudeRaw === undefined || speedRaw === undefined) {
    throw new Error('trim needs: <aircraft> <altitudeM> <tasMps>');
  }
  const spec = resolveAircraft(aircraftName);
  const altitudeM = Number(altitudeRaw);
  const trueAirspeedMps = Number(speedRaw);
  if (!Number.isFinite(altitudeM) || !Number.isFinite(trueAirspeedMps)) {
    throw new Error('altitude and airspeed must be numbers');
  }

  const throttleFlag = args.flags.get('throttle');
  const request =
    throttleFlag === undefined
      ? { altitudeM, trueAirspeedMps }
      : { altitudeM, trueAirspeedMps, throttle: Number(throttleFlag) };
  const solution = trim(spec, request);

  process.stdout.write(`${title(`trim  ·  ${spec.name}`)}\n`);
  process.stdout.write(
    `  ${pad('condition', 26)} ${throttleFlag === undefined ? 'level (gamma = 0), throttle free' : `throttle fixed at ${Number(throttleFlag).toFixed(2)}`}\n`,
  );
  process.stdout.write(`  ${pad('aircraft', 26)} ${spec.name}\n`);
  process.stdout.write(`  ${pad('altitude', 26)} ${num(solution.altitudeM, 0, 8)} m\n`);
  process.stdout.write(`  ${pad('true airspeed', 26)} ${num(solution.trueAirspeedMps, 2, 8)} m/s  (${num(solution.trueAirspeedMps * KT, 1, 6)} kt)\n`);
  process.stdout.write(`  ${pad('converged', 26)} ${solution.converged ? `yes (${solution.iterations} iterations)` : 'NO'}\n`);
  process.stdout.write(`  ${pad('residual norm', 26)} ${solution.residualNorm.toExponential(3)}${solution.residualNorm <= 1e-9 ? '' : '  <- not converged'}\n`);
  process.stdout.write(`  ${pad('angle of attack', 26)} ${num(solution.alphaRad * (180 / Math.PI), 2, 8)} deg\n`);
  process.stdout.write(`  ${pad('pitch attitude', 26)} ${num(solution.thetaRad * (180 / Math.PI), 2, 8)} deg\n`);
  process.stdout.write(`  ${pad('flight path angle', 26)} ${num(solution.flightPathAngleRad * (180 / Math.PI), 2, 8)} deg\n`);
  process.stdout.write(`  ${pad('elevator input', 26)} ${num(solution.controls.elevator, 3, 8)}\n`);
  process.stdout.write(`  ${pad('throttle input', 26)} ${num(solution.controls.throttle, 3, 8)}\n`);
  process.stdout.write(`  ${pad('lift coefficient', 26)} ${num(solution.liftCoefficient, 4, 8)}\n`);
  process.stdout.write(`  ${pad('drag coefficient', 26)} ${num(solution.dragCoefficient, 4, 8)}\n`);
  process.stdout.write(`  ${pad('lift / drag', 26)} ${num(solution.liftToDrag, 3, 8)}\n`);
  process.stdout.write(`  ${pad('lift force', 26)} ${num(solution.diagnostics.forces.liftN, 1, 8)} N\n`);
  process.stdout.write(`  ${pad('drag force', 26)} ${num(solution.diagnostics.forces.dragN, 1, 8)} N\n`);
  process.stdout.write(`  ${pad('thrust', 26)} ${num(solution.diagnostics.thrust.thrustN, 1, 8)} N\n`);
  process.stdout.write(`  ${pad('load factor (body z)', 26)} ${num(solution.diagnostics.loadFactor, 4, 8)}\n`);
  if (solution.stalled) process.stdout.write('  NOTE: this trim sits past the stall break — not a valid flight condition.\n');
}

function commandEnvelope(args: ParsedArgs): void {
  const name = args.positional[0];
  if (name === undefined) throw new Error('envelope needs an aircraft name');
  const spec = resolveAircraft(name);
  const unpowered = !(spec.propulsion.maxShaftPowerW > 0);
  const rows = envelopeSweep(spec, [0, 1500, 3000], [32, 40, 48, 56, 62, 68, 74, 80], unpowered ? 0 : undefined);

  const altitudes = [...new Set(rows.map((row) => row.altitudeM))];
  const speeds = [...new Set(rows.map((row) => row.airspeedMps))];

  process.stdout.write(`${title(`trim envelope  ·  ${spec.name}`)}\n`);
  process.stdout.write(
    `  Cells show angle of attack in degrees; "-" means no trim exists there\n` +
      `  ${unpowered ? 'Throttle fixed at zero: these are glide conditions, not level flight\n\n' : '\n'}`,
  );
  process.stdout.write(`  ${pad('alt\\V', 8)}`);
  for (const v of speeds) process.stdout.write(num(v, 0, 9));
  process.stdout.write('\n');

  for (const altitude of altitudes) {
    process.stdout.write(`  ${pad(String(altitude), 8)}`);
    for (const speed of speeds) {
      const row = rows.find((r) => r.altitudeM === altitude && r.airspeedMps === speed);
      if (!row || !row.trim.converged || row.trim.stalled) process.stdout.write(pad('-', 9, true));
      else process.stdout.write(num(row.trim.alphaRad * (180 / Math.PI), 2, 9));
    }
    process.stdout.write('\n');
  }

  const vmin = minimumLevelTrimSpeed(spec, 0, 0.02, unpowered ? 0 : undefined);
  const vmax = maxLevelSpeed(spec, 0);
  if (Number.isFinite(vmax.speedMps)) {
    process.stdout.write(
      `\n  sea-level usable band: ${num(vmin.speedMps * KT, 1, 5)} kt to ${num(vmax.speedMps * KT, 1, 5)} kt\n`,
    );
  } else {
    process.stdout.write(
      `\n  unpowered: no level-flight band exists at any speed. ` +
        `Minimum glide trim speed ${num(vmin.speedMps * KT, 1, 5)} kt.\n`,
    );
  }
}

function commandInfo(args: ParsedArgs): void {
  const name = args.positional[0];
  if (name === undefined) throw new Error('info needs an aircraft name');
  const spec = resolveAircraft(name);
  const sl = isaAtGeometricAltitude(0);
  const vs = stallSpeedMps(spec, sl);
  const clStar = bestLiftToDragCoefficient(spec);
  const vBestGlide = speedForLiftCoefficient(spec, sl, clStar);
  const vmax = maxLevelSpeed(spec, 0);
  const unpowered = !(spec.propulsion.maxShaftPowerW > 0);

  process.stdout.write(`${title(`airframe  ·  ${spec.name}`)}\n`);
  process.stdout.write(`  ${pad('mass', 24)} ${num(spec.mass.massKg, 0, 8)} kg\n`);
  process.stdout.write(`  ${pad('wing area', 24)} ${num(spec.wing.areaM2, 2, 8)} m^2\n`);
  process.stdout.write(`  ${pad('wing span', 24)} ${num(spec.wing.spanM, 2, 8)} m\n`);
  process.stdout.write(`  ${pad('aspect ratio', 24)} ${num(spec.wing.aspectRatio, 2, 8)}\n`);
  process.stdout.write(`  ${pad('weight', 24)} ${num(weightN(spec), 0, 8)} N\n`);
  process.stdout.write(`  ${pad('wing loading', 24)} ${num(wingLoading(spec), 1, 8)} N/m^2\n`);
  process.stdout.write(`  ${pad('CLmax', 24)} ${num(spec.lift.CLMax, 3, 8)}\n`);
  process.stdout.write(`  ${pad('CD0', 24)} ${num(spec.drag.CD0, 4, 8)}\n`);
  process.stdout.write(`  ${pad('rated shaft power', 24)} ${num(spec.propulsion.maxShaftPowerW / 1000, 1, 8)} kW\n`);
  process.stdout.write(`\nDerived performance (sea level)\n`);
  process.stdout.write(`  ${rule(58)}\n`);
  process.stdout.write(`  ${pad('stall speed (1 g)  Vs', 24)} ${num(vs, 2, 8)} m/s  (${num(vs * KT, 1, 6)} kt)\n`);
  process.stdout.write(`  ${pad('best L/D', 24)} ${num(bestLiftToDrag(spec), 2, 8)}\n`);
  process.stdout.write(`  ${pad('best L/D speed', 24)} ${num(vBestGlide, 2, 8)} m/s  (${num(vBestGlide * KT, 1, 6)} kt)\n`);
  process.stdout.write(
    `  ${pad(unpowered ? 'min glide trim speed' : 'min level trim speed', 24)} ${num(vminOrNan(spec) * KT, 1, 8)} kt\n`,
  );
  if (Number.isFinite(vmax.speedMps)) {
    process.stdout.write(`  ${pad('max level speed', 24)} ${num(vmax.speedMps * KT, 1, 8)} kt\n`);
  } else {
    process.stdout.write(`  ${pad('max level speed', 24)} ${pad('n/a', 8, true)}  (unpowered: no level trim exists)\n`);
  }
  if (unpowered) {
    // Best-glide sink from an actual trim, not from the V/(L/D) estimate, which
    // ignores the coupling between lift and the axial force balance.
    const glide = trim(spec, { altitudeM: 0, trueAirspeedMps: vBestGlide, throttle: 0 });
    if (glide.converged) {
      const sinkMps = vBestGlide * Math.sin(-glide.flightPathAngleRad);
      process.stdout.write(`  ${pad('best-glide sink rate', 24)} ${num(sinkMps, 2, 8)} m/s  (${num(sinkMps * 196.85, 0, 5)} fpm)\n`);
      process.stdout.write(`  ${pad('best glide ratio', 24)} ${num(glide.liftToDrag, 2, 8)}\n`);
    }
  }
}

function vminOrNan(spec: AircraftSpec): number {
  // An unpowered airframe has no level trim at all, so its "minimum speed" is the
  // minimum GLIDE trim speed, which needs the throttle fixed at zero.
  const throttle = spec.propulsion.maxShaftPowerW > 0 ? undefined : 0;
  return minimumLevelTrimSpeed(spec, 0, 0.02, throttle).speedMps;
}

/* --------------------------------------------------------------------- entry */

function main(): void {
  const args = parseArgs(process.argv.slice(2));
  switch (args.command) {
    case 'help':
    case '--help':
    case '-h':
      printHelp();
      return;
    case 'list':
      printList();
      return;
    case 'run':
      commandRun(args);
      return;
    case 'trim':
      commandTrim(args);
      return;
    case 'envelope':
      commandEnvelope(args);
      return;
    case 'info':
      commandInfo(args);
      return;
    default:
      process.stdout.write(`unknown command "${args.command}"\n\n`);
      printHelp();
      process.exitCode = 1;
  }
}

try {
  main();
} catch (error) {
  const message = error instanceof Error ? error.message : String(error);
  process.stderr.write(`error: ${message}\n`);
  process.exitCode = 1;
}
