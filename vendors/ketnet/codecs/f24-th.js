/*
 * KETNET F24-TH LoRaWAN thermometer.
 *
 * Source of truth for the wire formats is the firmware: LoRaWAN/App/app_payload.h
 * (FPort 2) and LoRaWAN/App/app_config.h (FPort 10/11/12). All multi-byte
 * fields are big-endian.
 *
 *   FPort 2  uplink    periodic / event measurement (v1 7 B, v2 15 B)
 *   FPort 3  uplink    degree-day day close (13 B), once per 24 h window
 *   FPort 10 downlink  CONFIG_DOWN      (19 B)
 *   FPort 11 downlink  APP_PARAMS_DOWN  (4 B)
 *   FPort 12 uplink    config echo      (21 B), sent after FPort 10/11 applied
 *
 * encodeDownlink() cannot choose the FPort (ChirpStack takes it from the
 * queue item), so the message type is inferred from the JSON keys - set the
 * matching fPort when enqueueing:
 *
 *   FPort 10: {"tx_period_s":300, "rejoin_period_h":0,
 *              "event_enable":{"mag":true,"acc":true,"in1":true,"in2":true,"btn2":true,"temp":true},
 *              "degreeday_seed":0, "in1_counter_seed":0, "in2_counter_seed":0}
 *     tx_period_s, rejoin_period_h and event_enable (or the raw
 *     event_enable_mask) are REQUIRED - every CONFIG_DOWN overwrites all of
 *     them. The three *_seed fields are optional; a present seed sets its
 *     command_flags bit, an absent one leaves that value untouched on the
 *     node. degreeday_seed is raw 0.1 degC*day units (like degreeday_accum);
 *     seeding it also restarts the FPort 3 day_index at 0.
 *     The node clamps tx_period_s up to 10 s; the echo shows the applied value.
 *
 *   FPort 11: {"target_temp_c":21.5, "heating":true}
 *
 * For more information, please refer to:
 * https://resources.lora-alliance.org/technical-specifications/ts013-1-0-0-payload-codec-api
 */

var EVENT_BITS = { mag: 0x01, acc: 0x02, in1: 0x04, in2: 0x08, btn2: 0x10, btn1: 0x20, temp: 0x40 };
var EVENT_ENABLE_VALID = 0x5F; /* MAG/ACC/IN1/IN2/BTN2/TEMP - BTN1 is never gated */

var CMD_SEED_DEGREEDAY = 0x01;
var CMD_SEED_IN1 = 0x02;
var CMD_SEED_IN2 = 0x04;

function u16(b, i) { return (b[i] << 8) | b[i + 1]; }
function s16(b, i) { var v = u16(b, i); return v & 0x8000 ? v - 0x10000 : v; }
function u32(b, i) { return ((b[i] << 24) >>> 0) + (b[i + 1] << 16) + (b[i + 2] << 8) + b[i + 3]; }
function s32(b, i) { return (b[i] << 24) | (b[i + 1] << 16) | (b[i + 2] << 8) | b[i + 3]; }

function putU16(out, v) { out.push((v >> 8) & 0xFF, v & 0xFF); }
function putU32(out, v) { out.push((v >>> 24) & 0xFF, (v >>> 16) & 0xFF, (v >>> 8) & 0xFF, v & 0xFF); }

function bitsToFlags(mask, names) {
  var o = {};
  for (var i = 0; i < names.length; i++) {
    o[names[i]] = (mask & EVENT_BITS[names[i]]) !== 0;
  }
  return o;
}

function eventNames(mask) {
  if (mask === 0) { return ["PERIODIC"]; }
  var list = [];
  for (var k in EVENT_BITS) {
    if (mask & EVENT_BITS[k]) { list.push(k.toUpperCase()); }
  }
  return list;
}

function decodeMeasurement(b) {
  var version = b[0];
  if (!((version === 1 && b.length === 7) || (version === 2 && b.length === 15))) {
    throw new Error("FPort 2: unexpected version " + version + " / length " + b.length);
  }
  var d = {
    version: version,
    event_flags: b[1],
    events: eventNames(b[1]),
    battery_level: b[2],
    temperature_c: s16(b, 3) / 10,
    humidity_rh: b[5],
    io_state: { in1: (b[6] & 0x01) !== 0, in2: (b[6] & 0x02) !== 0, mag: (b[6] & 0x04) !== 0 }
  };
  if (version === 2) {
    d.in1_counter = u32(b, 7);
    d.in2_counter = u32(b, 11);
  }
  return d;
}

function decodeEcho(b) {
  if (b.length !== 21 || b[0] !== 1) {
    throw new Error("FPort 12: unexpected version " + b[0] + " / length " + b.length);
  }
  var accum = s32(b, 9);
  return {
    version: b[0],
    tx_period_s: u16(b, 1),
    rejoin_period_h: u16(b, 3),
    event_enable_mask: b[5],
    event_enable: bitsToFlags(b[5], ["mag", "acc", "in1", "in2", "btn2", "temp"]),
    target_temp_c: s16(b, 6) / 10,
    heating: (b[8] & 0x01) !== 0,
    degreeday_accum: accum,
    degreeday_accum_c: accum / 10,
    in1_counter: u32(b, 13),
    in2_counter: u32(b, 17)
  };
}

function decodeDegreeday(b) {
  if (b.length !== 13 || b[0] !== 1) {
    throw new Error("FPort 3: unexpected version " + b[0] + " / length " + b.length);
  }
  var accum = s32(b, 9);
  return {
    version: b[0],
    day_index: u16(b, 1),
    daily_avg_c: s16(b, 3) / 10,
    daily_min_c: s16(b, 5) / 10,
    daily_max_c: s16(b, 7) / 10,
    degreeday_accum: accum,
    degreeday_accum_c: accum / 10
  };
}

function decodeUplink(input) {
  try {
    if (input.fPort === 2) { return { data: decodeMeasurement(input.bytes) }; }
    if (input.fPort === 3) { return { data: decodeDegreeday(input.bytes) }; }
    if (input.fPort === 12) { return { data: decodeEcho(input.bytes) }; }
    return { data: {}, warnings: ["unhandled fPort " + input.fPort] };
  } catch (e) {
    return { errors: [e.message] };
  }
}

function requireInt(d, key, min, max) {
  var v = d[key];
  if (typeof v !== "number" || Math.floor(v) !== v || v < min || v > max) {
    throw new Error(key + " must be an integer in " + min + ".." + max);
  }
  return v;
}

function encodeConfig(d) {
  var mask;
  if (typeof d.event_enable_mask === "number") {
    mask = requireInt(d, "event_enable_mask", 0, 255) & EVENT_ENABLE_VALID;
  } else if (d.event_enable && typeof d.event_enable === "object") {
    mask = 0;
    for (var k in d.event_enable) {
      if (!(k in EVENT_BITS) || k === "btn1") { throw new Error("event_enable: unknown key " + k); }
      if (d.event_enable[k]) { mask |= EVENT_BITS[k]; }
    }
  } else {
    throw new Error("event_enable (object) or event_enable_mask (number) is required");
  }

  var flags = 0;
  var dd = 0, c1 = 0, c2 = 0;
  if (d.degreeday_seed !== undefined) { dd = requireInt(d, "degreeday_seed", -2147483648, 2147483647); flags |= CMD_SEED_DEGREEDAY; }
  if (d.in1_counter_seed !== undefined) { c1 = requireInt(d, "in1_counter_seed", 0, 4294967295); flags |= CMD_SEED_IN1; }
  if (d.in2_counter_seed !== undefined) { c2 = requireInt(d, "in2_counter_seed", 0, 4294967295); flags |= CMD_SEED_IN2; }

  var out = [1];
  putU16(out, requireInt(d, "tx_period_s", 0, 65535));
  putU16(out, requireInt(d, "rejoin_period_h", 0, 65535));
  out.push(mask, flags);
  putU32(out, dd);
  putU32(out, c1);
  putU32(out, c2);
  return out;
}

function encodeParams(d) {
  if (typeof d.target_temp_c !== "number") { throw new Error("target_temp_c (degC) is required"); }
  var t = Math.round(d.target_temp_c * 10);
  if (t < -32768 || t > 32767) { throw new Error("target_temp_c out of range"); }
  var out = [1];
  putU16(out, t & 0xFFFF);
  out.push(d.heating ? 0x01 : 0x00);
  return out;
}

function encodeDownlink(input) {
  var d = input.data || {};
  try {
    var isParams = ("target_temp_c" in d) || ("heating" in d);
    var isConfig = ("tx_period_s" in d) || ("rejoin_period_h" in d) || ("event_enable" in d)
        || ("event_enable_mask" in d);
    if (isParams && isConfig) { throw new Error("mixed FPort 10 and FPort 11 fields - send them separately"); }
    if (isParams) { return { bytes: encodeParams(d) }; }
    if (isConfig) { return { bytes: encodeConfig(d) }; }
    throw new Error("no known fields (FPort 10: tx_period_s/...; FPort 11: target_temp_c/heating)");
  } catch (e) {
    return { errors: [e.message] };
  }
}

function decodeDownlink(input) {
  var b = input.bytes;
  try {
    if (input.fPort === 10) {
      if (b.length !== 19 || b[0] !== 1) { throw new Error("FPort 10: bad version/length"); }
      var d = {
        tx_period_s: u16(b, 1),
        rejoin_period_h: u16(b, 3),
        event_enable_mask: b[5],
        event_enable: bitsToFlags(b[5], ["mag", "acc", "in1", "in2", "btn2", "temp"])
      };
      if (b[6] & CMD_SEED_DEGREEDAY) { d.degreeday_seed = s32(b, 7); }
      if (b[6] & CMD_SEED_IN1) { d.in1_counter_seed = u32(b, 11); }
      if (b[6] & CMD_SEED_IN2) { d.in2_counter_seed = u32(b, 15); }
      return { data: d };
    }
    if (input.fPort === 11) {
      if (b.length !== 4 || b[0] !== 1) { throw new Error("FPort 11: bad version/length"); }
      return { data: { target_temp_c: s16(b, 1) / 10, heating: (b[3] & 0x01) !== 0 } };
    }
    return { data: {}, warnings: ["unhandled fPort " + input.fPort] };
  } catch (e) {
    return { errors: [e.message] };
  }
}
