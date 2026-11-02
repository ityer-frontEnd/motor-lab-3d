// ============================================================================
// THE HEART OF DRIVE — 三相异步电动机 · 3D 交互故障诊断实验室
// Single-file teaching instrument. Three.js r187 (pinned via static import map).
// All geometry, textures, charts, sample data and audio are generated in code.
// ============================================================================

import * as THREE from 'three';

// ===== 0. 工具函数 =====
const TAU = Math.PI * 2;
const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;
const RPM2RAD = TAU / 60;
const RAD2RPM = 60 / TAU;
const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const lerp = (a, b, t) => a + (b - a) * t;
const fract = (x) => x - Math.floor(x);
const damp = (a, b, lambda, dt) => lerp(a, b, 1 - Math.exp(-lambda * Math.max(0, dt)));
const easeInOut = (t) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);
const easeOut = (t) => 1 - Math.pow(1 - t, 3);
const fmt = (v, d = 2) => Number.isFinite(v) ? v.toFixed(d) : '—';
const fmtSigned = (v, d = 2) => `${v >= 0 ? '+' : '−'}${Math.abs(v).toFixed(d)}`;
const pad2 = (n) => String(n).padStart(2, '0');
const nowClock = () => {
  const d = new Date();
  return `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`;
};
const wrapDeg = (deg) => ((deg % 360) + 360) % 360;
const wrapAngle = (rad) => ((rad % TAU) + TAU) % TAU;
const pseudoRandom = (seed) => {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
};
const gauss = (rand) => {
  let u = 0, v = 0;
  while (u === 0) u = rand();
  while (v === 0) v = rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(TAU * v);
};
// Deterministic 50 Hz measurement noise. This is pre-generated and cyclic, not
// an ad-hoc random animation: it gives the normal feature cluster a realistic
// variance / kurtosis without destabilising the displayed rated operating point.
const MEASUREMENT_NOISE = (() => {
  const rand = pseudoRandom(90210);
  return Array.from({ length: 4096 }, () => gauss(rand));
})();
let measurementNoiseIndex = 0;
const setRangeFill = (el) => {
  if (!el) return;
  const min = Number(el.min || 0), max = Number(el.max || 100), value = Number(el.value);
  el.style.setProperty('--fill', `${((value - min) / (max - min)) * 100}%`);
};
const downloadBlob = (blob, name) => {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 2500);
};

// ===== 1. 全局唯一数据源：MOTOR_CONFIG =====
const MOTOR_CONFIG = Object.freeze({
  model: 'Y90S-4',
  name: 'Y90S-4 型三相异步电动机',
  series: 'Y 系列 · 全封闭自扇冷鼠笼式',
  ratedPowerKW: 1.1,
  ratedVoltageV: 380,
  connection: '△',
  ratedCurrentA: 2.2,
  ratedSpeedRpm: 1400,
  synchronousSpeedRpm: 1500,
  poles: 4,
  polePairs: 2,
  frequencyHz: 50,
  ratedSlip: (1500 - 1400) / 1500,
  ratedTorqueNm: 9550 * 1.1 / 1400,
  maxTorqueFactor: 2.2,
  startTorqueFactor: 1.8,
  startCurrentFactor: 5.6,
  breakdownSlip: 0.20,
  statorSlots: 24,
  rotorSlots: 22,
  airGapMm: 0.35,
  bearingFront: '6205-2Z',
  bearingRear: '6204-2Z',
  powerFactor: 0.78,
  efficiency: 0.79,
  protection: 'IP44',
  duty: 'S1',
  sampleHz: 50,
  windowSize: 100,
  ambientC: 25,
  steadyCaseC: 55,
  thermalTauS: 45,
  thresholds: Object.freeze({
    currentTripA: 3.5,
    currentWarnA: 3.2,
    speedTripRpm: 100,
    speedWarnRpm: 350,
    tempTripC: 75,
    tempWarnC: 65,
    vibWarnG: 0.16,
    vibTripG: 0.30
  })
});
const MC = MOTOR_CONFIG;
const SYNC_RPM = 60 * MC.frequencyHz / MC.polePairs; // 1500
const SYNC_OMEGA = SYNC_RPM * RPM2RAD;
const RATED_LOAD_TORQUE = MC.ratedTorqueNm;
const FRICTION_TORQUE = 0.22;
const ROTOR_INERTIA = 0.085; // kg·m², teaching-scale effective inertia
const DISPLAY_AUTO_RPM = 30;
const DISPLAY_ULTRA_RPM = 12;

const SAMPLE_DT = 1 / MC.sampleHz;
const SAMPLE_WINDOW = MC.windowSize;
const FEATURE_LABELS = {
  currentMean: '电流均值',
  currentVar: '电流方差',
  currentKurt: '电流峭度',
  speedRate: '转速变化率',
  followError: '跟随误差',
  tempSlope: '温度斜率'
};

// Standardized torque curve: Kloss-like shape, extended at high slip for a
// deep-bar / double-cage starting characteristic. Rated point is exact.
const TORQUE_CURVE = createMonotoneCurve(
  [0, 0.004, 0.015, 0.04, MC.ratedSlip, 0.12, 0.20, 0.35, 0.55, 0.75, 1.0],
  [0, 0.11, 0.34, 0.81, 1.0293, 1.58, 2.20, 2.10, 1.96, 1.87, 1.80]
);
// Current characteristic: no-load component + rotor component; values are
// calibrated to 2.2 A at rated slip and 5.5 A at locked rotor (teaching).
const CURRENT_CURVE = createMonotoneCurve(
  [0, 0.004, 0.015, 0.04, MC.ratedSlip, 0.12, 0.20, 0.35, 0.55, 0.75, 1.0],
  [0.62, 0.68, 0.86, 1.55, 2.20, 3.10, 3.85, 4.30, 4.78, 5.16, 5.50]
);

// ===== 2. 曲线插值 =====
function createMonotoneCurve(xs, ys) {
  const n = xs.length;
  const h = new Array(n - 1);
  const delta = new Array(n - 1);
  for (let i = 0; i < n - 1; i++) {
    h[i] = xs[i + 1] - xs[i];
    delta[i] = (ys[i + 1] - ys[i]) / h[i];
  }
  const m = new Array(n).fill(0);
  m[0] = delta[0];
  m[n - 1] = delta[n - 2];
  for (let i = 1; i < n - 1; i++) {
    if (delta[i - 1] * delta[i] <= 0) {
      m[i] = 0;
    } else {
      const w1 = 2 * h[i] + h[i - 1];
      const w2 = h[i] + 2 * h[i - 1];
      m[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
    }
  }
  return (x) => {
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (xs[mid] > x) hi = mid; else lo = mid;
    }
    const t = (x - xs[lo]) / h[lo];
    const t2 = t * t, t3 = t2 * t;
    const h00 = 2 * t3 - 3 * t2 + 1;
    const h10 = t3 - 2 * t2 + t;
    const h01 = -2 * t3 + 3 * t2;
    const h11 = t3 - t2;
    return h00 * ys[lo] + h10 * h[lo] * m[lo] + h01 * ys[lo + 1] + h11 * h[lo] * m[lo + 1];
  };
}

const torqueAtSlip = (s) => MC.ratedTorqueNm * TORQUE_CURVE(clamp(s, 0, 1));
const currentAtSlip = (s) => CURRENT_CURVE(clamp(s, 0, 1));

// ===== 3. 传感器定义（同一份数据源驱动 3D、右侧面板与底部状态带） =====
const SENSORS = Object.freeze([
  {
    id: 'hall', idx: '01', code: 'CT-H01', kind: 'current',
    name: '霍尔电流传感器', en: 'HALL CURRENT SENSOR', short: '电流',
    unit: 'A', range: '0–20 A · ±1.0% F.S.', normalText: '≈ 2.2 A',
    position: '接线盒出线电缆 · 穿芯式安装',
    principle: '霍尔元件测量电缆磁通，线性换算为定子线电流。',
    part: '定子绕组', partId: 'windings',
    hints: ['定子绕组', '电源电压', '机械负载'],
    camera: { pos: [2.70, 1.65, 3.45], target: [0.78, 0.40, 0.36] },
    anchor: [0.99, 0.56, 0.35], color: '#70d8e5'
  },
  {
    id: 'encoder', idx: '02', code: 'ENC-02', kind: 'speed',
    name: '增量式编码器', en: 'INCREMENTAL ENCODER', short: '转速',
    unit: 'r/min', range: '0–3000 r/min · ±1 pulse', normalText: '≈ 1400 r/min',
    position: '输出轴轴端 · 联轴器同轴安装',
    principle: '码盘随轴旋转产生 A/B 正交脉冲，单位时间计数得到实际转速。',
    part: '转子 · 轴系', partId: 'shaft',
    hints: ['转子', '轴承', '联轴器 / 负载'],
    camera: { pos: [2.25, 1.35, 4.80], target: [0, 0.10, 2.35] },
    anchor: [0, 0.22, 2.44], color: '#70d8e5'
  },
  {
    id: 'pt100', idx: '03', code: 'RTD-03', kind: 'temp',
    name: 'PT100 贴片温度传感器', en: 'PT100 SURFACE RTD', short: '温度',
    unit: '℃', range: '−40–180 ℃ · ±0.3 ℃', normalText: '≈ 55 ℃',
    position: '机座上 · 定子铁芯对应位置',
    principle: '铂电阻随温度线性变化，三线制消除引线电阻。',
    part: '定子铁芯 · 绕组', partId: 'statorCore',
    hints: ['定子铁芯', '绕组绝缘', '冷却条件'],
    camera: { pos: [2.80, 2.25, 3.10], target: [0.36, 0.78, -0.18] },
    anchor: [0.36, 0.78, -0.18], color: '#ffab5e'
  },
  {
    id: 'vib', idx: '04', code: 'VIB-04', kind: 'vibration',
    name: '振动加速度传感器', en: 'PIEZO ACCELEROMETER', short: '振动',
    unit: 'g RMS', range: '0–16 g · 10 kHz BW', normalText: '≈ 0.05 g',
    position: '前端盖轴承室正上方 · 磁吸底座',
    principle: '压电陶瓷受振动产生电荷，积分/均方根得到振动烈度。',
    part: '前端轴承', partId: 'bearingFront',
    hints: ['轴承滚道', '转子动平衡', '转子导条'],
    camera: { pos: [2.25, 2.20, 3.90], target: [0.34, 0.56, 1.18] },
    anchor: [0.35, 0.64, 1.28], color: '#ffab5e'
  },
  {
    id: 'photo', idx: '05', code: 'PH-05', kind: 'pulse',
    name: '对射式光电开关', en: 'THROUGH-BEAM PHOTO SWITCH', short: '通断脉冲',
    unit: 'p/s', range: '0–500 Hz · 1 ms', normalText: '均匀脉冲',
    position: '输出轴测码盘 · 随轴旋转',
    principle: '测码盘遮光/透光产生方波；脉冲消失即判断轴系停转。',
    part: '转子 · 轴系', partId: 'shaft',
    hints: ['转轴', '联轴器', '卡滞 / 堵转'],
    camera: { pos: [2.40, 1.70, 4.30], target: [0, 0.08, 2.12] },
    anchor: [0.05, 0.28, 2.16], color: '#e45a52'
  }
]);
const SENSOR_MAP = Object.fromEntries(SENSORS.map((s) => [s.id, s]));

// ===== 4. 故障与部件语义 =====
const FAULT_META = Object.freeze({
  normal: { name: '正常运行', en: 'HEALTHY', desc: '额定负载下稳定运行，磁场与转子保持滑差。', hint: '额定负载，稳定运行' },
  stall: { name: '卡滞堵转', en: 'JAM / LOCKED ROTOR', desc: '输出轴楔入异物，转子停转；定子磁场仍以同步速旋转。', hint: '楔块卡入输出轴，转子堵转' },
  bearing: { name: '轴承磨损', en: 'BEARING WEAR', desc: '前轴承滚道出现周期性冲击，振动 RMS 上升。', hint: '前端轴承磨损，振动冲击' },
  broken: { name: '转子断条', en: 'BROKEN BAR', desc: '两根鼠笼导条断裂，电流出现 2sf 边频分量。', hint: '2 根导条断裂，电流脉动' },
  overheat: { name: '绕组过热', en: 'WINDING OVERHEAT', desc: '绕组匝间短路导致损耗增加，温升具有热惯性。', hint: '绕组损耗增加，温度上升' },
  overload: { name: '过载运行', en: 'OVERLOAD', desc: '负载接近额定上限，转速沿 T(s) 下滑，电流接近预警。', hint: '负载高于额定，工作点下滑' }
});

const PART_META = {
  frame: {
    name: '铸铁机座', en: 'CAST-IRON FRAME', type: 'STRUCTURE', material: 'HT200 铸铁',
    desc: '承载定子铁芯与端盖，周向散热筋扩大表面换热面积。',
    specs: ['HT200', '周向散热筋', 'IP44'], focus: [4.2, 2.7, 3.4], target: [0, 0, 0.1]
  },
  base: {
    name: '一体式底座', en: 'INTEGRAL MOUNTING BASE', type: 'STRUCTURE', material: 'HT200 铸铁',
    desc: '四点安装地脚，带机加工安装孔，与基础刚性连接并抑制振动。',
    specs: ['4 × Ø10', '机加工结合面', '减振安装'], focus: [3.7, 1.8, 4.4], target: [0, -0.75, 0]
  },
  frontCap: {
    name: '前端盖', en: 'DRIVE-END END SHIELD', type: 'STRUCTURE', material: 'HT200 铸铁',
    desc: '以止口定位定子机座，内设轴承室与油封位，承受径向载荷。',
    specs: ['止口定位', '轴承室 6205', '6 × 螺栓'], focus: [3.5, 2.0, 4.8], target: [0, 0, 1.1]
  },
  rearCap: {
    name: '后端盖', en: 'NON-DRIVE END SHIELD', type: 'STRUCTURE', material: 'HT200 铸铁',
    desc: '封闭轴承室并将风扇与定子腔隔离。',
    specs: ['止口定位', '轴承室 6204', '风路导流'], focus: [-3.2, 2.0, -4.6], target: [0, 0, -1.1]
  },
  fanCowl: {
    name: '风罩', en: 'FAN COVER', type: 'COOLING', material: '冲压钢板 / 喷涂',
    desc: '径向防护格栅，防止异物进入旋转风扇，内部形成强制风冷通道。',
    specs: ['径向格栅', '冷轧钢', '防尘'], focus: [-3.9, 2.2, -4.7], target: [0, 0, -1.85]
  },
  fan: {
    name: '径向风扇', en: 'RADIAL COOLING FAN', type: 'COOLING', material: '铸铝 / 工程塑料',
    desc: '与转子同轴同速旋转，将空气径向甩向机座散热筋。',
    specs: ['8 叶片', '同轴同速', '径向离心'], focus: [-3.4, 2.1, -4.8], target: [0, 0, -1.45]
  },
  statorCore: {
    name: '定子铁芯', en: 'STATOR LAMINATED CORE', type: 'MAGNETIC', material: '冷轧硅钢片',
    desc: '0.5 mm 硅钢片叠压而成，24 槽内嵌三相绕组，构成旋转磁场磁路。',
    specs: ['24 槽', '硅钢片叠压', '气隙 0.35 mm'], focus: [3.2, 2.2, 4.2], target: [0, 0.15, 0]
  },
  windings: {
    name: '三相定子绕组', en: 'THREE-PHASE WINDINGS', type: 'ELECTRICAL', material: '聚酯漆包铜线',
    desc: 'U / V / W 三相绕组在空间错开 120° 电角度，通入三相电流后产生 4 极旋转磁场。',
    specs: ['U / V / W', '△ 380 V', 'Y 接降压启动'], focus: [2.7, 2.0, 3.9], target: [0, 0.1, 0.6]
  },
  rotorCore: {
    name: '转子铁芯', en: 'ROTOR LAMINATED CORE', type: 'MAGNETIC', material: '冷轧硅钢片',
    desc: '22 槽铁芯与定子同轴，外圆开设鼠笼导条槽，与定子之间保留气隙。',
    specs: ['22 槽', '气隙 0.35 mm', '硅钢叠片'], focus: [3.3, 2.0, 4.5], target: [0, 0, 0]
  },
  rotorBars: {
    name: '鼠笼导条 / 端环', en: 'SQUIRREL-CAGE BARS & RINGS', type: 'ELECTRICAL', material: '纯铜 / 铝青铜',
    desc: '22 根轴向导条与两端短路环构成闭合鼠笼。导条切割气隙磁场，感应电流与磁场作用产生电磁转矩。',
    specs: ['22 根导条', '两端短路环', '感应电流'], focus: [3.1, 1.8, 4.4], target: [0, 0, 0.2]
  },
  shaft: {
    name: '电机转轴', en: 'MOTOR SHAFT', type: 'MECHANICAL', material: '45 钢 / 调质',
    desc: '贯穿转子铁芯，前后轴承位与键槽经磨削加工，将电磁转矩输出至负载。',
    specs: ['45 钢', '键槽输出', '前后轴承位'], focus: [2.7, 1.6, 4.6], target: [0, 0, 1.5]
  },
  bearingFront: {
    name: '前端深沟球轴承', en: 'FRONT BALL BEARING 6205-2Z', type: 'BEARING', material: 'GCr15 轴承钢',
    desc: '双面防尘深沟球轴承，承受径向载荷并保持转子轴线定位。',
    specs: ['6205-2Z', 'GCr15', '双面防尘'], focus: [2.6, 1.8, 4.3], target: [0, 0, 1.02]
  },
  bearingRear: {
    name: '后端深沟球轴承', en: 'REAR BALL BEARING 6204-2Z', type: 'BEARING', material: 'GCr15 轴承钢',
    desc: '后轴承采用游隙配合，补偿轴系热伸长。',
    specs: ['6204-2Z', 'GCr15', '热伸长补偿'], focus: [-2.8, 1.7, -4.1], target: [0, 0, -1.02]
  },
  terminal: {
    name: '接线盒', en: 'TERMINAL BOX', type: 'ELECTRICAL', material: '铸铝 / 黄铜端子',
    desc: '内置 6 个接线柱，可通过连片切换 △ / Y 接法；盒盖可拆。',
    specs: ['6 端子', '△ / Y', 'IP55 盒盖'], focus: [1.8, 3.5, 2.7], target: [0, 1.0, 0.1]
  },
  nameplate: {
    name: '电机铭牌', en: 'RATED NAMEPLATE', type: 'IDENTITY', material: '拉丝铝 / 蚀刻',
    desc: '标注型号、额定功率、电压、电流、转速、接法、功率因数与工作制。',
    specs: ['Y90S-4', '1.1 kW', '380 V △'], focus: [2.9, 1.0, 2.5], target: [0.86, -0.04, 0.55]
  },
  hall: {
    name: '霍尔电流传感器', en: 'HALL CURRENT SENSOR', type: 'SENSOR', material: '工程塑料 / 铁氧体',
    desc: '穿芯安装在接线盒出线电缆上，测量定子线电流。',
    specs: ['CT-H01', '0–20 A', '±1.0% F.S.'], focus: [2.7, 1.65, 3.45], target: [0.85, 0.4, 0.35]
  },
  encoder: {
    name: '增量式编码器', en: 'INCREMENTAL ENCODER', type: 'SENSOR', material: '阳极氧化铝 / 工程塑料',
    desc: '通过联轴器与输出轴同轴安装，输出 A/B 正交脉冲测量转速。',
    specs: ['ENC-02', '0–3000 r/min', 'A/B · Z'], focus: [2.2, 1.3, 4.8], target: [0, 0.1, 2.35]
  },
  pt100: {
    name: 'PT100 贴片温度传感器', en: 'PT100 SURFACE RTD', type: 'SENSOR', material: '不锈钢探头 / 硅胶',
    desc: '贴在机座定子铁芯对应位置，铂电阻三线制测量壳体温度。',
    specs: ['RTD-03', '−40–180 ℃', '±0.3 ℃'], focus: [2.6, 2.2, 3.1], target: [0.36, 0.78, -0.18]
  },
  vib: {
    name: '振动加速度传感器', en: 'PIEZO ACCELEROMETER', type: 'SENSOR', material: '不锈钢 / 磁吸底座',
    desc: '磁吸于前端盖轴承室上方，监测滚道冲击、转子不平衡与断条振动。',
    specs: ['VIB-04', '10 kHz BW', '0–16 g'], focus: [2.2, 2.2, 3.9], target: [0.35, 0.64, 1.28]
  },
  photo: {
    name: '对射式光电开关', en: 'THROUGH-BEAM PHOTO SWITCH', type: 'SENSOR', material: '工程塑料 / 红外 LED',
    desc: '卡在旋转测码盘上，通过遮光脉冲判断轴系是否停转。',
    specs: ['PH-05', '0–500 Hz', '1 ms'], focus: [2.4, 1.7, 4.3], target: [0, 0.08, 2.12]
  },
  daq: {
    name: '数据采集卡', en: 'DATA ACQUISITION UNIT', type: 'INSTRUMENT', material: '阳极氧化铝',
    desc: '将五路传感器信号同步采样至 50 Hz，经滤波和归一化后送入特征提取。',
    specs: ['DAQ-01', '5 CH', '50 Hz'], focus: [3.8, 1.4, 3.4], target: [1.5, -0.8, 0.55]
  }
};

// ===== 5. 应用状态 =====
const state = {
  simTime: 0,
  powerOn: true,
  runCmd: true,
  fieldOn: true,
  tripped: false,
  fault: 'normal',
  omega: SYNC_RPM * (1 - MC.ratedSlip) * RPM2RAD,
  tempC: MC.steadyCaseC,
  thermalCurrent: MC.ratedCurrentA,
  loadPct: 100,
  loadTarget: 100,
  currentA: MC.ratedCurrentA,
  currentSS: MC.ratedCurrentA,
  voltage: MC.ratedVoltageV,
  slip: MC.ratedSlip,
  nRpm: MC.ratedSpeedRpm,
  torqueNm: RATED_LOAD_TORQUE,
  powerKW: RATED_LOAD_TORQUE * 1400 / 9550,
  emTorqueNm: torqueAtSlip(MC.ratedSlip),
  vibG: 0.05,
  sinceStart: 100,
  startTransient: 0,
  jam: 0,
  bearingExtraDrag: 0,
  faultCurrentOffset: 0,
  photoFreq: 6,
  lastFieldOn: true,
  contactorTransient: 1,
  fieldAlpha: 1,
  alarm: null,
  alarmLog: [],
  alarmCounters: { current: 0, speed: 0, temp: 0 },
  lastAlarmType: '',
  protectionTimer: 0,
  lastSampleTime: 0,
  lastUiUpdate: -1,
  lastChartUpdate: -1,
  lastFeatureUpdate: -1,
  feature: {
    currentMean: MC.ratedCurrentA, currentVar: 0.00002, currentKurt: 3.0,
    speedRate: 0, followError: 0, tempSlope: 0
  },
  svmScore: -1.8,
  svmConfidence: 0.95,
  svmNormal: true,
  faultLabel: '',
  sampleBuffer: [],
  trend: [],
  sensorHistory: { hall: [], encoder: [], pt100: [], vib: [], photo: [] },
  photoEdges: 0,
  lastPulseState: 0,
  displayRotorAngle: 0,
  displayFieldAngle: 0,
  displayScale: DISPLAY_AUTO_RPM / MC.ratedSpeedRpm,
  rotorDisplayRpm: DISPLAY_AUTO_RPM,
  fieldDisplayRpm: 1500 * DISPLAY_AUTO_RPM / MC.ratedSpeedRpm,
  autoStepIndex: 0,
  statusKey: 'rated'
};
const ui = {
  paused: false,
  observation: 'auto',
  observationBeforeField: 'auto',
  fieldViewActive: false,
  manualScale: 0.04,
  structure: 'assembly',
  explodeAmount: 0,
  explodeTarget: 0,
  explodeAnimating: false,
  preExplodePaused: false,
  rotorExtract: false,
  fieldVisible: false,
  barsVisible: true,
  airflow: false,
  annotations: true,
  selectedSensor: null,
  selectedPart: null,
  terminalOpen: false,
  terminalLid: 0,
  connection: 'delta',
  quality: 'auto',
  audioEnabled: false,
  soundMode: 'condition',
  volume: 0.42,
  cameraPreset: 'hero',
  cinemaActive: false,
  cinemaStart: 0,
  cinemaStep: 0,
  guideStep: 0,
  guideVisible: false,
  focusMode: false,
  debug: false,
  reducedMotion: window.matchMedia?.('(prefers-reduced-motion: reduce)')?.matches ?? false,
  hoverSensor: null,
  hoverPart: null,
  uiRefreshNow: false,
  chartPaused: false,
  modalCardId: null,
  modalPaused: false,
  atlasOpen: false,
  hoverX: null,
  lastFrameTime: 0,
  fps: 60,
  qualityDowngraded: false,
  alarmFlash: 0
};

// ===== 6. 派生量 / 物理模型 =====
function slipFromOmega(omega) {
  return clamp((SYNC_OMEGA - omega) / SYNC_OMEGA, 0, 1);
}
function omegaFromSlip(s) {
  return SYNC_OMEGA * (1 - clamp(s, 0, 1));
}
function solveEquilibriumSlip(targetTorque) {
  let lo = 0.0001, hi = MC.breakdownSlip;
  for (let i = 0; i < 28; i++) {
    const mid = (lo + hi) / 2;
    if (torqueAtSlip(mid) < targetTorque) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}
function expectedSteadySpeed(loadTorque) {
  const s = solveEquilibriumSlip(loadTorque + FRICTION_TORQUE);
  return SYNC_RPM * (1 - s);
}
function faultTorqueOffset(t) {
  switch (state.fault) {
    case 'bearing': return -0.32 - 0.10 * Math.sin(TAU * 2.7 * t);
    case 'broken': return -0.25 + 0.58 * Math.sin(TAU * 2 * Math.max(0.001, state.slip) * MC.frequencyHz * t);
    case 'overheat': return -0.10;
    default: return 0;
  }
}
function targetJamTorque() {
  return state.fault === 'stall' ? 28 : 0;
}
function targetFieldOn() {
  return state.powerOn && state.runCmd && !state.tripped;
}

function updatePhysics(dtWall) {
  const dt = clamp(dtWall, 0, 1 / 30);
  state.simTime += dt;
  state.sinceStart += dt;

  // Smooth (but physically integrated) load command
  state.loadPct = damp(state.loadPct, state.loadTarget, 4.2, dt);

  const wasFieldOn = state.fieldOn;
  state.fieldOn = targetFieldOn();
  if (state.fieldOn && !wasFieldOn) {
    state.sinceStart = 0;
    state.startTransient = 7.6;
    audioEngine?.powerOn();
  }
  if (!state.fieldOn && wasFieldOn) {
    audioEngine?.powerOff();
  }
  state.lastFieldOn = state.fieldOn;
  state.startTransient *= Math.exp(-dt / 0.28);

  // Main contactor / field angle identity is kept globally.
  const s = slipFromOmega(state.omega);
  state.slip = s;
  let Te = 0;
  if (state.fieldOn) {
    Te = torqueAtSlip(s) + (s > MC.breakdownSlip ? (s - MC.breakdownSlip) * 0.4 : 0);
  }
  state.emTorqueNm = Te;

  // Jam torque ramps into the shaft (wedge = high brake torque)
  const jamTarget = targetJamTorque();
  state.jam = damp(state.jam, jamTarget, jamTarget > state.jam ? 22 : 4, dt);
  if (Math.abs(state.jam) < 0.02) state.jam = 0;

  const loadTorque = (state.loadPct / 100) * RATED_LOAD_TORQUE;
  const friction = (state.omega > 0.03 ? FRICTION_TORQUE : 0) + (state.fieldOn ? 0 : 0);
  const speedRipple = state.fault === 'broken'
    ? 0.65 * Math.sin(TAU * 2 * Math.max(0.001, s) * MC.frequencyHz * state.simTime)
    : state.fault === 'bearing'
      ? 0.16 * Math.sin(TAU * 2.7 * state.simTime)
      : 0;
  const faultOffset = faultTorqueOffset(state.simTime);
  const netTorque = Te - loadTorque - friction - state.jam + faultOffset + speedRipple;

  if (state.fieldOn || Math.abs(state.omega) > 0) {
    state.omega += (netTorque / ROTOR_INERTIA) * dt;
  }
  if (state.omega < 0) state.omega = 0;
  if (state.omega > SYNC_OMEGA) state.omega = SYNC_OMEGA;
  if (state.omega < 0.22 && Te < loadTorque + friction + state.jam + 0.5) state.omega = 0;

  state.nRpm = state.omega * RAD2RPM;
  state.slip = slipFromOmega(state.omega);

  // Stator current model — no independent random noise generator.
  let I = 0;
  let thermalCore = 0;
  if (state.fieldOn) {
    const baseCurrent = currentAtSlip(state.slip);
    I = baseCurrent;
    thermalCore = baseCurrent;
    if (state.sinceStart < 4.5) I += state.startTransient;
    // deterministic, low-level supply / slot-harmonic ripple plus seeded
    // 50 Hz measurement noise (same sequence for every run)
    I += 0.0035 * Math.sin(TAU * 0.34 * state.simTime) + 0.0025 * Math.sin(TAU * 1.17 * state.simTime + 1.2);
    I += MEASUREMENT_NOISE[measurementNoiseIndex++ % MEASUREMENT_NOISE.length] * 0.0105;
    if (state.fault === 'bearing') {
      I += 0.46 + 0.07 * Math.sin(TAU * 6.3 * state.simTime) * (0.55 + 0.45 * Math.sin(TAU * 1.1 * state.simTime));
      I += 0.40 * Math.pow(Math.max(0, Math.sin(TAU * 6.3 * state.simTime)), 14);
      thermalCore += 0.46;
    } else if (state.fault === 'broken') {
      I += 0.38 + 0.17 * Math.sin(TAU * 2 * Math.max(0.001, state.slip) * MC.frequencyHz * state.simTime)
        + 0.06 * Math.sin(TAU * 0.61 * state.simTime);
      I += 0.46 * Math.pow(Math.max(0, Math.sin(TAU * 2 * Math.max(0.001, state.slip) * MC.frequencyHz * state.simTime)), 10);
      thermalCore += 0.38;
    } else if (state.fault === 'overheat') {
      const hotOffset = 0.34 + 0.24 * clamp((state.tempC - 55) / 30, 0, 1.2);
      I += hotOffset;
      thermalCore += hotOffset;
    } else if (state.fault === 'stall') {
      I += 0.18 * Math.sin(TAU * 4.2 * state.simTime) + 0.06 * Math.sin(TAU * 11.3 * state.simTime + 0.7);
      const impact = Math.pow(Math.max(0, Math.sin(TAU * 3.7 * state.simTime + 0.6)), 12);
      I += impact * 0.72;
      thermalCore += 0.15;
    }
    // Starting inrush is intentionally a short measured-current transient.
    if (state.fault === 'stall' && state.slip > 0.97 && state.sinceStart > 1.5) {
      I = Math.max(I, 5.05 + 0.20 * Math.sin(TAU * 4.2 * state.simTime));
    }
  }
  I = Math.max(0, I);
  state.currentSS = currentAtSlip(state.slip);
  state.currentA = I;

  // Thermal model: first-order inertia, copper-dominated loss, τ = 45 s.
  // A short thermal current filter keeps the 5–6× starting transient from
  // faking a temperature rise; sustained stall current still heats the winding.
  state.thermalCurrent = damp(state.thermalCurrent, thermalCore, 0.35, dt);
  const runningLoss = state.fieldOn
    ? 0.62 * Math.pow(state.thermalCurrent / MC.ratedCurrentA, 2) + 0.38 * Math.pow(state.nRpm / MC.ratedSpeedRpm, 2)
    : 0;
  const overheatLoss = state.fieldOn && state.fault === 'overheat' ? 0.95 : 0;
  const targetExcess = 30 * (runningLoss + overheatLoss);
  const thermalScale = $('#thermalAccel')?.checked ? 10 : 1;
  const dTemp = ((targetExcess - (state.tempC - MC.ambientC)) / MC.thermalTauS) * thermalScale;
  state.tempC += dTemp * dt;
  state.tempC = clamp(state.tempC, MC.ambientC - 2, 180);

  // Vibration RMS (deterministic fault signatures)
  let vib = 0.045 + 0.006 * Math.sin(TAU * 0.21 * state.simTime) + 0.004 * Math.sin(TAU * 1.3 * state.simTime);
  if (!state.fieldOn) vib = 0.012 + 0.002 * Math.sin(TAU * 0.2 * state.simTime);
  if (state.fieldOn && state.fault === 'bearing') {
    const impact = Math.pow(Math.max(0, Math.sin(TAU * 7.4 * state.simTime)), 12);
    vib += 0.22 + 0.18 * impact;
  } else if (state.fieldOn && state.fault === 'broken') {
    vib += 0.065 + 0.03 * Math.sin(TAU * 2 * state.slip * MC.frequencyHz * state.simTime);
  } else if (state.fieldOn && state.fault === 'stall') {
    vib += 0.14 + 0.06 * Math.sin(TAU * 4.1 * state.simTime);
  } else if (state.fieldOn && state.loadPct > 108) {
    vib += 0.025;
  }
  state.vibG = Math.max(0, vib);

  // Output quantities, explicitly NOT net accelerating torque.
  const outputTorque = state.fieldOn
    ? (state.omega > 1.2 ? loadTorque : Te)
    : 0;
  state.torqueNm = Math.max(0, outputTorque);
  state.powerKW = state.torqueNm * state.nRpm / 9550;

  state.fieldAlpha = damp(state.fieldAlpha, state.fieldOn ? 1 : 0, 7, dt);
  updateAlarmLogic(dt);
  updatePhotoSensor(dt);
}

function getAlarmSnapshot() {
  const now = new Date();
  return `${pad2(now.getHours())}:${pad2(now.getMinutes())}:${pad2(now.getSeconds())}`;
}

function updateAlarmLogic(dt) {
  const th = MC.thresholds;
  // Inhibit current/speed protection during the first 3.8 s after energisation;
  // the 5–6× starting inrush is normal and must not trip the drive.
  const activeRun = state.fieldOn && state.sinceStart > 3.8;
  const counters = state.alarmCounters;
  counters.current = (state.currentA > th.currentTripA && activeRun) ? counters.current + dt : 0;
  counters.speed = (state.nRpm < th.speedTripRpm && activeRun) ? counters.speed + dt : 0;
  counters.temp = (state.tempC > th.tempTripC) ? counters.temp + dt : 0;

  if (!state.alarm) {
    let type = null, text = '', threshold = '', reading = '';
    if (counters.current > 0.75) {
      type = 'stall'; text = '检测到卡滞 / 过流故障'; threshold = `I > ${th.currentTripA.toFixed(1)} A`;
      reading = `${state.currentA.toFixed(2)} A`;
    } else if (counters.speed > 1.1) {
      type = 'stall'; text = '检测到转子堵转'; threshold = `n < ${th.speedTripRpm} r/min`;
      reading = `${state.nRpm.toFixed(0)} r/min`;
    } else if (counters.temp > 1.0) {
      type = 'overheat'; text = '检测到壳体超温'; threshold = `T > ${th.tempTripC} ℃`;
      reading = `${state.tempC.toFixed(1)} ℃`;
    }
    if (type) {
      state.alarm = {
        type, text, threshold, reading,
        time: nowClock(), countdown: 4.2, executed: false,
        svm: state.svmNormal ? '正常类' : state.faultLabel || '故障类'
      };
      state.alarmLog.unshift({
        time: nowClock(), type, threshold, reading,
        svm: state.svmNormal ? '正常类' : (state.faultLabel || '故障类'),
        executed: false
      });
      state.alarmLog = state.alarmLog.slice(0, 18);
      dismissGuide();
      showToast(`${text} · 保护延时 4.2 s`, 'alarm');
      audioEngine?.alarmOn();
    }
  }

  if (state.alarm && !state.alarm.executed) {
    state.alarm.countdown = Math.max(0, state.alarm.countdown - dt);
    if (state.alarm.countdown <= 0) {
      state.alarm.executed = true;
      state.tripped = true;
      state.runCmd = false;
      state.protectionTimer = 0;
      const log = state.alarmLog[0];
      if (log) log.executed = true;
      showToast('保护停机已执行 · 请排除故障后复位', 'alarm');
      audioEngine?.protectionStop();
    }
  }
}

function updatePhotoSensor(dt) {
  // A twelve-slot code disc. Pulses are counted from the *display* mechanism angle
  // so the teaching HUD remains synchronized with the slow-motion rotor.
  const rate = ui.paused ? 0 : state.rotorDisplayRpm;
  const freq = rate / 60 * 12;
  state.photoFreq = freq;
  if (freq > 0.05) {
    const slot = Math.floor(state.displayRotorAngle / (TAU / 12));
    if (slot !== state.lastPulseState) {
      state.photoEdges += Math.abs(slot - state.lastPulseState);
      state.lastPulseState = slot;
    }
  }
}

// ===== 7. 50 Hz sampling / 2 s feature window =====
let sampleAccumulator = 0;
function pushSample(force = false) {
  const t = state.simTime;
  const s = {
    t,
    I: state.currentA,
    n: state.nRpm,
    T: state.tempC,
    vib: state.vibG,
    photo: state.photoFreq > 0.05 ? 1 : 0,
    fieldOn: state.fieldOn,
    slip: state.slip
  };
  state.sampleBuffer.push(s);
  if (state.sampleBuffer.length > SAMPLE_WINDOW) state.sampleBuffer.shift();

  if (force || sampleAccumulator >= SAMPLE_DT) {
    const h = state.sensorHistory;
    h.hall.push(s.I); if (h.hall.length > 160) h.hall.shift();
    h.encoder.push(s.n); if (h.encoder.length > 160) h.encoder.shift();
    h.pt100.push(s.T); if (h.pt100.length > 160) h.pt100.shift();
    h.vib.push(s.vib); if (h.vib.length > 160) h.vib.shift();
    h.photo.push(s.photo); if (h.photo.length > 160) h.photo.shift();
    sampleAccumulator = 0;
  }
}

function accumulateSamples(dtWall) {
  sampleAccumulator += dtWall;
  while (sampleAccumulator >= SAMPLE_DT) {
    pushSample(true);
    sampleAccumulator -= SAMPLE_DT;
  }
  if (state.trend.length === 0 || state.simTime - state.trend[state.trend.length - 1].t >= 0.10) {
    state.trend.push({ t: state.simTime, I: state.currentA, n: state.nRpm, T: state.tempC });
    while (state.trend.length > 780) state.trend.shift();
  }
}

function computeFeatures() {
  const win = state.sampleBuffer;
  if (win.length < 6) return;
  let sumI = 0, sumN = 0, sumT = 0;
  for (const p of win) { sumI += p.I; sumN += p.n; sumT += p.T; }
  const meanI = sumI / win.length, meanN = sumN / win.length, meanT = sumT / win.length;
  let varI = 0, m4 = 0;
  for (const p of win) {
    const d = p.I - meanI;
    varI += d * d;
    m4 += d * d * d * d;
  }
  varI /= win.length;
  m4 /= win.length;
  const kurt = varI > 1e-10 ? m4 / (varI * varI) : 3;

  // short-window slope for speed response (still within the 2 s buffer)
  const slopeWin = win.slice(Math.max(0, win.length - 30));
  let sx = 0, sy = 0, sxx = 0, sxy = 0;
  const t0 = slopeWin[0]?.t ?? 0;
  for (const p of slopeWin) {
    const x = p.t - t0;
    sx += x; sy += p.n; sxx += x * x; sxy += x * p.n;
  }
  const N = slopeWin.length;
  const denom = N * sxx - sx * sx;
  const speedRate = denom > 1e-9 ? (N * sxy - sx * sy) / denom : 0;

  // full 2 s temperature slope, °C/s
  let tSx = 0, tSy = 0, tSxx = 0, tSxy = 0;
  const T0 = win[0]?.t ?? 0;
  for (const p of win) {
    const x = p.t - T0;
    tSx += x; tSy += p.T; tSxx += x * x; tSxy += x * p.T;
  }
  const tDenom = win.length * tSxx - tSx * tSx;
  const tempSlope = tDenom > 1e-9 ? (win.length * tSxy - tSx * tSy) / tDenom : 0;

  const expected = expectedSteadySpeed((state.loadPct / 100) * RATED_LOAD_TORQUE);
  const followError = Math.abs(state.nRpm - expected);

  state.feature = {
    currentMean: meanI,
    currentVar: varI,
    currentKurt: kurt,
    speedRate,
    followError,
    tempSlope
  };

  // linear soft-margin classifier trained once; 2D projection of a six-signal
  // diagnostic feature vector, axes intentionally current mean × speed rate.
  const fc = svmClassify(state.feature);
  state.svmScore = fc.f;
  state.svmConfidence = fc.confidence;
  state.svmNormal = fc.normal;
  state.faultLabel = fc.normal ? '正常类' : classifyFaultName();

  // Keep a tiny feature trace for the SVM widget
  state.featureTrace = state.featureTrace || [];
  state.featureTrace.push({ x: meanI, y: speedRate });
  if (state.featureTrace.length > 24) state.featureTrace.shift();
}

function classifyFaultName() {
  const f = state.feature;
  // Prefer the known injected operating condition to keep the teaching verdict
  // legible; fall back to measured evidence when no condition is selected.
  if (state.fault === 'stall' || state.nRpm < 100) return '卡滞堵转';
  if (state.fault === 'bearing') return '轴承磨损';
  if (state.fault === 'broken') return '转子断条';
  if (state.fault === 'overheat') return '绕组过热';
  if (state.fault === 'overload') return '过载异常';
  if (f.tempSlope > 0.35 || state.tempC > 68) return '绕组过热';
  if (state.vibG > 0.18) return '轴承磨损';
  if (f.currentKurt > 4.2) return '转子断条';
  if (state.loadPct > 105) return '过载异常';
  return '故障倾向';
}

// ===== 8. 二维教学 SVM（soft-margin linear, fixed data set） =====
const svmModel = (() => {
  const rand = pseudoRandom(20261103);
  const training = [];
  // Healthy cluster: current mean 2.08–2.32 A, near-zero speed derivative.
  for (let i = 0; i < 46; i++) {
    training.push({
      x: 2.18 + gauss(rand) * 0.095,
      y: (rand() - 0.5) * 0.34,
      label: -1
    });
  }
  // Fault cluster: elevated current and/or negative speed derivative.
  for (let i = 0; i < 46; i++) {
    const mode = i % 4;
    let x, y;
    if (mode === 0) { x = 3.2 + rand() * 2.1; y = -1.7 + gauss(rand) * 0.8; }
    else if (mode === 1) { x = 2.55 + rand() * 0.55; y = -2.8 + gauss(rand) * 0.8; }
    else if (mode === 2) { x = 2.72 + rand() * 0.5; y = -1.1 + gauss(rand) * 0.7; }
    else { x = 2.5 + rand() * 0.95; y = -0.35 + gauss(rand) * 0.6; }
    training.push({ x, y, label: 1 });
  }

  // Feature normalization then soft-margin linear SVM via sub-gradient descent.
  const norm = (v) => [(v.x - 2.55) / 1.05, v.y / 2.2];
  const X = training.map(norm);
  let w = [0.9, -0.3], b = -0.2;
  const lr = 0.055, C = 1.4, epochs = 1400;
  for (let epoch = 0; epoch < epochs; epoch++) {
    const idx = epoch % X.length;
    const margin = training[idx].label * (w[0] * X[idx][0] + w[1] * X[idx][1] + b);
    if (margin < 1) {
      w[0] += lr * (training[idx].label * X[idx][0] - 2 / C / X.length * w[0]);
      w[1] += lr * (training[idx].label * X[idx][1] - 2 / C / X.length * w[1]);
      b += lr * training[idx].label;
    } else {
      w[0] -= lr * (2 / C / X.length) * w[0];
      w[1] -= lr * (2 / C / X.length) * w[1];
    }
  }
  // calmer, visually stable boundary after training
  const normW = Math.hypot(w[0], w[1]) || 1;
  w = w.map((v) => v / normW);
  b = b / normW;
  const rawScore = (x, y) => {
    const z = norm({ x, y });
    return w[0] * z[0] + w[1] * z[1] + b;
  };
  // The scatter's 2D slice is placed so the healthy cluster (≈2.2 A) keeps a
  // margin and the injected faults (≥2.5 A plus speed transients) cross it.
  // This is an explicitly labelled teaching slice of the RBF model.
  const boundaryCurrent = 2.40;
  const boundaryZx = (boundaryCurrent - 2.55) / 1.05;
  b = -(w[0] * boundaryZx);
  const thresholdOffset = 0;
  const classify = (feature) => {
    const f = rawScore(feature.currentMean, feature.speedRate) + thresholdOffset;
    const faultProb = 1 / (1 + Math.exp(-clamp(f * 18, -18, 18)));
    const normal = f < 0;
    return {
      f,
      normal,
      confidence: clamp(normal ? 1 - faultProb : faultProb, 0.5, 0.995)
    };
  };
  // Compute support vectors on the 2D display data.
  const withScores = training.map((p) => ({ ...p, f: rawScore(p.x, p.y) }));
  const support = withScores
    .filter((p) => p.label * p.f < 1.15 && p.label * p.f > -0.4)
    .sort((a, b2) => Math.abs(a.f) - Math.abs(b2.f))
    .slice(0, 10);
  let correct = 0;
  for (const p of withScores) if ((p.f >= 0 ? 1 : -1) === p.label) correct++;
  const accuracy = correct / withScores.length;
  return {
    training: withScores,
    support,
    weights: w,
    bias: b,
    rawScore,
    classify,
    norm,
    accuracy
  };
})();

function svmClassify(feature) {
  return svmModel.classify(feature);
}

// ===== 9. 初始化趋势数据（开机即处于温热稳定状态） =====
function prefillTelemetry() {
  const start = -60;
  state.simTime = 0;
  state.trend.length = 0;
  for (let i = 0; i <= 600; i++) {
    const t = start + i * 0.1;
    const wiggle = 0.012 * Math.sin(i * 0.19) + 0.005 * Math.sin(i * 0.53 + 1.1);
    state.trend.push({
      t,
      I: 2.2 + wiggle,
      n: 1400 + 0.08 * Math.sin(i * 0.17),
      T: 55 + 0.015 * Math.sin(i * 0.08)
    });
  }
  state.sampleBuffer.length = 0;
  const rand = pseudoRandom(4182);
  for (let i = 0; i < SAMPLE_WINDOW; i++) {
    const t = -2 + i * SAMPLE_DT;
    const wiggle = 0.0035 * Math.sin(i * 0.3) + gauss(rand) * 0.0105;
    state.sampleBuffer.push({
      t,
      I: 2.2 + wiggle,
      n: 1400 + 0.06 * Math.sin(i * 0.12),
      T: 55 + 0.002 * i * 0.01,
      vib: 0.048,
      photo: 1,
      fieldOn: true,
      slip: MC.ratedSlip
    });
  }
  for (let i = 0; i < 80; i++) {
    const wiggle = gauss(rand) * 0.0095;
    state.sensorHistory.hall.push(2.2 + wiggle);
    state.sensorHistory.encoder.push(1400 + 0.07 * Math.sin(i * 0.2));
    state.sensorHistory.pt100.push(55 + 0.01 * Math.sin(i * 0.05));
    state.sensorHistory.vib.push(0.048 + 0.004 * Math.sin(i * 0.5));
    state.sensorHistory.photo.push(i % 4 < 2 ? 1 : 0);
  }
  computeFeatures();
}

// ============================================================================
// PART II — studio, materials, procedural geometry
// ============================================================================

// ===== 10. 场景 / 渲染器 / 摄影棚环境 =====
const GROUND_Y = -1.16;
const MOTOR = {
  bodyR: 0.82,
  bodyZ0: -1.03,
  bodyZ1: 1.03,
  bodyLen: 2.06,
  statorOuter: 0.66,
  statorInner: 0.43,
  statorLen: 1.55,
  rotorR: 0.39,
  rotorLen: 1.50,
  shaftR: 0.135,
  frontCapZ: 1.13,
  rearCapZ: -1.13,
  capR: 0.805,
  fanZ: -1.36,
  cowlZ: -1.80,
  cutStart: 120 * DEG2RAD,
  cutLength: 240 * DEG2RAD
};

let renderer = null;
let scene = null;
let camera = null;
let orbitRig = null;
let pmremGenerator = null;
let environmentTexture = null;
let keyLight = null;
let rimLight = null;
let fillLight = null;
let alarmLight = null;
let ground = null;
let dust = null;
let stageElement = null;
let canvasElement = null;
let stageW = 1000;
let stageH = 520;
let currentPixelRatio = 1;
let environmentGenerated = false;
let sceneReady = false;

const groups = {
  root: null,
  frameFull: null,
  frameCut: null,
  stator: null,
  rotorSpin: null,
  fan: null,
  fanCowl: null,
  frontCap: null,
  rearCap: null,
  terminal: null,
  terminalLid: null,
  terminalLinksDelta: null,
  terminalLinksWye: null,
  terminalInternals: null,
  base: null,
  sensors: null,
  sensorHighlights: null,
  field: null,
  fieldPoles: null,
  currentArrows: null,
  barrels: null,
  dav: null,
  daq: null,
  airflow: null,
  alarmBeacon: null,
  jam: null,
  cableRoot: null,
  nameplate: null,
  bearingCages: [],
  brokenBarMarkers: null,
  bearingMarkers: null,
  windingMats: [],
  barMesh: null
};

const partGroups = new Map();
const partPickables = [];
const partBasePositions = new Map();
const animationGroups = new Map();
const sensorPickables = new Map();
const highlightObjects = new Map();
const glowSprites = [];
const textureCache = new Map();
const materialLib = {};

const CAMERA_PRESETS = {
  hero: { pos: [3.45, 2.12, 4.27], target: [0, -0.03, 0.03], fov: 32 },
  section: { pos: [4.32, 2.30, 4.42], target: [0, -0.01, 0.10], fov: 32 },
  field: { pos: [2.25, 1.28, 3.92], target: [0, 0.08, 0.50], fov: 30 },
  rotor: { pos: [4.15, 1.95, 5.18], target: [0.04, 0.04, 1.03], fov: 31 },
  sensors: { pos: [0.30, 6.15, 4.30], target: [0, -0.16, 0.44], fov: 31 },
  exploded: { pos: [6.00, 3.25, 7.50], target: [0.10, 0.05, 0.40], fov: 31 }
};

function initThree() {
  stageElement = $('#stage');
  canvasElement = $('#gl');
  stageW = Math.max(1, stageElement.clientWidth);
  stageH = Math.max(1, stageElement.clientHeight);

  try {
    if (!window.WebGL2RenderingContext) throw new Error('WebGL2RenderingContext unavailable');
    renderer = new THREE.WebGLRenderer({
      canvas: canvasElement,
      antialias: true,
      alpha: true,
      powerPreference: 'high-performance',
      stencil: false
    });
  } catch (err) {
    showFallback('无法创建 WebGL2 渲染上下文。请使用最新版 Chrome / Edge / Safari，并确认硬件加速开启。');
    return false;
  }

  renderer.setClearColor(0x000000, 0);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  currentPixelRatio = renderer.getPixelRatio();
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.08;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  renderer.info.autoReset = true;

  scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x080b10, 17, 40);

  camera = new THREE.PerspectiveCamera(32, stageW / stageH, 0.1, 80);
  const initialCam = getResponsiveCamera('hero');
  camera.position.set(...initialCam.pos);
  camera.lookAt(new THREE.Vector3(...initialCam.target));

  orbitRig = new OrbitRig(camera, canvasElement, {
    target: new THREE.Vector3(...initialCam.target),
    onInteraction: () => {
      if (ui.cinemaActive) stopCinematic(false);
      if (ui.guideVisible && ui.guideStep === 0) advanceGuide();
      if (ui.selectedSensor && ui.selectedSensor.kind === 'auto') { /* no op */ }
      setActiveCameraChip(null);
    },
    onClick: (event) => handleCanvasClick(event)
  });

  buildStudioEnvironment();
  buildLights();
  buildGround();
  buildMaterials();
  buildMotor();
  buildDust();
  updateRendererQuality(ui.quality, true);
  prefillTelemetry();
  buildPartPickList();
  refreshAllUI(true);
  sceneReady = true;
  return true;
}

function showFallback(message) {
  const el = $('#fallback');
  if (el) {
    el.classList.add('show');
    const p = $('#fallbackText');
    if (p && message) p.textContent = message;
  }
  document.getElementById('boot-screen')?.classList.add('done');
}

function buildStudioEnvironment() {
  const envScene = new THREE.Scene();
  envScene.background = new THREE.Color(0x04070a);
  const panelMat = (hex, intensity) => new THREE.MeshBasicMaterial({
    color: new THREE.Color(hex).multiplyScalar(intensity),
    side: THREE.DoubleSide,
    toneMapped: false
  });
  const addPanel = (hex, intensity, w, h, pos, lookAt = [0, 0, 0]) => {
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(w, h), panelMat(hex, intensity));
    mesh.position.set(...pos);
    if (lookAt) mesh.lookAt(new THREE.Vector3(...lookAt));
    envScene.add(mesh);
    return mesh;
  };

  addPanel(0xfff2df, 7.5, 7.5, 5.8, [5.5, 6.2, 4.4]);
  addPanel(0xc8e6ff, 2.2, 6.2, 4.2, [-6.0, 2.4, 3.0]);
  addPanel(0x7bdcff, 5.0, 1.5, 7.0, [-4.8, 4.4, -6.5], [0, 1, 0]);
  addPanel(0xffd8ad, 2.6, 2.0, 4.0, [3.8, 1.6, -6.5], [0, 1, 0]);
  const overhead = addPanel(0xffffff, 4.0, 8.0, 7.0, [0, 8.5, 0], [0, 0, 0]);
  overhead.rotation.x = -Math.PI / 2;
  addPanel(0x9bc7df, 0.9, 12, 7, [0, -1.5, 7.5], [0, 0, 0]);
  // A narrow bottom bounce card keeps the lower end shield readable in black.
  addPanel(0x6d6156, 1.3, 6.5, 1.4, [1.0, -2.2, 3.2]);

  pmremGenerator = new THREE.PMREMGenerator(renderer);
  pmremGenerator.compileCubemapShader();
  environmentTexture = pmremGenerator.fromScene(envScene, 0.035).texture;
  scene.environment = environmentTexture;
  scene.environmentIntensity = 0.92;
  environmentGenerated = true;

  envScene.traverse((obj) => {
    if (obj.isMesh) {
      obj.geometry.dispose();
      obj.material.dispose();
    }
  });
  pmremGenerator.dispose();
}

function buildLights() {
  scene.add(new THREE.HemisphereLight(0x91b4c8, 0x080b0e, 0.18));
  fillLight = new THREE.DirectionalLight(0x86aec2, 0.38);
  fillLight.position.set(-3.5, -1.5, 5.5);
  scene.add(fillLight);

  keyLight = new THREE.SpotLight(0xffeeda, 310, 30, Math.PI * 0.27, 0.72, 1.55);
  keyLight.position.set(5.8, 7.2, 6.6);
  keyLight.target.position.set(0, -0.1, 0.1);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(2048, 2048);
  keyLight.shadow.bias = -0.00035;
  keyLight.shadow.normalBias = 0.018;
  keyLight.shadow.camera.near = 4;
  keyLight.shadow.camera.far = 22;
  keyLight.shadow.radius = 3;
  scene.add(keyLight);
  scene.add(keyLight.target);

  rimLight = new THREE.SpotLight(0x6dcfe7, 260, 28, Math.PI * 0.32, 0.8, 1.6);
  rimLight.position.set(-5.4, 4.8, -6.8);
  rimLight.target.position.set(0, 0.15, 0.3);
  scene.add(rimLight);
  scene.add(rimLight.target);

  const topKick = new THREE.SpotLight(0xffc791, 120, 20, Math.PI * 0.36, 0.85, 1.7);
  topKick.position.set(-1.5, 6.5, 1.0);
  topKick.target.position.set(0, 0, 0);
  scene.add(topKick);
  scene.add(topKick.target);

  alarmLight = new THREE.PointLight(0xff3329, 0, 4.5, 1.8);
  alarmLight.position.set(0.2, 1.4, 0.2);
  scene.add(alarmLight);
}

function buildGround() {
  const groundMat = new THREE.MeshStandardMaterial({
    color: 0x090d12,
    metalness: 0.48,
    roughness: 0.34,
    envMapIntensity: 0.5
  });
  const g = new THREE.Mesh(new THREE.PlaneGeometry(70, 70), groundMat);
  g.rotation.x = -Math.PI / 2;
  g.position.y = GROUND_Y - 0.005;
  g.receiveShadow = true;
  scene.add(g);
  ground = g;

  // Soft contact shadow under the feet / motor body.
  const contact = new THREE.Mesh(
    new THREE.PlaneGeometry(5.4, 4.0),
    new THREE.MeshBasicMaterial({
      map: makeContactShadowTexture(),
      transparent: true,
      depthWrite: false,
      opacity: 0.94
    })
  );
  contact.rotation.x = -Math.PI / 2;
  contact.position.set(0.25, GROUND_Y + 0.006, 0.15);
  scene.add(contact);

  // Sparse floor reference: a single fine axis cross, no decorative rings.
  const axisMat = new THREE.LineBasicMaterial({ color: 0x2e4351, transparent: true, opacity: 0.28 });
  const axisPts = [
    new THREE.Vector3(-4.8, GROUND_Y + 0.009, 0.2), new THREE.Vector3(4.8, GROUND_Y + 0.009, 0.2),
    new THREE.Vector3(0.2, GROUND_Y + 0.009, -4.8), new THREE.Vector3(0.2, GROUND_Y + 0.009, 4.8)
  ];
  const axisGeo = new THREE.BufferGeometry().setFromPoints(axisPts);
  const axis = new THREE.LineSegments(axisGeo, axisMat);
  scene.add(axis);
}

function buildDust() {
  const count = 130;
  const positions = new Float32Array(count * 3);
  const rand = pseudoRandom(92);
  for (let i = 0; i < count; i++) {
    positions[i * 3] = (rand() - 0.5) * 12;
    positions[i * 3 + 1] = rand() * 3.2 - 0.7;
    positions[i * 3 + 2] = (rand() - 0.5) * 10;
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  const mat = new THREE.PointsMaterial({
    color: 0x8fbccb, size: 0.012, transparent: true, opacity: 0.23,
    depthWrite: false, sizeAttenuation: true, blending: THREE.AdditiveBlending
  });
  dust = new THREE.Points(geo, mat);
  dust.frustumCulled = false;
  scene.add(dust);
}

// ===== 11. 程序纹理 =====
function makeCanvasTexture(w, h, draw, colorSpace = THREE.SRGBColorSpace) {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  const ctx = c.getContext('2d');
  draw(ctx, w, h);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = colorSpace;
  tex.anisotropy = Math.min(8, renderer?.capabilities.getMaxAnisotropy?.() || 4);
  tex.needsUpdate = true;
  return tex;
}

function makeNoiseTexture(size = 256, strength = 10) {
  return makeCanvasTexture(size, size, (ctx, w, h) => {
    const img = ctx.createImageData(w, h);
    const rand = pseudoRandom(771);
    for (let i = 0; i < img.data.length; i += 4) {
      const n = 128 + (rand() - 0.5) * strength;
      img.data[i] = n; img.data[i + 1] = n; img.data[i + 2] = n; img.data[i + 3] = 255;
    }
    ctx.putImageData(img, 0, 0);
    // A few very faint cast-metal pores, not a stone texture.
    ctx.globalAlpha = 0.22;
    for (let i = 0; i < 40; i++) {
      ctx.fillStyle = '#4d4d4d';
      const x = rand() * w, y = rand() * h, r = 0.4 + rand() * 1.4;
      ctx.beginPath(); ctx.arc(x, y, r, 0, TAU); ctx.fill();
    }
  }, THREE.NoColorSpace);
}

function makeBrushedTexture() {
  return makeCanvasTexture(512, 512, (ctx, w, h) => {
    ctx.fillStyle = '#999';
    ctx.fillRect(0, 0, w, h);
    const rand = pseudoRandom(1244);
    for (let i = 0; i < 2100; i++) {
      const y = rand() * h;
      const alpha = 0.015 + rand() * 0.05;
      ctx.strokeStyle = rand() > 0.5 ? `rgba(255,255,255,${alpha})` : `rgba(0,0,0,${alpha})`;
      ctx.lineWidth = 0.4 + rand() * 0.7;
      ctx.beginPath();
      const x0 = rand() * w;
      ctx.moveTo(x0, y);
      ctx.lineTo(x0 + 40 + rand() * 190, y + (rand() - 0.5) * 0.6);
      ctx.stroke();
    }
  }, THREE.NoColorSpace);
}

function makeContactShadowTexture() {
  return makeCanvasTexture(512, 512, (ctx, w, h) => {
    const grd = ctx.createRadialGradient(w * 0.5, h * 0.52, 10, w * 0.5, h * 0.52, w * 0.5);
    grd.addColorStop(0, 'rgba(0,0,0,0.72)');
    grd.addColorStop(0.4, 'rgba(0,0,0,0.43)');
    grd.addColorStop(0.72, 'rgba(0,0,0,0.13)');
    grd.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = grd;
    ctx.fillRect(0, 0, w, h);
  });
}

function drawNameplateArt(ctx, w, h) {
  ctx.fillStyle = '#05080b';
  ctx.fillRect(0, 0, w, h);
  const grd = ctx.createLinearGradient(0, 0, w, h);
  grd.addColorStop(0, '#d7d7d1');
  grd.addColorStop(0.2, '#b8b9b3');
  grd.addColorStop(0.45, '#e3e2da');
  grd.addColorStop(0.68, '#a8aaa5');
  grd.addColorStop(1, '#ced0ca');
  const pad = Math.round(w * 0.026);
  ctx.fillStyle = grd;
  ctx.fillRect(pad, pad, w - pad * 2, h - pad * 2);
  // fine brushed metal
  const rand = pseudoRandom(89);
  for (let i = 0; i < 800; i++) {
    const y = pad + rand() * (h - pad * 2);
    ctx.strokeStyle = rand() > 0.5 ? 'rgba(255,255,255,.10)' : 'rgba(40,42,42,.07)';
    ctx.lineWidth = 0.5 + rand();
    ctx.beginPath();
    ctx.moveTo(pad, y);
    ctx.lineTo(w - pad, y + (rand() - 0.5) * 1.6);
    ctx.stroke();
  }
  // screws
  const screw = (x, y) => {
    ctx.fillStyle = 'rgba(40,43,44,.32)';
    ctx.beginPath(); ctx.arc(x, y, w * 0.011, 0, TAU); ctx.fill();
    ctx.strokeStyle = 'rgba(20,22,23,.55)'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x - 5, y); ctx.lineTo(x + 5, y); ctx.stroke();
  };
  screw(pad + w * 0.03, pad + h * 0.055);
  screw(w - pad - w * 0.03, pad + h * 0.055);
  screw(pad + w * 0.03, h - pad - h * 0.055);
  screw(w - pad - w * 0.03, h - pad - h * 0.055);
  // dark etched text field
  const x0 = w * 0.075, y0 = h * 0.16, x1 = w * 0.925, y1 = h * 0.86;
  ctx.strokeStyle = 'rgba(33,36,36,.78)';
  ctx.lineWidth = Math.max(1, w * 0.002);
  ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
  ctx.strokeRect(x0 + 4, y0 + 4, x1 - x0 - 8, y1 - y0 - 8);
  ctx.fillStyle = '#1f2424';
  ctx.textBaseline = 'middle';
  const titleFont = `600 ${Math.round(h * 0.063)}px "Microsoft YaHei","PingFang SC",sans-serif`;
  ctx.font = titleFont;
  ctx.fillText('三相异步电动机', x0 + w * 0.035, y0 + h * 0.095);
  ctx.font = `500 ${Math.round(h * 0.036)}px "Cascadia Code","Consolas",monospace`;
  ctx.fillStyle = '#3b3f3e';
  ctx.fillText('THREE-PHASE INDUCTION MOTOR', x0 + w * 0.036, y0 + h * 0.155);
  ctx.textAlign = 'right';
  ctx.fillStyle = '#1f2424';
  ctx.font = `600 ${Math.round(h * 0.084)}px "Bahnschrift","Segoe UI",sans-serif`;
  ctx.fillText('Y90S-4', x1 - w * 0.033, y0 + h * 0.092);
  ctx.font = `500 ${Math.round(h * 0.026)}px "Cascadia Code",monospace`;
  ctx.fillText('No. 01 / TEACHING SAMPLE', x1 - w * 0.034, y0 + h * 0.156);
  ctx.textAlign = 'left';

  const lines = [
    ['额定功率', '1.1 kW', '额定电压', '380 V   △'],
    ['额定电流', '2.2 A', '额定转速', '1400 r/min'],
    ['频率 / 极数', '50 Hz / 4P', '接法', '△'],
    ['功率因数', 'cos φ 0.78', '效率', 'η 79 %'],
    ['防护等级', 'IP44', '工作制', 'S1'],
  ];
  const col1 = x0 + w * 0.036;
  const col2 = x0 + w * 0.44;
  const col3 = x0 + w * 0.646;
  const rowH = h * 0.105;
  ctx.font = `500 ${Math.round(h * 0.032)}px "Microsoft YaHei","PingFang SC",sans-serif`;
  lines.forEach((row, i) => {
    const y = y0 + h * 0.235 + i * rowH;
    ctx.fillStyle = '#3d4240';
    ctx.fillText(row[0], col1, y);
    ctx.fillStyle = '#151a1a';
    ctx.font = `600 ${Math.round(h * 0.034)}px "Cascadia Code","Consolas",monospace`;
    ctx.fillText(row[1], col2, y);
    ctx.font = `500 ${Math.round(h * 0.032)}px "Microsoft YaHei","PingFang SC",sans-serif`;
    ctx.fillStyle = '#3d4240';
    ctx.fillText(row[2], col3, y);
    ctx.fillStyle = '#151a1a';
    ctx.font = `600 ${Math.round(h * 0.032)}px "Cascadia Code","Consolas",monospace`;
    ctx.fillText(row[3], col3 + w * 0.14, y);
    ctx.font = `500 ${Math.round(h * 0.032)}px "Microsoft YaHei","PingFang SC",sans-serif`;
  });
  ctx.strokeStyle = 'rgba(45,48,46,.36)';
  ctx.lineWidth = 1;
  for (let i = 1; i < 5; i++) {
    const y = y0 + h * 0.195 + i * rowH;
    ctx.beginPath(); ctx.moveTo(col1, y); ctx.lineTo(x1 - w * 0.035, y); ctx.stroke();
  }
  ctx.fillStyle = 'rgba(34,37,36,.78)';
  ctx.font = `500 ${Math.round(h * 0.025)}px "Cascadia Code",monospace`;
  ctx.fillText('MOTOR BRIEF / NO.01   ·   教学虚构铭牌   ·   数据口径与 3D 模型一致', x0 + w * 0.036, y1 - h * 0.045);
}

function makeNameplateTexture() {
  return makeCanvasTexture(1024, 560, (ctx, w, h) => drawNameplateArt(ctx, w, h));
}

function makeGlowTexture(hex = '#70d8e5') {
  const key = 'glow-' + hex;
  if (textureCache.has(key)) return textureCache.get(key);
  const tex = makeCanvasTexture(128, 128, (ctx, w, h) => {
    const grd = ctx.createRadialGradient(w / 2, h / 2, 0, w / 2, h / 2, w / 2);
    const color = new THREE.Color(hex);
    const c = `rgba(${Math.round(color.r * 255)},${Math.round(color.g * 255)},${Math.round(color.b * 255)},`;
    grd.addColorStop(0, c + '0.95)');
    grd.addColorStop(0.16, c + '0.48)');
    grd.addColorStop(0.46, c + '0.12)');
    grd.addColorStop(1, c + '0)');
    ctx.fillStyle = grd;
    ctx.fillRect(0, 0, w, h);
  });
  textureCache.set(key, tex);
  return tex;
}

function makeLabelTexture(text, opts = {}) {
  const { color = '#dfeef2', bg = 'rgba(7,12,17,.78)', border = 'rgba(112,216,229,.36)', size = 54 } = opts;
  const pad = 18;
  const font = `600 ${size}px "Cascadia Code","Consolas",monospace`;
  const measure = document.createElement('canvas').getContext('2d');
  measure.font = font;
  const w = Math.ceil(measure.measureText(text).width + pad * 2);
  const h = size + pad * 2;
  const tex = makeCanvasTexture(w, h, (ctx, cw, ch) => {
    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, cw, ch);
    ctx.strokeStyle = border;
    ctx.lineWidth = 2;
    ctx.strokeRect(1, 1, cw - 2, ch - 2);
    ctx.fillStyle = color;
    ctx.font = font;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, cw / 2, ch / 2 + 2);
  });
  tex.userData = { aspect: w / h };
  return tex;
}

function addLeafTextureToMaterial(material, key, maker, repeatX = 1, repeatY = 1) {
  if (!textureCache.has(key)) textureCache.set(key, maker());
  const tex = textureCache.get(key).clone();
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(repeatX, repeatY);
  tex.needsUpdate = true;
  material.roughnessMap = tex;
  material.needsUpdate = true;
}

// ===== 12. 材质系统 =====
function buildMaterials() {
  const noise = makeNoiseTexture(256, 8);
  noise.wrapS = noise.wrapT = THREE.RepeatWrapping;
  noise.repeat.set(3, 3);
  const noise2 = noise.clone();
  noise2.needsUpdate = true;
  noise2.repeat.set(2, 5);
  const brushed = makeBrushedTexture();
  brushed.wrapS = brushed.wrapT = THREE.RepeatWrapping;
  brushed.repeat.set(3, 2);

  materialLib.castIron = new THREE.MeshStandardMaterial({
    color: 0x484c51, metalness: 0.58, roughness: 0.64,
    roughnessMap: noise, envMapIntensity: 0.82
  });
  materialLib.castIronLight = new THREE.MeshStandardMaterial({
    color: 0x5b6065, metalness: 0.56, roughness: 0.58,
    roughnessMap: noise, envMapIntensity: 0.86
  });
  materialLib.castIronDark = new THREE.MeshStandardMaterial({
    color: 0x33373c, metalness: 0.58, roughness: 0.70,
    roughnessMap: noise2, envMapIntensity: 0.70
  });
  materialLib.machinedEdge = new THREE.MeshStandardMaterial({
    color: 0xa6adb4, metalness: 0.93, roughness: 0.24,
    roughnessMap: brushed, envMapIntensity: 1.15
  });
  materialLib.steel = new THREE.MeshStandardMaterial({
    color: 0x999fa4, metalness: 0.95, roughness: 0.22,
    roughnessMap: brushed, envMapIntensity: 1.2
  });
  materialLib.steelDark = new THREE.MeshStandardMaterial({
    color: 0x5d646b, metalness: 0.92, roughness: 0.3,
    roughnessMap: brushed, envMapIntensity: 1.04
  });
  materialLib.aluminum = new THREE.MeshStandardMaterial({
    color: 0x9ba1a3, metalness: 0.8, roughness: 0.42,
    roughnessMap: noise, envMapIntensity: 1.0
  });
  materialLib.siliconSteel = new THREE.MeshStandardMaterial({
    color: 0x373e45, metalness: 0.7, roughness: 0.46,
    roughnessMap: noise2, envMapIntensity: 0.8
  });
  materialLib.siliconEdge = new THREE.MeshStandardMaterial({
    color: 0x6f777e, metalness: 0.85, roughness: 0.32,
    envMapIntensity: 0.95
  });
  materialLib.copper = new THREE.MeshStandardMaterial({
    color: 0xb97c50, metalness: 0.86, roughness: 0.3, envMapIntensity: 1.02
  });
  materialLib.copperU = new THREE.MeshStandardMaterial({
    color: 0xc58b5d, metalness: 0.82, roughness: 0.32, envMapIntensity: 1.03,
    emissive: 0x000000
  });
  materialLib.copperV = new THREE.MeshStandardMaterial({
    color: 0xb7a170, metalness: 0.78, roughness: 0.34, envMapIntensity: 0.98,
    emissive: 0x000000
  });
  materialLib.copperW = new THREE.MeshStandardMaterial({
    color: 0xcbb083, metalness: 0.8, roughness: 0.34, envMapIntensity: 0.96,
    emissive: 0x000000
  });
  materialLib.copperBar = new THREE.MeshStandardMaterial({
    color: 0xb47042, metalness: 0.91, roughness: 0.26, envMapIntensity: 1.12
  });
  materialLib.copperDark = new THREE.MeshStandardMaterial({
    color: 0x6f4b35, metalness: 0.8, roughness: 0.42, envMapIntensity: 0.8
  });
  materialLib.brass = new THREE.MeshStandardMaterial({
    color: 0xb59b65, metalness: 0.9, roughness: 0.29, envMapIntensity: 1.0
  });
  materialLib.blackPolymer = new THREE.MeshStandardMaterial({
    color: 0x14181d, metalness: 0.18, roughness: 0.62, envMapIntensity: 0.62
  });
  materialLib.blackMetal = new THREE.MeshStandardMaterial({
    color: 0x242a30, metalness: 0.8, roughness: 0.42, envMapIntensity: 0.9
  });
  materialLib.rubber = new THREE.MeshStandardMaterial({
    color: 0x0d1013, metalness: 0.05, roughness: 0.82, envMapIntensity: 0.35
  });
  materialLib.insulator = new THREE.MeshStandardMaterial({
    color: 0x1e2428, metalness: 0.08, roughness: 0.76, envMapIntensity: 0.4
  });
  materialLib.fan = new THREE.MeshStandardMaterial({
    color: 0x7c838a, metalness: 0.7, roughness: 0.5,
    roughnessMap: noise, envMapIntensity: 0.9
  });
  materialLib.fanDark = new THREE.MeshStandardMaterial({
    color: 0x3b4248, metalness: 0.74, roughness: 0.52, envMapIntensity: 0.8
  });
  materialLib.bearingSteel = new THREE.MeshStandardMaterial({
    color: 0x7f8a93, metalness: 0.96, roughness: 0.2, envMapIntensity: 1.2
  });
  materialLib.bearingBrass = new THREE.MeshStandardMaterial({
    color: 0x9b8a65, metalness: 0.86, roughness: 0.38, envMapIntensity: 0.9
  });
  materialLib.terminalRed = new THREE.MeshStandardMaterial({ color: 0x5c2622, metalness: 0.45, roughness: 0.45 });
  materialLib.terminalBlue = new THREE.MeshStandardMaterial({ color: 0x24404e, metalness: 0.45, roughness: 0.45 });
  materialLib.ledGreen = new THREE.MeshBasicMaterial({ color: 0x83d3a4, toneMapped: false });
  materialLib.ledRed = new THREE.MeshBasicMaterial({ color: 0xff5148, toneMapped: false });
  materialLib.ledAmber = new THREE.MeshBasicMaterial({ color: 0xffab5e, toneMapped: false });
  materialLib.sensorBody = new THREE.MeshStandardMaterial({
    color: 0x20272e, metalness: 0.4, roughness: 0.48, envMapIntensity: 0.8
  });
  materialLib.sensorLabel = new THREE.MeshBasicMaterial({ color: 0x9ecbd6, toneMapped: false });
  materialLib.glassBlack = new THREE.MeshPhysicalMaterial({
    color: 0x0c1115, metalness: 0.1, roughness: 0.16, clearcoat: 1, clearcoatRoughness: 0.1
  });
  materialLib.paintRed = new THREE.MeshStandardMaterial({
    color: 0x8a332f, metalness: 0.32, roughness: 0.5, envMapIntensity: 0.8
  });
  materialLib.paintAmber = new THREE.MeshStandardMaterial({
    color: 0xb3773f, metalness: 0.3, roughness: 0.48, envMapIntensity: 0.8
  });
  materialLib.paintGreen = new THREE.MeshStandardMaterial({
    color: 0x436b53, metalness: 0.3, roughness: 0.46, envMapIntensity: 0.8
  });
  materialLib.emissiveCyan = new THREE.MeshBasicMaterial({
    color: 0x70d8e5, transparent: true, opacity: 0.65,
    blending: THREE.AdditiveBlending, depthWrite: false, toneMapped: false
  });
  materialLib.emissiveAmber = new THREE.MeshBasicMaterial({
    color: 0xffab5e, transparent: true, opacity: 0.82,
    blending: THREE.AdditiveBlending, depthWrite: false, toneMapped: false
  });
  materialLib.emissiveRed = new THREE.MeshBasicMaterial({
    color: 0xff5b4f, transparent: true, opacity: 0.8,
    blending: THREE.AdditiveBlending, depthWrite: false, toneMapped: false
  });
  materialLib.emissiveCopper = new THREE.MeshStandardMaterial({
    color: 0x8b4a32, emissive: 0x1c0803, emissiveIntensity: 0, metalness: 0.6, roughness: 0.38
  });
  materialLib.nameplate = new THREE.MeshStandardMaterial({
    map: makeNameplateTexture(), metalness: 0.52, roughness: 0.33,
    envMapIntensity: 0.8
  });
  materialLib.nameplate.map.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
  materialLib.highlightCyan = new THREE.MeshBasicMaterial({
    color: 0x70d8e5, transparent: true, opacity: 0.14,
    blending: THREE.AdditiveBlending, depthWrite: false, side: THREE.DoubleSide, toneMapped: false
  });
  materialLib.highlightRed = new THREE.MeshBasicMaterial({
    color: 0xff4f45, transparent: true, opacity: 0.16,
    blending: THREE.AdditiveBlending, depthWrite: false, side: THREE.DoubleSide, toneMapped: false
  });
  materialLib.selectedLine = new THREE.LineBasicMaterial({
    color: 0x87effc, transparent: true, opacity: 0.7, depthWrite: false, toneMapped: false
  });
}

// ===== 13. 几何工具 =====
function mesh(geo, mat, partId = null, parent = null) {
  const m = new THREE.Mesh(geo, mat);
  m.castShadow = true;
  m.receiveShadow = true;
  if (partId) {
    m.userData.partId = partId;
    partPickables.push(m);
  }
  if (parent) parent.add(m);
  return m;
}

function cylZ(radiusTop, radiusBottom, height, radialSegments = 48, openEnded = false) {
  const geo = new THREE.CylinderGeometry(radiusTop, radiusBottom, height, radialSegments, 1, openEnded);
  geo.rotateX(Math.PI / 2);
  return geo;
}

function latheZ(points, segments = 64, phiStart = 0, phiLength = TAU) {
  const geo = new THREE.LatheGeometry(points, segments, phiStart, phiLength);
  geo.rotateX(Math.PI / 2);
  return geo;
}

function roundedBoxGeo(w, h, d, r = 0.03) {
  const rr = Math.min(r, Math.min(w, d) / 2 - 0.001);
  const shape = new THREE.Shape();
  const x = -w / 2, y = -d / 2;
  shape.moveTo(x + rr, y);
  shape.lineTo(x + w - rr, y);
  shape.quadraticCurveTo(x + w, y, x + w, y + rr);
  shape.lineTo(x + w, y + d - rr);
  shape.quadraticCurveTo(x + w, y + d, x + w - rr, y + d);
  shape.lineTo(x + rr, y + d);
  shape.quadraticCurveTo(x, y + d, x, y + d - rr);
  shape.lineTo(x, y + rr);
  shape.quadraticCurveTo(x, y, x + rr, y);
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: h, bevelEnabled: true, bevelSegments: 2, bevelSize: Math.min(0.012, h * 0.18),
    bevelThickness: Math.min(0.012, h * 0.18), steps: 1, curveSegments: 5
  });
  geo.rotateX(-Math.PI / 2);
  geo.center();
  return geo;
}

function annularSectorGeo(outerR, innerR, start, length, depth, curveSegments = 96) {
  const shape = new THREE.Shape();
  const steps = Math.max(12, Math.ceil(curveSegments * length / TAU));
  const end = start + length;
  shape.moveTo(outerR * Math.cos(start), outerR * Math.sin(start));
  for (let i = 1; i <= steps; i++) {
    const a = start + length * i / steps;
    shape.lineTo(outerR * Math.cos(a), outerR * Math.sin(a));
  }
  shape.lineTo(innerR * Math.cos(end), innerR * Math.sin(end));
  for (let i = 1; i <= steps; i++) {
    const a = end - length * i / steps;
    shape.lineTo(innerR * Math.cos(a), innerR * Math.sin(a));
  }
  shape.closePath();
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth, bevelEnabled: true, bevelSegments: 2, bevelSize: 0.006,
    bevelThickness: 0.006, steps: 1, curveSegments: 4
  });
  geo.translate(0, 0, -depth / 2);
  return geo;
}

function addHexBolt(parent, x, y, z, size = 0.04, rotZ = 0, mat = null) {
  const boltMat = mat || materialLib.steelDark;
  const shank = new THREE.Mesh(
    new THREE.CylinderGeometry(size, size * 0.96, size * 1.15, 6, 1),
    boltMat
  );
  shank.rotation.x = Math.PI / 2;
  shank.position.set(x, y, z);
  shank.rotation.z = rotZ;
  shank.castShadow = true;
  parent.add(shank);
  const washer = new THREE.Mesh(new THREE.TorusGeometry(size * 0.9, size * 0.12, 6, 16), boltMat);
  washer.position.set(x, y, z - size * 0.7);
  parent.add(washer);
  const cross = new THREE.Mesh(new THREE.BoxGeometry(size * 0.82, size * 0.08, size * 0.06), materialLib.castIronDark);
  cross.position.set(x, y, z + size * 0.58);
  parent.add(cross);
  return shank;
}

function addRingOfBolts(parent, count, radius, z, size = 0.035, phase = 0) {
  for (let i = 0; i < count; i++) {
    const a = phase + i / count * TAU;
    addHexBolt(parent, radius * Math.cos(a), radius * Math.sin(a), z, size, a);
  }
}

function makeGlowSprite(hex, size = 0.5, opacity = 0.65) {
  const mat = new THREE.SpriteMaterial({
    map: makeGlowTexture(hex),
    color: 0xffffff,
    transparent: true,
    opacity,
    blending: THREE.AdditiveBlending,
    depthWrite: false,
    depthTest: true,
    toneMapped: false
  });
  const sprite = new THREE.Sprite(mat);
  sprite.scale.set(size, size, 1);
  sprite.userData.baseOpacity = opacity;
  glowSprites.push(sprite);
  return sprite;
}

function makeSensorTagSprite(text, color = '#70d8e5') {
  const tex = makeLabelTexture(text, { color, size: 54, bg: 'rgba(5,10,15,.76)', border: color });
  const mat = new THREE.SpriteMaterial({
    map: tex, transparent: true, depthWrite: false, toneMapped: false, opacity: 0.82
  });
  const sprite = new THREE.Sprite(mat);
  const aspect = tex.userData?.aspect || 3.5;
  sprite.scale.set(0.12 * aspect, 0.12, 1);
  sprite.renderOrder = 8;
  return sprite;
}

// ===== 14. 相机轨道控制（自包含，无 addon 依赖） =====
class OrbitRig {
  constructor(camera, dom, opts = {}) {
    this.camera = camera;
    this.dom = dom;
    this.target = opts.target?.clone() || new THREE.Vector3();
    this.goalTarget = this.target.clone();
    this.onInteraction = opts.onInteraction || (() => {});
    this.onClick = opts.onClick || (() => {});
    const offset = camera.position.clone().sub(this.target);
    const spherical = new THREE.Spherical().setFromVector3(offset);
    this.radius = spherical.radius;
    this.phi = spherical.phi;
    this.theta = spherical.theta;
    this.goalRadius = this.radius;
    this.goalPhi = this.phi;
    this.goalTheta = this.theta;
    this.transition = null;
    this.dragging = false;
    this.pointers = new Map();
    this.lastX = 0; this.lastY = 0;
    this.downX = 0; this.downY = 0; this.downTime = 0;
    this.moved = 0;
    this.pinchDist = 0;
    this.enabled = true;
    this.dom.style.cursor = 'grab';
    this.bind();
  }

  bind() {
    const dom = this.dom;
    dom.addEventListener('pointerdown', (e) => {
      if (!this.enabled) return;
      if (e.pointerType === 'mouse' && e.button !== 0) return;
      dom.setPointerCapture?.(e.pointerId);
      this.pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (this.pointers.size === 1) {
        this.dragging = true;
        this.moved = 0;
        this.downX = e.clientX;
        this.downY = e.clientY;
        this.lastX = e.clientX;
        this.lastY = e.clientY;
        this.downTime = performance.now();
        this.cancelTransition();
        dom.style.cursor = 'grabbing';
      } else if (this.pointers.size === 2) {
        const pts = [...this.pointers.values()];
        this.pinchDist = Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
      }
      this.onInteraction('pointer');
    }, { passive: true });

    dom.addEventListener('pointermove', (e) => {
      if (!this.pointers.has(e.pointerId)) return;
      this.pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (this.pointers.size >= 2) {
        const pts = [...this.pointers.values()];
        const dist = Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
        if (this.pinchDist > 0) {
          const factor = this.pinchDist / Math.max(1, dist);
          this.goalRadius = clamp(this.goalRadius * (0.88 + factor * 0.12), 3.1, 19);
          this.radius = this.goalRadius;
        }
        this.pinchDist = dist;
        this.moved = 99;
        return;
      }
      if (!this.dragging) return;
      const dx = e.clientX - this.lastX;
      const dy = e.clientY - this.lastY;
      this.lastX = e.clientX;
      this.lastY = e.clientY;
      this.moved += Math.abs(dx) + Math.abs(dy);
      if (this.moved < 2) return;
      this.goalTheta -= dx * 0.0055;
      this.goalPhi -= dy * 0.0049;
      this.goalPhi = clamp(this.goalPhi, 0.19, Math.PI * 0.485);
      this.theta = this.goalTheta;
      this.phi = this.goalPhi;
    }, { passive: true });

    const release = (e) => {
      if (!this.pointers.has(e.pointerId) && e.pointerType) return;
      this.pointers.delete(e.pointerId);
      if (this.pointers.size === 0) {
        const wasDragging = this.dragging;
        this.dragging = false;
        this.dom.style.cursor = 'grab';
        this.onInteraction('release');
        if (wasDragging && this.moved < 5 && performance.now() - this.downTime < 700) {
          this.onClick(e);
        }
      }
      this.pinchDist = 0;
    };
    dom.addEventListener('pointerup', release, { passive: true });
    dom.addEventListener('pointercancel', release, { passive: true });
    dom.addEventListener('pointerleave', (e) => {
      if (this.pointers.has(e.pointerId) && e.pointerType !== 'mouse') release(e);
    }, { passive: true });

    dom.addEventListener('wheel', (e) => {
      if (!this.enabled) return;
      e.preventDefault();
      this.cancelTransition();
      this.goalRadius = clamp(this.goalRadius * Math.exp(e.deltaY * 0.0011), 3.1, 19);
      this.onInteraction('wheel');
    }, { passive: false });
  }

  cancelTransition() {
    if (!this.transition) return;
    this.transition = null;
    const offset = this.camera.position.clone().sub(this.target);
    const sph = new THREE.Spherical().setFromVector3(offset);
    this.radius = clamp(sph.radius, 3.1, 19);
    this.phi = sph.phi;
    this.theta = sph.theta;
    this.goalRadius = this.radius;
    this.goalPhi = this.phi;
    this.goalTheta = this.theta;
    this.goalTarget.copy(this.target);
  }

  transitionTo(pos, target, duration = 1.15) {
    const toPos = new THREE.Vector3(...pos);
    const toTarget = target instanceof THREE.Vector3 ? target.clone() : new THREE.Vector3(...target);
    this.transition = {
      fromPos: this.camera.position.clone(),
      toPos,
      fromTarget: this.target.clone(),
      toTarget,
      elapsed: 0,
      duration: Math.max(0.05, duration)
    };
    this.goalTarget.copy(toTarget);
  }

  setImmediate(pos, target) {
    this.camera.position.set(...pos);
    this.target.copy(target instanceof THREE.Vector3 ? target : new THREE.Vector3(...target));
    this.goalTarget.copy(this.target);
    const sph = new THREE.Spherical().setFromVector3(this.camera.position.clone().sub(this.target));
    this.radius = sph.radius; this.phi = sph.phi; this.theta = sph.theta;
    this.goalRadius = this.radius; this.goalPhi = this.phi; this.goalTheta = this.theta;
    this.transition = null;
    this.camera.lookAt(this.target);
  }

  update(dt) {
    if (!this.transition && !this.dragging) {
      this.theta = damp(this.theta, this.goalTheta, 13, dt);
      this.phi = damp(this.phi, this.goalPhi, 13, dt);
      this.radius = damp(this.radius, this.goalRadius, 11, dt);
      this.target.lerp(this.goalTarget, 1 - Math.exp(-9 * dt));
    }
    if (this.transition) {
      const tr = this.transition;
      tr.elapsed += dt;
      const t = clamp(tr.elapsed / tr.duration, 0, 1);
      const e = easeInOut(t);
      this.camera.position.lerpVectors(tr.fromPos, tr.toPos, e);
      this.target.lerpVectors(tr.fromTarget, tr.toTarget, e);
      this.goalTarget.copy(tr.toTarget);
      if (t >= 1) {
        this.camera.position.copy(tr.toPos);
        this.target.copy(tr.toTarget);
        const sph = new THREE.Spherical().setFromVector3(this.camera.position.clone().sub(this.target));
        this.radius = sph.radius; this.phi = sph.phi; this.theta = sph.theta;
        this.goalRadius = this.radius; this.goalPhi = this.phi; this.goalTheta = this.theta;
        this.transition = null;
      }
    } else {
      const sinPhi = Math.sin(this.phi);
      const pos = new THREE.Vector3(
        this.target.x + this.radius * sinPhi * Math.sin(this.theta),
        this.target.y + this.radius * Math.cos(this.phi),
        this.target.z + this.radius * sinPhi * Math.cos(this.theta)
      );
      this.camera.position.copy(pos);
    }
    this.camera.lookAt(this.target);
  }
}

// ===== 15. 镜头导演 =====
function getResponsiveCamera(name) {
  const preset = CAMERA_PRESETS[name] || CAMERA_PRESETS.hero;
  const aspect = stageW / Math.max(1, stageH);
  let fit = 1;
  if (aspect < 1.72) fit *= Math.pow(1.72 / aspect, 0.62);
  if (aspect > 2.35) fit *= 0.95;
  const target = new THREE.Vector3(...preset.target);
  const pos = new THREE.Vector3(...preset.pos).sub(target).multiplyScalar(fit).add(target);
  return { pos: pos.toArray(), target: target.toArray(), fov: preset.fov || 32 };
}

function setCameraPreset(name, immediate = false, duration = 1.18) {
  if (!orbitRig) return;
  if (name === 'hero' && ui.structure !== 'assembly') {
    setStructure('assembly');
  }
  ui.cameraPreset = name;
  updateCameraChips();
  const p = getResponsiveCamera(name === 'rotor' && ui.structure === 'assembly' ? 'rotor' : name);
  if (immediate) orbitRig.setImmediate(p.pos, p.target);
  else orbitRig.transitionTo(p.pos, p.target, duration);
  if (camera.fov !== p.fov) {
    camera.fov = p.fov;
    camera.updateProjectionMatrix();
  }
  if (ui.guideVisible && ui.guideStep <= 1) advanceGuideTo(2);
}

function updateCameraChips() {
  $$('.cam-chip').forEach((b) => b.classList.toggle('active', b.dataset.camera === ui.cameraPreset));
  const select = $('#cameraSelect');
  if (select && ui.cameraPreset && ['hero', 'section', 'field', 'rotor', 'sensors'].includes(ui.cameraPreset)) {
    select.value = ui.cameraPreset;
  }
}

function setActiveCameraChip(name) {
  if (name) {
    ui.cameraPreset = name;
    updateCameraChips();
  } else {
    ui.cameraPreset = null;
    $$('.cam-chip').forEach((b) => b.classList.remove('active'));
  }
}

function focusCamera(pos, target, immediate = false, duration = 1.08) {
  if (!orbitRig) return;
  const aspect = stageW / Math.max(1, stageH);
  const fit = aspect < 1.72 ? Math.pow(1.72 / aspect, 0.58) : 1;
  const t = new THREE.Vector3(...target);
  const p = new THREE.Vector3(...pos).sub(t).multiplyScalar(fit).add(t);
  if (immediate) orbitRig.setImmediate(p.toArray(), t.toArray());
  else orbitRig.transitionTo(p.toArray(), t.toArray(), duration);
}

function resizeRenderer() {
  if (!renderer || !camera) return;
  stageW = Math.max(1, stageElement.clientWidth);
  stageH = Math.max(1, stageElement.clientHeight);
  renderer.setSize(stageW, stageH, false);
  camera.aspect = stageW / stageH;
  camera.updateProjectionMatrix();
  if (renderer.getPixelRatio() !== currentPixelRatio) {
    renderer.setPixelRatio(currentPixelRatio);
    renderer.setSize(stageW, stageH, false);
  }
  drawAllMiniCharts(true);
  if (ui.modalCardId) drawModal();
  if (ui.atlasOpen) drawAtlasCards();
}

// ===== 16. 画质档位 =====
function updateRendererQuality(mode, initial = false) {
  if (!renderer) return;
  ui.quality = mode;
  const dpr = window.devicePixelRatio || 1;
  let ratio = Math.min(dpr, 2);
  let shadowSize = 2048;
  let shadow = true;
  let dustVisible = true;
  if (mode === 'smooth') {
    ratio = Math.min(dpr, 1.1);
    shadow = false;
    dustVisible = false;
  } else if (mode === 'auto') {
    ratio = Math.min(dpr, 1.6);
    shadowSize = 1024;
  }
  currentPixelRatio = ratio;
  renderer.setPixelRatio(ratio);
  renderer.setSize(stageW, stageH, false);
  renderer.shadowMap.enabled = shadow;
  renderer.shadowMap.needsUpdate = true;
  if (keyLight) {
    keyLight.castShadow = shadow;
    keyLight.shadow.mapSize.set(shadowSize, shadowSize);
    if (keyLight.shadow.map) {
      keyLight.shadow.map.dispose();
      keyLight.shadow.map = null;
    }
  }
  if (dust) dust.visible = dustVisible;
  scene.traverse((obj) => {
    if (obj.isMesh && obj.material) {
      const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
      for (const m of mats) {
        if ('envMapIntensity' in m) m.envMapIntensity = mode === 'smooth' ? Math.min(m.envMapIntensity, 0.75) : m.envMapIntensity;
      }
    }
  });
  if (!initial) drawAllMiniCharts(true);
}

// ============================================================================
// PART III — procedural motor geometry
// ===== 17. 装配骨架 / 部件注册 =====
// ============================================================================
let motorRoot = null;
let allPickables = [];
let annotationAnchors = {};
let sensorTagSprites = [];

function registerPart(id, group, explodeOffset = null) {
  group.userData.partId = id;
  group.traverse((obj) => {
    if (obj.isMesh || obj.isSprite) {
      if (!obj.userData.partId) obj.userData.partId = id;
      if (obj.isMesh) obj.userData.partId = obj.userData.partId || id;
    }
  });
  partGroups.set(id, group);
  partBasePositions.set(id, group.position.clone());
  if (explodeOffset) {
    animationGroups.set(id, {
      group,
      base: group.position.clone(),
      offset: new THREE.Vector3(...explodeOffset)
    });
  }
  return group;
}

function buildMotor() {
  motorRoot = new THREE.Group();
  motorRoot.name = 'Y90S-4 motor';
  scene.add(motorRoot);
  groups.root = motorRoot;

  buildBase();
  buildFrame();
  buildEndCaps();
  buildStator();
  buildRotor();
  buildFanAndCowl();
  buildTerminalBox();
  buildAlarmBeacon();
  buildSensors();
  buildDaqAndCables();
  buildJamAndFaultMarkers();
  buildFieldVisuals();
  buildAirflow();
  buildAnnotations();
  finalizePickables();

  groups.frontCap.visible = true;
  groups.frameFull.visible = true;
  groups.frameCut.visible = false;
  groups.nameplateGroup.visible = true;
  groups.field.visible = false;
  groups.jam.visible = false;
  if (groups.brokenBarMarkers) groups.brokenBarMarkers.visible = false;
  if (groups.bearingMarkers) groups.bearingMarkers.visible = false;
  if (groups.airflow) groups.airflow.visible = false;
  ui.explodeAmount = 0;
  ui.explodeTarget = 0;
}

function finalizePickables() {
  allPickables = [];
  const seen = new Set();
  motorRoot.traverse((obj) => {
    if (obj.isMesh && obj.userData.partId && obj.visible !== false) {
      if (!seen.has(obj.uuid)) {
        seen.add(obj.uuid);
        allPickables.push(obj);
      }
    }
  });
}

// ===== 18. 底座 =====
function buildBase() {
  const g = new THREE.Group();
  g.name = 'integral-base';
  groups.base = g;
  motorRoot.add(g);

  const plate = mesh(roundedBoxGeo(1.84, 0.17, 3.12, 0.075), materialLib.castIronDark, 'base', g);
  plate.position.set(0, -1.055, 0.03);
  const topPlate = mesh(roundedBoxGeo(1.68, 0.035, 3.02, 0.06), materialLib.castIron, 'base', g);
  topPlate.position.set(0, -0.967, 0.03);

  // longitudinal machine pads under the frame
  for (const z of [-0.66, 0.66]) {
    const pad = mesh(roundedBoxGeo(1.54, 0.125, 0.51, 0.05), materialLib.castIron, 'base', g);
    pad.position.set(0, -0.88, z);
  }
  // four mounting positions, visually drilled through the base pads
  const holePos = [[-0.60, -1.10], [0.60, -1.10], [-0.60, 1.16], [0.60, 1.16]];
  for (const [x, z] of holePos) {
    const hole = new THREE.Mesh(new THREE.CylinderGeometry(0.065, 0.065, 0.025, 30), materialLib.blackPolymer);
    hole.position.set(x, -0.943, z);
    g.add(hole);
    const rim = new THREE.Mesh(new THREE.TorusGeometry(0.077, 0.008, 6, 30), materialLib.machinedEdge);
    rim.rotation.x = Math.PI / 2;
    rim.position.set(x, -0.94, z);
    g.add(rim);
    const bolt = new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.042, 0.05, 6), materialLib.steelDark);
    bolt.position.set(x, -0.956, z);
    g.add(bolt);
  }
  // recessed side slots add a mechanical read without decorative noise
  for (const x of [-0.9, 0.9]) {
    for (const z of [-0.45, 0.55]) {
      const slot = mesh(roundedBoxGeo(0.07, 0.022, 0.22, 0.02), materialLib.blackMetal, 'base', g);
      slot.position.set(x, -0.975, z);
    }
  }
  // Earth terminal on the side of the base.
  const earth = new THREE.Group();
  const post = new THREE.Mesh(new THREE.CylinderGeometry(0.036, 0.042, 0.065, 16), materialLib.brass);
  post.rotation.z = Math.PI / 2;
  post.position.set(0.87, -0.88, -0.88);
  earth.add(post);
  const earthScrew = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.024, 0.075, 6), materialLib.steelDark);
  earthScrew.rotation.z = Math.PI / 2;
  earthScrew.position.set(0.91, -0.88, -0.88);
  earth.add(earthScrew);
  const earthLabel = makeSensorTagSprite('PE', '#d9b879');
  earthLabel.scale.multiplyScalar(0.24);
  earthLabel.position.set(0.99, -0.78, -0.88);
  earthLabel.visible = false;
  earth.add(earthLabel);
  g.add(earth);
  registerPart('base', g);
}

// ===== 19. 机座 / 散热筋 / 铭牌 =====
function buildFrame() {
  // Full exterior body (assembly view)
  const full = new THREE.Group();
  full.name = 'frame-full';
  groups.frameFull = full;
  motorRoot.add(full);

  const shell = mesh(cylZ(MOTOR.bodyR, MOTOR.bodyR * 0.995, MOTOR.bodyLen, 72, true), materialLib.castIron, 'frame', full);
  shell.rotation.y = 0;
  const innerRimF = mesh(new THREE.TorusGeometry(MOTOR.bodyR * 0.985, 0.035, 8, 64), materialLib.castIronDark, 'frame', full);
  innerRimF.position.z = MOTOR.bodyZ1 - 0.008;
  const innerRimR = innerRimF.clone();
  innerRimR.position.z = MOTOR.bodyZ0 + 0.008;
  full.add(innerRimR);

  // Circumferential cooling fins — instanced for efficiency.
  const finCount = 13;
  const finGeo = new THREE.TorusGeometry(MOTOR.bodyR + 0.028, 0.033, 6, 72);
  const fins = new THREE.InstancedMesh(finGeo, materialLib.castIron, finCount);
  fins.castShadow = true; fins.receiveShadow = true;
  fins.userData.partId = 'frame';
  const mtx = new THREE.Matrix4();
  for (let i = 0; i < finCount; i++) {
    const z = lerp(MOTOR.bodyZ0 + 0.12, MOTOR.bodyZ1 - 0.12, i / (finCount - 1));
    mtx.makeTranslation(0, 0, z);
    fins.setMatrixAt(i, mtx);
  }
  fins.instanceMatrix.needsUpdate = true;
  full.add(fins);
  allPickables.push(fins);

  // low longitudinal ribs at four positions create cast silhouette breaks
  for (let i = 0; i < 4; i++) {
    const a = Math.PI / 4 + i * Math.PI / 2;
    const rib = mesh(new THREE.BoxGeometry(0.035, 0.055, 1.84), materialLib.castIronDark, 'frame', full);
    rib.position.set(
      Math.sin(a) * (MOTOR.bodyR + 0.045),
      Math.cos(a) * (MOTOR.bodyR + 0.045),
      0
    );
    rib.rotation.z = -a;
  }

  // terminal base pad
  const pad = mesh(roundedBoxGeo(0.58, 0.055, 0.82, 0.03), materialLib.castIronDark, 'frame', full);
  pad.position.set(0, MOTOR.bodyR + 0.02, 0.14);

  // lifting eye
  const eye = new THREE.Group();
  const eyeBase = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.095, 0.07, 20), materialLib.castIronDark);
  eyeBase.position.set(-0.34, MOTOR.bodyR + 0.03, -0.66);
  eye.add(eyeBase);
  const eyeRing = new THREE.Mesh(new THREE.TorusGeometry(0.105, 0.026, 8, 20, Math.PI * 1.45), materialLib.castIron);
  eyeRing.position.set(-0.34, MOTOR.bodyR + 0.16, -0.66);
  eyeRing.rotation.x = Math.PI / 2;
  eyeRing.rotation.z = 0.35;
  eye.add(eyeRing);
  full.add(eye);

  // longitudinal cast seam on the far side
  const seam = mesh(new THREE.BoxGeometry(0.012, 0.012, 1.9), materialLib.castIronDark, 'frame', full);
  seam.position.set(-MOTOR.bodyR * 0.999, 0.18, 0);
  seam.rotation.z = 0.28;

  registerPart('frame', full);

  // Section shell (240° cut-away, with visible wall thickness)
  const cut = new THREE.Group();
  cut.name = 'frame-cut';
  groups.frameCut = cut;
  motorRoot.add(cut);
  const cutWall = mesh(
    annularSectorGeo(MOTOR.bodyR, MOTOR.bodyR - 0.11, MOTOR.cutStart, MOTOR.cutLength, MOTOR.bodyLen, 110),
    materialLib.castIron,
    'frame',
    cut
  );
  const cutFinGeo = new THREE.TorusGeometry(MOTOR.bodyR + 0.028, 0.033, 6, 48, MOTOR.cutLength);
  cutFinGeo.rotateZ(MOTOR.cutStart);
  const cutFins = new THREE.InstancedMesh(cutFinGeo, materialLib.castIron, 13);
  cutFins.castShadow = true; cutFins.receiveShadow = true;
  cutFins.userData.partId = 'frame';
  const m2 = new THREE.Matrix4();
  for (let i = 0; i < 13; i++) {
    const z = lerp(MOTOR.bodyZ0 + 0.12, MOTOR.bodyZ1 - 0.12, i / 12);
    m2.makeTranslation(0, 0, z);
    cutFins.setMatrixAt(i, m2);
  }
  cutFins.instanceMatrix.needsUpdate = true;
  cut.add(cutFins);
  allPickables.push(cutFins);
  // machined cut faces at both ends of the removed sector
  for (const a of [MOTOR.cutStart, MOTOR.cutStart + MOTOR.cutLength]) {
    const cutFace = new THREE.Mesh(new THREE.BoxGeometry(0.112, 0.006, MOTOR.bodyLen), materialLib.machinedEdge);
    cutFace.position.set(Math.cos(a) * (MOTOR.bodyR - 0.055), Math.sin(a) * (MOTOR.bodyR - 0.055), 0);
    cutFace.rotation.z = a;
    cutFace.castShadow = true;
    cutFace.userData.partId = 'frame';
    cut.add(cutFace);
  }
  const cutRimF = new THREE.Mesh(new THREE.TorusGeometry(MOTOR.bodyR - 0.05, 0.021, 6, 72, MOTOR.cutLength), materialLib.machinedEdge);
  cutRimF.geometry.rotateZ(MOTOR.cutStart);
  cutRimF.position.z = MOTOR.bodyZ1;
  cut.add(cutRimF);
  const cutRimR = cutRimF.clone();
  cutRimR.position.z = MOTOR.bodyZ0;
  cut.add(cutRimR);
  // dust/vent holes on the lower shell
  for (let i = 0; i < 5; i++) {
    const hole = new THREE.Mesh(new THREE.CircleGeometry(0.04, 18), materialLib.blackPolymer);
    const a = Math.PI * 0.63 + i * 0.12;
    hole.position.set(Math.cos(a) * (MOTOR.bodyR + 0.001), Math.sin(a) * (MOTOR.bodyR + 0.001), -0.84 + i * 0.12);
    hole.lookAt(new THREE.Vector3(Math.cos(a) * 2, Math.sin(a) * 2, hole.position.z));
    cut.add(hole);
  }
  registerPart('frame', full); // maintain primary hit group for exterior

  // nameplate (separate part) and small manufacturer badge
  const np = new THREE.Group();
  np.name = 'nameplate';
  groups.nameplateGroup = np;
  motorRoot.add(np);
  // The plate stands proud of the fin tips so it is actually readable from the
  // 3/4 view; a cast pad visually carries it back to the frame.
  const nameplatePad = new THREE.Mesh(roundedBoxGeo(0.06, 0.35, 0.70, 0.022), materialLib.castIronDark);
  nameplatePad.position.set(0.85, -0.055, 0.61);
  nameplatePad.userData.partId = 'nameplate';
  np.add(nameplatePad);
  const backing = new THREE.Mesh(roundedBoxGeo(0.025, 0.375, 0.73, 0.014), materialLib.steelDark);
  backing.position.set(0.897, -0.055, 0.61);
  np.add(backing);
  const plate = new THREE.Mesh(new THREE.PlaneGeometry(0.77, 0.415), materialLib.nameplate);
  plate.rotation.y = Math.PI / 2;
  plate.position.set(0.927, -0.055, 0.61);
  plate.castShadow = false;
  plate.userData.partId = 'nameplate';
  np.add(plate);
  // rivets
  for (const z of [0.975, 0.245]) {
    const rivet = new THREE.Mesh(new THREE.SphereGeometry(0.016, 10, 8), materialLib.steelDark);
    rivet.position.set(0.932, 0.14, z);
    np.add(rivet);
  }
  const badge = new THREE.Mesh(new THREE.CircleGeometry(0.075, 24), materialLib.blackMetal);
  badge.rotation.y = Math.PI / 2;
  badge.position.set(MOTOR.bodyR + 0.038, 0.36, -0.28);
  np.add(badge);
  const badgeRing = new THREE.Mesh(new THREE.TorusGeometry(0.076, 0.006, 6, 28), materialLib.machinedEdge);
  badgeRing.rotation.y = Math.PI / 2;
  badgeRing.position.copy(badge.position);
  np.add(badgeRing);
  registerPart('nameplate', np);

  // casting ribs / mounting ears near the base, part of the frame
  for (const z of [-0.84, 0.84]) {
    const foot = mesh(roundedBoxGeo(0.5, 0.17, 0.38, 0.035), materialLib.castIronDark, 'frame', full);
    foot.position.set(0, -0.79, z);
  }
}

// ===== 20. 端盖 / 轴承压盖 =====
function buildEndCaps() {
  const front = buildSingleEndCap(true);
  const rear = buildSingleEndCap(false);
  groups.frontCap = front;
  groups.rearCap = rear;
  animationGroups.set('frontCap', {
    group: front, base: front.position.clone(), offset: new THREE.Vector3(0.02, 0.03, 1.18)
  });
  animationGroups.set('rearCap', {
    group: rear, base: rear.position.clone(), offset: new THREE.Vector3(-0.01, 0.01, -0.35)
  });
}

function buildSingleEndCap(front) {
  const g = new THREE.Group();
  g.name = front ? 'front-end-cap' : 'rear-end-cap';
  motorRoot.add(g);
  const dir = front ? 1 : -1;
  const z0 = front ? MOTOR.frontCapZ : MOTOR.rearCapZ;
  const points = [
    new THREE.Vector2(0.155, z0 + 0.09 * dir),
    new THREE.Vector2(0.19, z0 + 0.15 * dir),
    new THREE.Vector2(0.30, z0 + 0.16 * dir),
    new THREE.Vector2(0.35, z0 + 0.11 * dir),
    new THREE.Vector2(0.54, z0 + 0.10 * dir),
    new THREE.Vector2(0.70, z0 + 0.075 * dir),
    new THREE.Vector2(0.805, z0 + 0.01 * dir),
    new THREE.Vector2(0.79, z0 - 0.09 * dir),
    new THREE.Vector2(0.71, z0 - 0.16 * dir),
    new THREE.Vector2(0.30, z0 - 0.19 * dir),
    new THREE.Vector2(0.20, z0 - 0.14 * dir),
    new THREE.Vector2(0.155, z0 - 0.10 * dir)
  ];
  const geo = latheZ(points, 80);
  const shell = mesh(geo, materialLib.castIron, front ? 'frontCap' : 'rearCap', g);
  shell.material = materialLib.castIron;
  // The lathe surfaces are open; close the rear with a dark ring plate.
  const back = new THREE.Mesh(
    new THREE.RingGeometry(0.16, 0.78, 64),
    materialLib.castIronDark
  );
  back.position.z = z0 - 0.19 * dir;
  back.rotation.y = front ? Math.PI : 0;
  back.userData.partId = front ? 'frontCap' : 'rearCap';
  g.add(back);

  const flange = mesh(new THREE.TorusGeometry(0.795, 0.028, 8, 72), materialLib.castIronDark, front ? 'frontCap' : 'rearCap', g);
  flange.position.z = z0 - 0.02 * dir;
  // raised machined face ring
  const faceRing = mesh(new THREE.TorusGeometry(0.585, 0.008, 5, 64), materialLib.machinedEdge, front ? 'frontCap' : 'rearCap', g);
  faceRing.position.z = z0 + 0.105 * dir;
  // bearing cover / grease retainer
  const cover = mesh(new THREE.CylinderGeometry(0.255, 0.255, 0.045, 40), materialLib.steelDark, front ? 'frontCap' : 'rearCap', g);
  cover.rotation.x = Math.PI / 2;
  cover.position.z = z0 + 0.17 * dir;
  const coverRing = mesh(new THREE.TorusGeometry(0.258, 0.018, 7, 40), materialLib.machinedEdge, front ? 'frontCap' : 'rearCap', g);
  coverRing.position.z = z0 + 0.195 * dir;
  for (let i = 0; i < 4; i++) {
    const a = i / 4 * TAU + Math.PI / 4;
    addHexBolt(g, Math.cos(a) * 0.205, Math.sin(a) * 0.205, z0 + 0.198 * dir, 0.027, a);
  }
  const centerSeal = new THREE.Mesh(new THREE.TorusGeometry(0.155, 0.017, 6, 32), materialLib.rubber);
  centerSeal.position.z = z0 + 0.19 * dir;
  g.add(centerSeal);

  for (let i = 0; i < 6; i++) {
    const a = i / 6 * TAU + Math.PI / 6;
    addHexBolt(g, Math.cos(a) * 0.66, Math.sin(a) * 0.66, z0 + 0.10 * dir, 0.035, a);
  }
  // small cast ribs at the 3/6/9/12 positions
  for (let i = 0; i < 4; i++) {
    const a = i / 4 * TAU;
    const rib = mesh(new THREE.BoxGeometry(0.17, 0.034, 0.035), materialLib.castIron, front ? 'frontCap' : 'rearCap', g);
    rib.position.set(Math.cos(a) * 0.54, Math.sin(a) * 0.54, z0 + 0.115 * dir);
    rib.rotation.z = a;
  }
  registerPart(front ? 'frontCap' : 'rearCap', g);
  return g;
}

// ===== 21. 定子：铁芯、24 槽、三相绕组 =====
function buildStator() {
  const root = new THREE.Group();
  root.name = 'stator';
  groups.stator = root;
  motorRoot.add(root);

  const coreGroup = new THREE.Group();
  const windGroup = new THREE.Group();
  root.add(coreGroup); root.add(windGroup);

  const yoke = mesh(
    annularSectorGeo(MOTOR.statorOuter, 0.50, MOTOR.cutStart, MOTOR.cutLength, MOTOR.statorLen, 120),
    materialLib.siliconSteel,
    'statorCore',
    coreGroup
  );
  // machined-looking cut edges expose the lamination thickness
  for (const a of [MOTOR.cutStart, MOTOR.cutStart + MOTOR.cutLength]) {
    const edge = new THREE.Mesh(new THREE.BoxGeometry(0.165, 0.004, MOTOR.statorLen), materialLib.siliconEdge);
    edge.position.set(Math.cos(a) * 0.582, Math.sin(a) * 0.582, 0);
    edge.rotation.z = a;
    edge.userData.partId = 'statorCore';
    coreGroup.add(edge);
  }
  // stacked lamination lines around the outer yoke
  const lamiGeo = new THREE.TorusGeometry(MOTOR.statorOuter + 0.001, 0.0027, 4, 64, MOTOR.cutLength);
  lamiGeo.rotateZ(MOTOR.cutStart);
  const lami = new THREE.InstancedMesh(lamiGeo, materialLib.castIronDark, 18);
  lami.castShadow = false;
  lami.userData.partId = 'statorCore';
  const lm = new THREE.Matrix4();
  for (let i = 0; i < 18; i++) {
    lm.makeTranslation(0, 0, lerp(-MOTOR.statorLen * 0.46, MOTOR.statorLen * 0.46, i / 17));
    lami.setMatrixAt(i, lm);
  }
  lami.instanceMatrix.needsUpdate = true;
  coreGroup.add(lami);

  const toothGeo = new THREE.BoxGeometry(0.105, 0.066, MOTOR.statorLen);
  const toothCount = 16;
  const teeth = new THREE.InstancedMesh(toothGeo, materialLib.siliconSteel, toothCount);
  teeth.castShadow = true; teeth.receiveShadow = true;
  teeth.userData.partId = 'statorCore';
  const slotAngles = [];
  const tm = new THREE.Matrix4();
  const quat = new THREE.Quaternion();
  const pos = new THREE.Vector3();
  const one = new THREE.Vector3(1, 1, 1);
  const toothAngles = [];
  let tIdx = 0;
  for (let i = 0; i < 24; i++) {
    const a = i / 24 * TAU;
    if (a + 0.001 < MOTOR.cutStart) continue;
    toothAngles.push(a);
    quat.setFromAxisAngle(new THREE.Vector3(0, 0, 1), a);
    pos.set(Math.cos(a) * 0.452, Math.sin(a) * 0.452, 0);
    tm.compose(pos, quat, one);
    teeth.setMatrixAt(tIdx++, tm);
  }
  for (let i = 0; i < toothCount; i++) {
    if (i >= tIdx) {
      const a = toothAngles[tIdx - 1] ?? MOTOR.cutStart;
      quat.setFromAxisAngle(new THREE.Vector3(0, 0, 1), a);
      pos.set(Math.cos(a) * 0.452, Math.sin(a) * 0.452, 0);
      tm.compose(pos, quat, one);
      teeth.setMatrixAt(i, tm);
    }
  }
  teeth.count = tIdx;
  teeth.instanceMatrix.needsUpdate = true;
  coreGroup.add(teeth);

  // slot centres sit halfway between adjacent teeth
  for (let i = 0; i < 24; i++) {
    const a = i / 24 * TAU + Math.PI / 24;
    if (a < MOTOR.cutStart - 0.016) continue;
    slotAngles.push(a);
  }
  // Slot liner
  const liner = new THREE.InstancedMesh(
    new THREE.BoxGeometry(0.09, 0.016, MOTOR.statorLen * 0.995),
    materialLib.insulator,
    slotAngles.length
  );
  liner.userData.partId = 'windings';
  const cm = new THREE.Matrix4();
  const cq = new THREE.Quaternion();
  const cp = new THREE.Vector3();
  slotAngles.forEach((a, i) => {
    cq.setFromAxisAngle(new THREE.Vector3(0, 0, 1), a);
    cp.set(Math.cos(a) * 0.452, Math.sin(a) * 0.452, 0);
    cm.compose(cp, cq, one);
    liner.setMatrixAt(i, cm);
  });
  liner.instanceMatrix.needsUpdate = true;
  coreGroup.add(liner);

  // Three coloured phase conductors; visible through the cutaway.
  const copperMats = [materialLib.copperU, materialLib.copperV, materialLib.copperW];
  groups.windingMats = copperMats;
  const phaseCounts = [0, 0, 0];
  slotAngles.forEach((a, i) => phaseCounts[i % 3]++);
  const slotIdx = [0, 0, 0];
  const wireGeos = [0, 1, 2].map(() => new THREE.BoxGeometry(0.048, 0.026, MOTOR.statorLen * 0.985));
  for (let p = 0; p < 3; p++) {
    const inst = new THREE.InstancedMesh(wireGeos[p], copperMats[p], phaseCounts[p]);
    inst.castShadow = true;
    inst.userData.partId = 'windings';
    slotAngles.forEach((a, i) => {
      if (i % 3 !== p) return;
      cq.setFromAxisAngle(new THREE.Vector3(0, 0, 1), a);
      cp.set(Math.cos(a) * 0.452, Math.sin(a) * 0.452, 0);
      cm.compose(cp, cq, one);
      inst.setMatrixAt(slotIdx[p]++, cm);
    });
    inst.instanceMatrix.needsUpdate = true;
    windGroup.add(inst);
  }

  // End-winding overhangs, low-saturation U/V/W identification.
  const arcGeoms = [0, 1, 2].map(() => {
    const geo = new THREE.TorusGeometry(0.435, 0.034 + 0.004, 7, 18, 0.22);
    return geo;
  });
  slotAngles.forEach((a, i) => {
    const mat = copperMats[i % 3];
    for (const dz of [-0.795, 0.795]) {
      const arc = mesh(arcGeoms[i % 3], mat, 'windings', windGroup);
      arc.rotation.z = a - 0.11;
      arc.position.z = dz;
    }
  });
  // phase leads rising toward the terminal box
  const leadColours = [0xb98a60, 0xb3a078, 0xc2a87e];
  for (let i = 0; i < 3; i++) {
    const curve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(-0.20 + i * 0.16, 0.48, 0.64),
      new THREE.Vector3(-0.10 + i * 0.12, 0.74, 0.48),
      new THREE.Vector3(-0.05 + i * 0.06, 0.84, 0.12)
    ]);
    const lead = new THREE.Mesh(
      new THREE.TubeGeometry(curve, 20, 0.015, 5, false),
      new THREE.MeshStandardMaterial({ color: leadColours[i], metalness: 0.7, roughness: 0.42 })
    );
    lead.userData.partId = 'windings';
    windGroup.add(lead);
  }
  // Fine lamination lines on the internal bore are added as dark thin arcs.
  const boreLines = new THREE.InstancedMesh(
    new THREE.TorusGeometry(0.665, 0.0018, 4, 48, MOTOR.cutLength),
    materialLib.castIronDark, 16
  );
  boreLines.geometry.rotateZ(MOTOR.cutStart);
  for (let i = 0; i < 16; i++) {
    lm.makeTranslation(0, 0, lerp(-0.72, 0.72, i / 15));
    boreLines.setMatrixAt(i, lm);
  }
  boreLines.instanceMatrix.needsUpdate = true;
  boreLines.userData.partId = 'statorCore';
  coreGroup.add(boreLines);

  registerPart('statorCore', coreGroup);
  registerPart('windings', windGroup);

  // Air-gap reference ring (thin, only visible when sectioned)
  const gap = new THREE.Mesh(
    new THREE.TorusGeometry(0.402, 0.0022, 4, 80, MOTOR.cutLength),
    new THREE.MeshBasicMaterial({ color: 0x6c8796, transparent: true, opacity: 0.36, depthWrite: false })
  );
  gap.geometry.rotateZ(MOTOR.cutStart);
  gap.position.z = 0.79;
  gap.userData.partId = 'statorCore';
  root.add(gap);
}

// ===== 22. 转子 / 轴系 / 轴承 =====
function buildRotor() {
  const spin = new THREE.Group();
  spin.name = 'rotor-assembly';
  groups.rotorSpin = spin;
  motorRoot.add(spin);
  animationGroups.set('rotorSpin', {
    group: spin, base: spin.position.clone(), offset: new THREE.Vector3(0.0, 0.16, 0.84)
  });

  // --- rotor core ---
  const coreGroup = new THREE.Group();
  const core = mesh(cylZ(MOTOR.rotorR, MOTOR.rotorR, MOTOR.rotorLen, 64, false), materialLib.siliconSteel, 'rotorCore', coreGroup);
  spin.add(coreGroup);
  const rotorRingGeo = new THREE.TorusGeometry(MOTOR.rotorR + 0.0005, 0.0018, 4, 64);
  const rotorRings = new THREE.InstancedMesh(rotorRingGeo, materialLib.castIronDark, 18);
  rotorRings.userData.partId = 'rotorCore';
  const rm = new THREE.Matrix4();
  for (let i = 0; i < 18; i++) {
    rm.makeTranslation(0, 0, lerp(-MOTOR.rotorLen * 0.47, MOTOR.rotorLen * 0.47, i / 17));
    rotorRings.setMatrixAt(i, rm);
  }
  rotorRings.instanceMatrix.needsUpdate = true;
  coreGroup.add(rotorRings);
  // core end plates
  for (const z of [-MOTOR.rotorLen / 2 - 0.012, MOTOR.rotorLen / 2 + 0.012]) {
    const capPlate = new THREE.Mesh(new THREE.CylinderGeometry(MOTOR.rotorR, MOTOR.rotorR, 0.022, 56), materialLib.steelDark);
    capPlate.rotation.x = Math.PI / 2;
    capPlate.position.z = z;
    capPlate.userData.partId = 'rotorCore';
    coreGroup.add(capPlate);
  }
  registerPart('rotorCore', coreGroup);

  // --- squirrel cage ---
  const barsGroup = new THREE.Group();
  const barCount = 22;
  const barGeo = new THREE.BoxGeometry(0.033, 0.044, MOTOR.rotorLen * 0.98);
  const barMesh = new THREE.InstancedMesh(barGeo, materialLib.copperBar, barCount);
  barMesh.castShadow = true;
  barMesh.userData.partId = 'rotorBars';
  const barAngles = [];
  const bm = new THREE.Matrix4();
  const bq = new THREE.Quaternion();
  const bp = new THREE.Vector3();
  const bs = new THREE.Vector3(1, 1, 1);
  const copper = new THREE.Color(0xb7784b);
  for (let i = 0; i < barCount; i++) {
    const a = i / barCount * TAU;
    barAngles.push(a);
    bq.setFromAxisAngle(new THREE.Vector3(0, 0, 1), a);
    bp.set(Math.cos(a) * 0.369, Math.sin(a) * 0.369, 0);
    bm.compose(bp, bq, bs);
    barMesh.setMatrixAt(i, bm);
    barMesh.setColorAt(i, copper);
  }
  barMesh.instanceMatrix.needsUpdate = true;
  if (barMesh.instanceColor) barMesh.instanceColor.needsUpdate = true;
  barsGroup.add(barMesh);
  groups.barMesh = barMesh;
  for (const z of [-0.77, 0.77]) {
    const ring = mesh(new THREE.TorusGeometry(MOTOR.rotorR - 0.045, 0.045, 10, 64), materialLib.copperBar, 'rotorBars', barsGroup);
    ring.position.z = z;
    ring.castShadow = true;
  }
  // End-ring hub fillets
  for (const z of [-0.73, 0.73]) {
    const hub = mesh(new THREE.CylinderGeometry(0.23, 0.23, 0.05, 40), materialLib.copperDark, 'rotorBars', barsGroup);
    hub.rotation.x = Math.PI / 2;
    hub.position.z = z;
  }
  spin.add(barsGroup);
  groups.bars = barsGroup;
  registerPart('rotorBars', barsGroup);

  // --- shaft ---
  const shaftGroup = new THREE.Group();
  const segments = [
    [-1.34, -1.17, 0.09, 0.09],
    [-1.22, -0.91, 0.115, 0.115],
    [-0.95, 0.95, 0.135, 0.135],
    [0.91, 1.17, 0.118, 0.118],
    [1.17, 1.98, 0.12, 0.12],
    [1.98, 2.08, 0.105, 0.115],
    [2.08, 2.62, 0.08, 0.095]
  ];
  for (const [z0, z1, r0, r1] of segments) {
    const len = z1 - z0;
    const seg = mesh(cylZ(r1, r0, len, 36, false), materialLib.steel, 'shaft', shaftGroup);
    seg.position.z = (z0 + z1) / 2;
  }
  // shoulders and machined collars
  for (const z of [-0.93, 0.93, 1.15, 1.98]) {
    const collar = mesh(new THREE.TorusGeometry(z > 1.9 ? 0.105 : 0.13, 0.014, 6, 36), materialLib.machinedEdge, 'shaft', shaftGroup);
    collar.position.z = z;
  }
  // keyway and key
  const keyway = new THREE.Mesh(new THREE.BoxGeometry(0.038, 0.012, 0.35), materialLib.blackPolymer);
  keyway.position.set(0, 0.121, 1.62);
  keyway.userData.partId = 'shaft';
  shaftGroup.add(keyway);
  const key = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.018, 0.29), materialLib.machinedEdge);
  key.position.set(0, 0.132, 1.62);
  key.userData.partId = 'shaft';
  shaftGroup.add(key);
  // twelve-slot code disc
  const disc = new THREE.Mesh(new THREE.CylinderGeometry(0.285, 0.285, 0.022, 48), materialLib.blackMetal);
  disc.rotation.x = Math.PI / 2;
  disc.position.z = 2.10;
  disc.userData.partId = 'shaft';
  shaftGroup.add(disc);
  const discHub = new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.085, 0.06, 32), materialLib.steelDark);
  discHub.rotation.x = Math.PI / 2;
  discHub.position.z = 2.105;
  discHub.userData.partId = 'shaft';
  shaftGroup.add(discHub);
  for (let i = 0; i < 12; i++) {
    const a = i / 12 * TAU;
    const slot = new THREE.Mesh(new THREE.BoxGeometry(0.032, 0.006, 0.0025), materialLib.machinedEdge);
    slot.position.set(Math.cos(a) * 0.23, Math.sin(a) * 0.23, 2.114);
    slot.rotation.z = a;
    shaftGroup.add(slot);
  }
  // bright index mark makes slow rotation readable
  const index = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.018, 0.003), materialLib.paintRed);
  index.position.set(0.17, 0, 2.116);
  index.rotation.z = 0.12;
  shaftGroup.add(index);
  // coupling and encoder shaft
  const coupling = mesh(new THREE.CylinderGeometry(0.155, 0.155, 0.10, 32), materialLib.aluminum, 'shaft', shaftGroup);
  coupling.rotation.x = Math.PI / 2;
  coupling.position.z = 2.28;
  const encoderShaft = mesh(cylZ(0.055, 0.08, 0.34, 24), materialLib.steel, 'shaft', shaftGroup);
  encoderShaft.position.z = 2.46;
  spin.add(shaftGroup);
  registerPart('shaft', shaftGroup);

  // --- bearings ---
  const bearingFront = buildBearing('front', 1.015, 'bearingFront');
  const bearingRear = buildBearing('rear', -1.015, 'bearingRear');
  spin.add(bearingFront);
  spin.add(bearingRear);
  groups.bearingCages.push(...bearingFront.userData.cages, ...bearingRear.userData.cages);
  groups.bearingFront = bearingFront;
  groups.bearingRear = bearingRear;
  registerPart('bearingFront', bearingFront);
  registerPart('bearingRear', bearingRear);
}

function buildBearing(position, z, partId) {
  const g = new THREE.Group();
  const outer = new THREE.Mesh(new THREE.TorusGeometry(0.222, 0.035, 12, 48), materialLib.bearingSteel);
  outer.position.z = z;
  outer.castShadow = true;
  g.add(outer);
  const inner = new THREE.Mesh(new THREE.TorusGeometry(0.156, 0.023, 12, 40), materialLib.bearingSteel);
  inner.position.z = z;
  inner.castShadow = true;
  g.add(inner);
  const cage = new THREE.Group();
  cage.position.z = z;
  const cageRing = new THREE.Mesh(new THREE.TorusGeometry(0.188, 0.006, 5, 44), materialLib.bearingBrass);
  cage.add(cageRing);
  const ballGeo = new THREE.SphereGeometry(0.042, 16, 12);
  for (let i = 0; i < 9; i++) {
    const a = i / 9 * TAU;
    const ball = new THREE.Mesh(ballGeo, materialLib.bearingSteel);
    ball.position.set(Math.cos(a) * 0.189, Math.sin(a) * 0.189, 0);
    ball.castShadow = true;
    cage.add(ball);
  }
  g.add(cage);
  g.userData.cages = [cage];
  // Dust shields are represented by two dark outer lips; balls remain visible
  // in the teaching cutaway.
  const shield = new THREE.Mesh(new THREE.TorusGeometry(0.205, 0.013, 6, 44), materialLib.blackMetal);
  shield.position.z = z + 0.055;
  g.add(shield);
  g.traverse((obj) => { if (obj.isMesh) obj.userData.partId = partId; });
  // Place the full bearing assembly so that its axial centre sits at z.
  g.position.z = 0; // geometry above already uses absolute z
  g.userData.axisZ = z;
  g.name = `${partId}-${position}`;
  return g;
}

// ===== 23. 风扇 / 风罩 / 冷却气流示意 =====
function buildFanAndCowl() {
  const fan = new THREE.Group();
  fan.name = 'radial-fan';
  motorRoot.add(fan);
  groups.fan = fan;
  animationGroups.set('fan', {
    group: fan, base: fan.position.clone(), offset: new THREE.Vector3(-0.02, 0.02, -0.72)
  });
  const hub = mesh(new THREE.CylinderGeometry(0.17, 0.19, 0.24, 36), materialLib.fanDark, 'fan', fan);
  hub.rotation.x = Math.PI / 2;
  hub.position.z = MOTOR.fanZ;
  const hubCap = mesh(new THREE.SphereGeometry(0.15, 28, 14), materialLib.steelDark, 'fan', fan);
  hubCap.scale.z = 0.42;
  hubCap.position.z = MOTOR.fanZ - 0.13;
  for (let i = 0; i < 8; i++) {
    const a = i / 8 * TAU;
    const bladePivot = new THREE.Group();
    bladePivot.rotation.z = a;
    const blade = mesh(new THREE.BoxGeometry(0.48, 0.155, 0.032), materialLib.fan, 'fan', bladePivot);
    blade.position.set(0.36, 0, MOTOR.fanZ);
    blade.rotation.x = 0.38;
    blade.rotation.y = -0.12;
    // softened leading edge
    const edge = mesh(new THREE.CylinderGeometry(0.032, 0.012, 0.46, 8), materialLib.aluminum, 'fan', bladePivot);
    edge.rotation.z = Math.PI / 2;
    edge.position.set(0.36, 0.09, MOTOR.fanZ);
    fan.add(bladePivot);
  }
  const fanNut = mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.05, 6), materialLib.steelDark, 'fan', fan);
  fanNut.rotation.x = Math.PI / 2;
  fanNut.position.z = MOTOR.fanZ - 0.28;
  registerPart('fan', fan);

  const cowl = new THREE.Group();
  cowl.name = 'fan-cowl';
  motorRoot.add(cowl);
  groups.fanCowl = cowl;
  animationGroups.set('fanCowl', {
    group: cowl, base: cowl.position.clone(), offset: new THREE.Vector3(-0.02, 0.04, -1.22)
  });
  const outerR = 0.72;
  const cowlZ = -1.79;
  // Side cage with 12 radial guards.
  for (let i = 0; i < 12; i++) {
    const a = i / 12 * TAU;
    const bar = mesh(new THREE.BoxGeometry(0.018, 0.018, 0.55), materialLib.steelDark, 'fanCowl', cowl);
    bar.position.set(Math.cos(a) * outerR, Math.sin(a) * outerR, -1.52);
    bar.rotation.z = a;
  }
  for (const z of [-1.25, -1.81]) {
    const ring = mesh(new THREE.TorusGeometry(outerR, 0.021, 7, 48), materialLib.steelDark, 'fanCowl', cowl);
    ring.position.z = z;
  }
  // Rear perforated grille: concentric rings and radial spokes.
  for (const r of [0.20, 0.38, 0.55, 0.69]) {
    const ring = mesh(new THREE.TorusGeometry(r, 0.013, 5, 48), materialLib.steelDark, 'fanCowl', cowl);
    ring.position.z = cowlZ - 0.02;
  }
  for (let i = 0; i < 10; i++) {
    const a = i / 10 * TAU;
    const spoke = mesh(new THREE.BoxGeometry(0.67, 0.016, 0.017), materialLib.steelDark, 'fanCowl', cowl);
    spoke.position.set(Math.cos(a) * 0.35, Math.sin(a) * 0.35, cowlZ - 0.025);
    spoke.rotation.z = a;
  }
  const cowlCenter = mesh(new THREE.CylinderGeometry(0.12, 0.15, 0.06, 28), materialLib.blackMetal, 'fanCowl', cowl);
  cowlCenter.rotation.x = Math.PI / 2;
  cowlCenter.position.z = cowlZ - 0.025;
  const cowlMount = mesh(new THREE.TorusGeometry(0.70, 0.025, 7, 64), materialLib.castIronDark, 'fanCowl', cowl);
  cowlMount.position.z = -1.235;
  registerPart('fanCowl', cowl);
}

// ===== 24. 接线盒与 △/Y 连片 =====
function buildTerminalBox() {
  const g = new THREE.Group();
  g.name = 'terminal-box';
  groups.terminal = g;
  motorRoot.add(g);

  const y0 = MOTOR.bodyR + 0.075;
  const plinth = mesh(roundedBoxGeo(0.73, 0.075, 0.91, 0.04), materialLib.castIronDark, 'terminal', g);
  plinth.position.set(0, y0, 0.14);
  const basePlate = mesh(roundedBoxGeo(0.70, 0.035, 0.88, 0.035), materialLib.castIron, 'terminal', g);
  basePlate.position.set(0, y0 + 0.06, 0.14);
  // Four cast walls form an open terminal chamber under the removable lid.
  const wallMat = materialLib.castIron;
  const wallL = mesh(roundedBoxGeo(0.045, 0.20, 0.84, 0.018), wallMat, 'terminal', g);
  wallL.position.set(-0.335, y0 + 0.17, 0.14);
  const wallR = wallL.clone(); wallR.position.x = 0.335; g.add(wallR);
  const wallF = mesh(roundedBoxGeo(0.66, 0.20, 0.045, 0.018), wallMat, 'terminal', g);
  wallF.position.set(0, y0 + 0.17, 0.575);
  const wallB = wallF.clone(); wallB.position.z = -0.295; g.add(wallB);
  // dark interior tray
  const tray = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.012, 0.75), materialLib.blackPolymer);
  tray.position.set(0, y0 + 0.105, 0.14);
  tray.userData.partId = 'terminal';
  g.add(tray);
  // terminal posts, 3 × 2
  const posts = [];
  const postGeo = new THREE.CylinderGeometry(0.031, 0.034, 0.13, 12);
  const nutGeo = new THREE.CylinderGeometry(0.046, 0.046, 0.028, 6);
  const labels = ['U1', 'V1', 'W1', 'W2', 'U2', 'V2'];
  for (let row = 0; row < 2; row++) {
    for (let col = 0; col < 3; col++) {
      const x = (col - 1) * 0.21;
      const z = 0.14 + (row === 0 ? 0.19 : -0.19);
      const post = new THREE.Mesh(postGeo, materialLib.brass);
      post.position.set(x, y0 + 0.17, z);
      post.userData.partId = 'terminal';
      g.add(post);
      const nut = new THREE.Mesh(nutGeo, materialLib.brass);
      nut.position.set(x, y0 + 0.238, z);
      nut.userData.partId = 'terminal';
      g.add(nut);
      const lab = makeSensorTagSprite(labels[row * 3 + col], '#d6c293');
      lab.scale.multiplyScalar(0.52);
      lab.position.set(x + 0.05, y0 + 0.33, z + 0.02);
      lab.visible = false;
      g.add(lab);
      posts.push({ x, z, label: labels[row * 3 + col], labelSprite: lab });
    }
  }
  groups.terminalPosts = posts;
  groups.terminalInternals = g;

  // shared copper straps for Δ / Y animation
  const strapGroup = new THREE.Group();
  g.add(strapGroup);
  const straps = [];
  for (let i = 0; i < 3; i++) {
    const meshStrap = new THREE.Mesh(new THREE.BoxGeometry(0.046, 0.024, 0.36), materialLib.brass);
    meshStrap.castShadow = true;
    meshStrap.userData.partId = 'terminal';
    meshStrap.userData.delta = {
      pos: new THREE.Vector3((i - 1) * 0.21, y0 + 0.245, 0.14),
      quat: new THREE.Quaternion(),
      scale: new THREE.Vector3(1, 1, 1)
    };
    meshStrap.userData.wye = {
      pos: new THREE.Vector3((i - 1) * 0.115, y0 + 0.245, 0.33),
      quat: new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2),
      scale: new THREE.Vector3(1.2, 1, 0.66)
    };
    strapGroup.add(meshStrap);
    straps.push(meshStrap);
  }
  groups.terminalStraps = straps;

  // Removable lid, saved separately for animation.
  const lid = new THREE.Group();
  lid.position.set(0, y0 + 0.285, 0.14);
  g.add(lid);
  const lidBody = mesh(roundedBoxGeo(0.75, 0.075, 0.95, 0.04), materialLib.castIron, 'terminal', lid);
  lid.position.set(0, y0 + 0.285, 0.14);
  lidBody.position.set(0, 0, 0);
  const lidPlate = mesh(roundedBoxGeo(0.61, 0.012, 0.81, 0.025), materialLib.castIronLight, 'terminal', lid);
  lidPlate.position.set(0, 0.045, 0);
  for (let i = 0; i < 4; i++) {
    const x = (i % 2 ? 1 : -1) * 0.28;
    const z = (i < 2 ? -1 : 1) * 0.35;
    const bolt = new THREE.Mesh(new THREE.CylinderGeometry(0.031, 0.031, 0.022, 6), materialLib.steelDark);
    bolt.position.set(x, 0.058, z);
    bolt.userData.partId = 'terminal';
    lid.add(bolt);
  }
  groups.terminalLid = lid;
  lid.userData.basePosition = lid.position.clone();

  // cable glands and a ventilation label
  for (const x of [-0.22, 0.22]) {
    const gland = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.06, 0.1, 20), materialLib.steelDark);
    gland.rotation.z = Math.PI / 2;
    gland.position.set(x, y0 + 0.06, -0.32);
    gland.userData.partId = 'terminal';
    g.add(gland);
  }
  const labelTex = makeLabelTexture('Δ / Y', { color: '#d5c09a', size: 44, bg: 'rgba(30,30,28,.8)', border: 'rgba(190,171,137,.5)' });
  const label = new THREE.Mesh(new THREE.PlaneGeometry(0.14, 0.07), new THREE.MeshBasicMaterial({ map: labelTex, transparent: true }));
  label.position.set(0.245, y0 + 0.12, 0.58);
  label.userData.partId = 'terminal';
  g.add(label);
  registerPart('terminal', g);
}

// ===== 25. 声光报警灯 =====
function buildAlarmBeacon() {
  const g = new THREE.Group();
  g.name = 'alarm-beacon';
  const x = 0.43, z = -0.73, baseY = MOTOR.bodyR + 0.02;
  const mount = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.09, 0.07, 18), materialLib.castIronDark);
  mount.position.set(x, baseY + 0.03, z);
  mount.userData.partId = 'frame';
  g.add(mount);
  const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.21, 12), materialLib.steelDark);
  stem.position.set(x, baseY + 0.15, z);
  stem.userData.partId = 'frame';
  g.add(stem);
  const makeLens = (color, emissive, y) => {
    const mat = new THREE.MeshStandardMaterial({
      color, emissive, emissiveIntensity: 0.1,
      transparent: true, opacity: 0.82, roughness: 0.17, metalness: 0.15,
      envMapIntensity: 1.1
    });
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.07, 18), mat);
    lens.position.set(x, y, z);
    lens.userData.partId = 'frame';
    g.add(lens);
    return { mesh: lens, mat };
  };
  const green = makeLens(0x5e9270, 0x1f442b, baseY + 0.19);
  const amber = makeLens(0xba8752, 0x3e2510, baseY + 0.265);
  const red = makeLens(0xa64943, 0x3b0d0b, baseY + 0.34);
  for (const y of [baseY + 0.225, baseY + 0.30, baseY + 0.375]) {
    const divider = new THREE.Mesh(new THREE.CylinderGeometry(0.062, 0.062, 0.014, 18), materialLib.blackMetal);
    divider.position.set(x, y, z);
    divider.userData.partId = 'frame';
    g.add(divider);
  }
  const cap = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.055, 0.03, 18), materialLib.blackMetal);
  cap.position.set(x, baseY + 0.395, z);
  cap.userData.partId = 'frame';
  g.add(cap);
  const haloTex = makeGlowTexture('#ff5c50');
  const halo = new THREE.Sprite(new THREE.SpriteMaterial({
    map: haloTex, transparent: true, opacity: 0, depthWrite: false,
    blending: THREE.AdditiveBlending, toneMapped: false
  }));
  halo.scale.set(0.6, 0.6, 1);
  halo.position.set(x, baseY + 0.34, z + 0.04);
  g.add(halo);
  groups.alarmBeacon = g;
  groups.beaconMats = { green: green.mat, amber: amber.mat, red: red.mat, halo };
  motorRoot.add(g);
}

// ===== 26. 五路传感器实体 =====
function buildSensors() {
  const sensors = new THREE.Group();
  sensors.name = 'sensors';
  groups.sensors = sensors;
  motorRoot.add(sensors);
  groups.sensorLeds = {};
  groups.sensorHalos = {};
  groups.sensorTags = {};
  const addLed = (parent, x, y, z, color) => {
    const mat = new THREE.MeshBasicMaterial({ color, toneMapped: false });
    const led = new THREE.Mesh(new THREE.SphereGeometry(0.014, 10, 8), mat);
    led.position.set(x, y, z);
    parent.add(led);
    return mat;
  };
  const addHalo = (id, x, y, z, hex) => {
    const sprite = makeGlowSprite(hex, 0.38, 0.0);
    sprite.position.set(x, y, z);
    sprite.material.opacity = 0;
    sensors.add(sprite);
    groups.sensorHalos[id] = sprite;
  };

  // --- 01 hall current clamp, on the outgoing cable ---
  {
    const g = new THREE.Group();
    g.name = 'sensor-hall';
    const anchor = new THREE.Vector3(0.982, 0.50, 0.278);
    const tangent = new THREE.Vector3(0.08, -0.86, 0.10).normalize();
    const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), tangent);
    g.position.copy(anchor);
    g.quaternion.copy(q);
    const ringMat = materialLib.sensorBody;
    for (const z of [-0.035, 0.035]) {
      const ring = new THREE.Mesh(new THREE.TorusGeometry(0.079, 0.024, 10, 28), ringMat);
      ring.position.z = z;
      ring.castShadow = true;
      g.add(ring);
    }
    const clampBody = new THREE.Mesh(roundedBoxGeo(0.15, 0.17, 0.12, 0.025), materialLib.sensorBody);
    clampBody.position.set(0.14, 0, 0);
    clampBody.rotation.z = -0.2;
    g.add(clampBody);
    const clampScrew = new THREE.Mesh(new THREE.CylinderGeometry(0.021, 0.021, 0.07, 6), materialLib.brass);
    clampScrew.rotation.z = Math.PI / 2;
    clampScrew.position.set(0.225, 0, 0);
    g.add(clampScrew);
    const ledMat = addLed(g, 0.17, 0.045, 0.066, 0x70d8e5);
    const tag = makeSensorTagSprite('CT-H01', '#70d8e5');
    tag.position.set(0.02, 0.30, 0.02);
    tag.visible = false;
    g.add(tag);
    sensors.add(g);
    registerPart('hall', g);
    groups.sensorLeds.hall = ledMat;
    groups.sensorTags.hall = tag;
    addHalo('hall', anchor.x, anchor.y, anchor.z, '#70d8e5');
  }

  // --- 02 incremental encoder on the output shaft ---
  {
    const g = new THREE.Group();
    g.name = 'sensor-encoder';
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.185, 0.185, 0.27, 32), materialLib.blackMetal);
    body.rotation.x = Math.PI / 2;
    body.position.z = 2.49;
    body.castShadow = true;
    g.add(body);
    const face = new THREE.Mesh(new THREE.CylinderGeometry(0.155, 0.155, 0.018, 32), materialLib.blackPolymer);
    face.rotation.x = Math.PI / 2;
    face.position.z = 2.633;
    g.add(face);
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.185, 0.018, 6, 32), materialLib.machinedEdge);
    ring.position.z = 2.59;
    g.add(ring);
    const cableGland = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.04, 0.11, 12), materialLib.steelDark);
    cableGland.rotation.z = Math.PI / 2;
    cableGland.position.set(-0.14, -0.14, 2.53);
    g.add(cableGland);
    // aluminum mounting bracket returns to the front end shield
    const arm = new THREE.Mesh(roundedBoxGeo(0.055, 0.055, 0.74, 0.02), materialLib.aluminum);
    arm.position.set(0.37, 0.36, 2.25);
    arm.rotation.y = -0.08;
    g.add(arm);
    const foot = new THREE.Mesh(roundedBoxGeo(0.16, 0.05, 0.2, 0.02), materialLib.aluminum);
    foot.position.set(0.37, 0.34, 1.86);
    g.add(foot);
    const armIn = new THREE.Mesh(roundedBoxGeo(0.27, 0.045, 0.06, 0.018), materialLib.aluminum);
    armIn.position.set(0.24, 0.36, 2.57);
    g.add(armIn);
    const ledMat = addLed(g, 0.11, 0.11, 2.64, 0x70d8e5);
    const tag = makeSensorTagSprite('ENC-02', '#70d8e5');
    tag.position.set(-0.04, 0.36, 2.58);
    tag.visible = false;
    g.add(tag);
    sensors.add(g);
    registerPart('encoder', g);
    groups.sensorLeds.encoder = ledMat;
    groups.sensorTags.encoder = tag;
    addHalo('encoder', 0.02, 0.06, 2.57, '#70d8e5');
  }

  // --- 03 PT100 surface RTD on the frame crown ---
  {
    const g = new THREE.Group();
    g.name = 'sensor-pt100';
    const angle = -0.405;
    const px = 0.34, py = Math.sqrt(Math.max(0.01, (MOTOR.bodyR + 0.016) ** 2 - px * px));
    g.position.set(px, py, -0.25);
    g.rotation.z = angle;
    const pad = new THREE.Mesh(roundedBoxGeo(0.19, 0.022, 0.13, 0.018), materialLib.steel);
    pad.castShadow = true;
    g.add(pad);
    const ceramic = new THREE.Mesh(roundedBoxGeo(0.13, 0.016, 0.085, 0.012), new THREE.MeshStandardMaterial({
      color: 0xcac4b5, metalness: 0.18, roughness: 0.62
    }));
    ceramic.position.y = 0.018;
    g.add(ceramic);
    const wire = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.23, 8), materialLib.rubber);
    wire.rotation.z = Math.PI / 2;
    wire.position.set(0.17, 0.012, 0.02);
    g.add(wire);
    const ledMat = addLed(g, 0.045, 0.028, 0.05, 0xffab5e);
    const tag = makeSensorTagSprite('RTD-03', '#ffab5e');
    tag.position.set(0, 0.24, 0.02);
    tag.visible = false;
    g.add(tag);
    sensors.add(g);
    registerPart('pt100', g);
    groups.sensorLeds.pt100 = ledMat;
    groups.sensorTags.pt100 = tag;
    addHalo('pt100', px, py, -0.25, '#ffab5e');
  }

  // --- 04 accelerometer on the drive end shield ---
  {
    const g = new THREE.Group();
    g.name = 'sensor-vib';
    const base = new THREE.Mesh(new THREE.CylinderGeometry(0.065, 0.075, 0.032, 22), materialLib.steelDark);
    base.rotation.x = Math.PI / 2;
    base.position.set(0.36, 0.49, 1.262);
    g.add(base);
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.052, 0.06, 0.115, 18), materialLib.brass);
    body.rotation.x = Math.PI / 2;
    body.position.set(0.36, 0.49, 1.34);
    body.castShadow = true;
    g.add(body);
    const top = new THREE.Mesh(new THREE.CylinderGeometry(0.048, 0.052, 0.035, 18), materialLib.sensorBody);
    top.rotation.x = Math.PI / 2;
    top.position.set(0.36, 0.49, 1.414);
    g.add(top);
    const cable = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.16, 8), materialLib.rubber);
    cable.rotation.x = Math.PI / 2;
    cable.rotation.z = 0.55;
    cable.position.set(0.31, 0.53, 1.41);
    g.add(cable);
    const ledMat = addLed(g, 0.39, 0.54, 1.40, 0xffab5e);
    const tag = makeSensorTagSprite('VIB-04', '#ffab5e');
    tag.position.set(0.36, 0.72, 1.40);
    tag.visible = false;
    g.add(tag);
    sensors.add(g);
    registerPart('vib', g);
    groups.sensorLeds.vib = ledMat;
    groups.sensorTags.vib = tag;
    addHalo('vib', 0.36, 0.54, 1.38, '#ffab5e');
  }

  // --- 05 through-beam photo switch on the code disc ---
  {
    const g = new THREE.Group();
    g.name = 'sensor-photo';
    const prong1 = new THREE.Mesh(roundedBoxGeo(0.16, 0.038, 0.035, 0.012), materialLib.blackPolymer);
    prong1.position.set(0.02, 0.275, 2.055);
    g.add(prong1);
    const prong2 = prong1.clone();
    prong2.position.z = 2.145;
    g.add(prong2);
    const bridge = new THREE.Mesh(roundedBoxGeo(0.04, 0.19, 0.13, 0.014), materialLib.blackPolymer);
    bridge.position.set(0.105, 0.27, 2.1);
    g.add(bridge);
    const mountArm = new THREE.Mesh(new THREE.CylinderGeometry(0.013, 0.013, 0.48, 10), materialLib.aluminum);
    mountArm.rotation.z = 1.02;
    mountArm.rotation.x = 0.16;
    mountArm.position.set(0.29, 0.37, 1.89);
    g.add(mountArm);
    const ledMat = addLed(g, 0.005, 0.285, 2.155, 0xff574d);
    const tag = makeSensorTagSprite('PH-05', '#e45a52');
    tag.position.set(0.04, 0.45, 2.10);
    tag.visible = false;
    g.add(tag);
    sensors.add(g);
    registerPart('photo', g);
    groups.sensorLeds.photo = ledMat;
    groups.sensorTags.photo = tag;
    addHalo('photo', 0.02, 0.28, 2.10, '#e45a52');
  }

  // --- signal highlight ghosts (one per measured part) ---
  const hg = new THREE.Group();
  groups.sensorHighlights = hg;
  motorRoot.add(hg);
  const addHighlight = (id, obj) => {
    const mat = new THREE.MeshBasicMaterial({
      color: 0x6fdfec, transparent: true, opacity: 0.0,
      depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide, toneMapped: false
    });
    obj.traverse((o) => { if (o.isMesh) o.material = mat; });
    obj.visible = false;
    hg.add(obj);
    highlightObjects.set(id, { object: obj, material: mat });
  };
  const windHl = new THREE.Group();
  for (const z of [-0.79, 0.79]) {
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.447, 0.025, 8, 64, MOTOR.cutLength), materialLib.highlightCyan);
    ring.geometry.rotateZ(MOTOR.cutStart);
    ring.position.z = z;
    windHl.add(ring);
  }
  addHighlight('windings', windHl);
  const rotorHl = new THREE.Group();
  const rotorRing = new THREE.Mesh(new THREE.TorusGeometry(0.402, 0.008, 6, 64), materialLib.highlightCyan);
  rotorHl.add(rotorRing);
  const rotorOutline = new THREE.Mesh(new THREE.CylinderGeometry(0.405, 0.405, 1.5, 20, 1, true), materialLib.highlightCyan);
  rotorOutline.rotation.x = Math.PI / 2;
  rotorHl.add(rotorOutline);
  addHighlight('rotor', rotorHl);
  const coreHl = new THREE.Group();
  for (const z of [-0.40, 0.40]) {
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.667, 0.006, 5, 64, MOTOR.cutLength), materialLib.highlightCyan);
    ring.geometry.rotateZ(MOTOR.cutStart);
    ring.position.z = z;
    coreHl.add(ring);
  }
  addHighlight('statorCore', coreHl);
  const bearingHl = new THREE.Group();
  for (const z of [-1.015, 1.015]) {
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.255, 0.012, 6, 48), materialLib.highlightCyan);
    ring.position.z = z;
    bearingHl.add(ring);
  }
  addHighlight('bearingFront', bearingHl);
  const shaftHl = new THREE.Group();
  const shaftOutline = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 1.1, 24, 1, true), materialLib.highlightCyan);
  shaftOutline.rotation.x = Math.PI / 2;
  shaftOutline.position.z = 1.7;
  shaftHl.add(shaftOutline);
  const discRing = new THREE.Mesh(new THREE.TorusGeometry(0.30, 0.006, 5, 48), materialLib.highlightCyan);
  discRing.position.z = 2.10;
  shaftHl.add(discRing);
  addHighlight('shaft', shaftHl);

  // selection signal paths (drawn on top, only selected sensor)
  groups.signalPaths = {};
  for (const sensor of SENSORS) {
    const group = new THREE.Group();
    const targetPoint = sensorTargetPoint(sensor.id);
    const source = new THREE.Vector3(...sensor.anchor);
    const curve = new THREE.CatmullRomCurve3([
      source,
      source.clone().lerp(targetPoint, 0.42).add(new THREE.Vector3(0.10, 0.16, 0.05)),
      targetPoint
    ]);
    const geo = new THREE.BufferGeometry().setFromPoints(curve.getPoints(36));
    const line = new THREE.Line(geo, new THREE.LineDashedMaterial({
      color: new THREE.Color(sensor.color), dashSize: 0.055, gapSize: 0.032,
      transparent: true, opacity: 0.65, depthTest: false, toneMapped: false
    }));
    line.computeLineDistances();
    line.renderOrder = 30;
    line.visible = false;
    group.add(line);
    const pulse = makeGlowSprite(sensor.color, 0.135, 0.0);
    pulse.material.depthTest = false;
    pulse.renderOrder = 31;
    group.add(pulse);
    group.userData = { curve, sensor, pulse, line };
    group.visible = false;
    motorRoot.add(group);
    groups.signalPaths[sensor.id] = group;
  }
}

function sensorTargetPoint(id) {
  switch (id) {
    case 'hall': return new THREE.Vector3(0.16, 0.35, 0.55);
    case 'encoder': return new THREE.Vector3(0.0, 0.0, 1.25);
    case 'pt100': return new THREE.Vector3(0.25, 0.42, -0.15);
    case 'vib': return new THREE.Vector3(0.0, 0.0, 1.02);
    case 'photo': return new THREE.Vector3(0.0, 0.0, 2.10);
    default: return new THREE.Vector3();
  }
}

// ===== 27. DAQ / 线缆 =====
function buildDaqAndCables() {
  const cableRoot = new THREE.Group();
  cableRoot.name = 'motor-cables';
  groups.cableRoot = cableRoot;
  motorRoot.add(cableRoot);

  const cableMat = new THREE.MeshStandardMaterial({
    color: 0x101317, metalness: 0.12, roughness: 0.72, envMapIntensity: 0.35
  });
  const cableMat2 = new THREE.MeshStandardMaterial({
    color: 0x171d23, metalness: 0.1, roughness: 0.68, envMapIntensity: 0.4
  });
  const tube = (points, radius, mat, seg = 64) => {
    const curve = new THREE.CatmullRomCurve3(points.map((p) => new THREE.Vector3(...p)));
    const m = new THREE.Mesh(new THREE.TubeGeometry(curve, seg, radius, 6, false), mat);
    m.castShadow = true;
    cableRoot.add(m);
    return { mesh: m, curve };
  };

  // Main feed cable — deliberately routed through the hall clamp.
  const mainPath = [
    [0.30, 1.06, 0.16],
    [0.54, 1.00, 0.22],
    [0.81, 0.80, 0.23],
    [0.958, 0.62, 0.26],
    [0.982, 0.50, 0.278],
    [1.00, 0.20, 0.33],
    [1.10, -0.35, 0.46],
    [1.18, -0.92, 0.58],
    [1.36, -1.03, 0.68]
  ];
  tube(mainPath, 0.034, cableMat, 80);
  // Cable glands at terminal
  for (const x of [-0.22, 0.22]) {
    const gland = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.055, 0.11, 18), materialLib.steelDark);
    gland.rotation.z = Math.PI / 2;
    gland.position.set(x, MOTOR.bodyR + 0.11, -0.32);
    cableRoot.add(gland);
  }
  // Thin sensor harness (kept close to the body).
  const harness = [
    [0.32, 0.93, -0.35],
    [0.82, 0.72, -0.30],
    [1.02, 0.15, 0.10],
    [1.28, -0.72, 0.45],
    [1.40, -1.00, 0.65]
  ];
  tube(harness, 0.018, cableMat2, 64);
  for (let i = 0; i < 5; i++) {
    const clamp = new THREE.Mesh(new THREE.TorusGeometry(0.025, 0.006, 5, 12), materialLib.steelDark);
    clamp.position.set(0.88 + i * 0.1, 0.46 - i * 0.30, 0.18 + i * 0.06);
    clamp.rotation.x = Math.PI / 2;
    cableRoot.add(clamp);
  }
  // local cables from each sensor toward the harness
  tube([[0.36, 0.79, -0.24], [0.50, 0.86, -0.40], [0.74, 0.79, -0.39], [0.84, 0.73, -0.32]], 0.011, cableMat2, 30);
  tube([[0.36, 0.54, 1.42], [0.62, 0.66, 1.54], [0.92, 0.43, 1.31], [1.0, 0.14, 0.13]], 0.011, cableMat2, 40);
  tube([[-0.14, -0.14, 2.53], [0.55, -0.02, 2.58], [0.95, -0.55, 1.5], [1.35, -0.98, 0.72]], 0.012, cableMat2, 46);
  tube([[0.06, 0.25, 2.18], [0.48, 0.02, 2.35], [0.91, -0.65, 1.35], [1.35, -0.99, 0.7]], 0.010, cableMat2, 44);
  tube([[0.20, 0.08, 0.10], [0.96, -0.10, 0.12], [1.18, -0.78, 0.46], [1.4, -1.0, 0.67]], 0.012, cableMat2, 40);

  // DAQ enclosure
  const daq = new THREE.Group();
  daq.name = 'daq-unit';
  const body = new THREE.Mesh(roundedBoxGeo(0.55, 0.22, 0.38, 0.035), materialLib.blackMetal);
  body.position.set(1.53, -0.99, 0.72);
  body.castShadow = true;
  daq.add(body);
  const top = new THREE.Mesh(roundedBoxGeo(0.48, 0.015, 0.30, 0.018), materialLib.glassBlack);
  top.position.set(1.53, -0.875, 0.71);
  daq.add(top);
  const screenTex = makeCanvasTexture(512, 300, (ctx, w, h) => {
    ctx.fillStyle = '#06100f'; ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = 'rgba(112,216,229,.35)'; ctx.lineWidth = 3; ctx.strokeRect(12, 12, w - 24, h - 24);
    ctx.fillStyle = '#79e0e9';
    ctx.font = '600 58px "Cascadia Code","Consolas",monospace';
    ctx.fillText('DAQ-01', 30, 85);
    ctx.fillStyle = '#9cb8b9';
    ctx.font = '500 25px "Cascadia Code","Consolas",monospace';
    ctx.fillText('5 CH  ·  50 Hz  ·  100 pt', 30, 138);
    ctx.fillStyle = '#5a8f87';
    ctx.fillText('CURRENT / SPEED / TEMP', 30, 182);
    ctx.fillText('VIB / PHOTO', 30, 218);
    ctx.fillStyle = '#70d8e5';
    for (let i = 0; i < 12; i++) {
      const h2 = 10 + Math.abs(Math.sin(i * 0.9)) * 28;
      ctx.fillRect(30 + i * 21, 266 - h2, 8, h2);
    }
    ctx.fillStyle = '#83d3a4';
    ctx.beginPath(); ctx.arc(w - 35, 35, 8, 0, TAU); ctx.fill();
  });
  const screen = new THREE.Mesh(new THREE.PlaneGeometry(0.45, 0.26), new THREE.MeshBasicMaterial({ map: screenTex, transparent: true, toneMapped: false }));
  screen.rotation.x = -Math.PI / 2;
  screen.rotation.z = 0.02;
  screen.position.set(1.53, -0.866, 0.71);
  daq.add(screen);
  const ledMat = new THREE.MeshBasicMaterial({ color: 0x83d3a4, toneMapped: false });
  const led = new THREE.Mesh(new THREE.SphereGeometry(0.022, 12, 8), ledMat);
  led.position.set(1.34, -0.94, 0.54);
  daq.add(led);
  const ports = new THREE.Group();
  for (let i = 0; i < 5; i++) {
    const port = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.027, 0.045, 16), materialLib.brass);
    port.rotation.z = Math.PI / 2;
    port.position.set(1.30, -0.96, 0.58 + i * 0.05);
    ports.add(port);
  }
  daq.add(ports);
  const daqLabel = makeSensorTagSprite('DAQ-01 / 50 Hz', '#70d8e5');
  daqLabel.scale.multiplyScalar(0.22);
  daqLabel.material.opacity = 0.45;
  daqLabel.position.set(1.53, -0.80, 0.72);
  daqLabel.visible = false;
  daq.add(daqLabel);
  groups.daqLabel = daqLabel;
  daq.traverse((o) => { if (o.isMesh) o.userData.partId = 'daq'; });
  motorRoot.add(daq);
  groups.daq = daq;
  registerPart('daq', daq);
}

// ===== 28. 卡滞楔块 / 断条 / 磨损标记 =====
function buildJamAndFaultMarkers() {
  const jam = new THREE.Group();
  jam.name = 'jam-wedge';
  const metal = new THREE.MeshStandardMaterial({ color: 0x50545a, metalness: 0.82, roughness: 0.35 });
  const red = new THREE.MeshStandardMaterial({ color: 0xa43932, metalness: 0.2, roughness: 0.5, emissive: 0x330805, emissiveIntensity: 0.4 });
  const clampRing = new THREE.Mesh(new THREE.TorusGeometry(0.18, 0.045, 8, 32), metal);
  clampRing.position.z = 1.57;
  jam.add(clampRing);
  const wedge = new THREE.Mesh(new THREE.BoxGeometry(0.38, 0.16, 0.26), red);
  wedge.position.set(0.27, -0.17, 1.57);
  wedge.rotation.z = -0.34;
  jam.add(wedge);
  const handle = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.30, 0.06), metal);
  handle.position.set(0.48, 0.03, 1.57);
  handle.rotation.z = 0.25;
  jam.add(handle);
  const warning = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.09, 0.008), new THREE.MeshBasicMaterial({
    map: makeLabelTexture('JAM', { color: '#ffd0c9', size: 52, bg: 'rgba(140,30,24,.92)', border: 'rgba(255,115,104,.8)' }),
    transparent: true, toneMapped: false
  }));
  warning.position.set(0.27, -0.17, 1.708);
  jam.add(warning);
  const glow = makeGlowSprite('#ff5b4f', 0.55, 0.0);
  glow.position.set(0.28, -0.15, 1.66);
  jam.add(glow);
  jam.userData.glow = glow;
  jam.visible = false;
  motorRoot.add(jam);
  groups.jam = jam;

  const broken = new THREE.Group();
  broken.name = 'broken-bars';
  const brokenMat = new THREE.MeshStandardMaterial({ color: 0x241612, metalness: 0.6, roughness: 0.5 });
  const crackMat = new THREE.MeshBasicMaterial({ color: 0xff6656, transparent: true, opacity: 0.85, toneMapped: false });
  for (const barIndex of [3, 14]) {
    const a = barIndex / 22 * TAU;
    const holder = new THREE.Group();
    holder.rotation.z = a;
    const coverBar = new THREE.Mesh(new THREE.BoxGeometry(0.046, 0.054, 0.29), brokenMat);
    coverBar.position.set(0.37, 0, -0.13);
    holder.add(coverBar);
    const crack = new THREE.Mesh(new THREE.BoxGeometry(0.052, 0.06, 0.012), crackMat);
    crack.position.set(0.37, 0, 0.02);
    holder.add(crack);
    holder.userData.userDataPart = 'rotorBars';
    broken.add(holder);
  }
  broken.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.userData.partId = 'rotorBars'; } });
  broken.visible = false;
  groups.rotorSpin.add(broken);
  groups.brokenBarMarkers = broken;

  const bearingMarks = new THREE.Group();
  bearingMarks.name = 'bearing-wear-markers';
  const markMat = new THREE.MeshBasicMaterial({ color: 0xffb267, transparent: true, opacity: 0.9, toneMapped: false });
  for (const idx of [1, 4, 7]) {
    const a = idx / 9 * TAU;
    const spot = new THREE.Mesh(new THREE.SphereGeometry(0.018, 10, 8), markMat);
    spot.position.set(Math.cos(a) * 0.188, Math.sin(a) * 0.188, 1.015);
    bearingMarks.add(spot);
  }
  bearingMarks.traverse((o) => { if (o.isMesh) o.userData.partId = 'bearingFront'; });
  bearingMarks.visible = false;
  groups.rotorSpin.add(bearingMarks);
  groups.bearingMarkers = bearingMarks;
}

// ===== 29. 四极旋转磁场 / 感应电流箭头 =====
function buildFieldVisuals() {
  const field = new THREE.Group();
  field.name = 'rotating-field';
  groups.field = field;
  groups.fieldMaterials = [];
  groups.fieldSprites = [];
  groups.fieldPoles = new THREE.Group();
  field.add(groups.fieldPoles);
  motorRoot.add(field);

  const poleColors = [
    { hex: 0xff9b93, name: 'N', opacity: 0.54 },
    { hex: 0x70d8e5, name: 'S', opacity: 0.52 },
    { hex: 0xff9b93, name: 'N', opacity: 0.54 },
    { hex: 0x70d8e5, name: 'S', opacity: 0.52 }
  ];
  for (let i = 0; i < 4; i++) {
    const info = poleColors[i];
    const center = i * Math.PI / 2;
    const mat = new THREE.MeshBasicMaterial({
      color: info.hex, transparent: true, opacity: 0,
      blending: THREE.AdditiveBlending, depthWrite: false,
      side: THREE.DoubleSide, toneMapped: false
    });
    const arc = new THREE.Mesh(new THREE.TorusGeometry(0.425, 0.031, 6, 28, 1.17), mat);
    arc.geometry.rotateZ(center - 0.585);
    arc.position.z = 0.88;
    groups.fieldPoles.add(arc);
    const innerArc = new THREE.Mesh(new THREE.TorusGeometry(0.412, 0.010, 4, 24, 0.9), mat.clone());
    innerArc.geometry.rotateZ(center - 0.45);
    innerArc.position.z = 0.91;
    groups.fieldPoles.add(innerArc);
    const glow = makeGlowSprite(`#${info.hex.toString(16).padStart(6, '0')}`, 0.27, 0.0);
    glow.position.set(Math.cos(center) * 0.422, Math.sin(center) * 0.422, 0.92);
    groups.fieldPoles.add(glow);
    const labelTex = makeLabelTexture(info.name, {
      color: info.name === 'N' ? '#ffb2ac' : '#94f3ff', size: 64,
      bg: 'rgba(7,12,17,.76)', border: info.name === 'N' ? 'rgba(255,155,147,.54)' : 'rgba(112,216,229,.54)'
    });
    const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: labelTex, transparent: true, depthWrite: false, opacity: 0.0, toneMapped: false }));
    label.scale.set(0.12, 0.12, 1);
    label.position.set(Math.cos(center) * 0.47, Math.sin(center) * 0.47, 0.95);
    groups.fieldPoles.add(label);
    // broad luminous pole bands make the four-pole field legible in the bore
    const bandMat = new THREE.MeshBasicMaterial({
      color: info.hex, transparent: true, opacity: 0,
      blending: THREE.NormalBlending, depthWrite: false, side: THREE.DoubleSide, toneMapped: false
    });
    const band = new THREE.Mesh(new THREE.RingGeometry(0.31, 0.468, 40, 1, center - 0.54, 1.08), bandMat);
    band.position.z = 0.875;
    groups.fieldPoles.add(band);
    groups.fieldBands = groups.fieldBands || [];
    groups.fieldBands.push(bandMat);
    groups.fieldMaterials.push(mat, innerArc.material);
    groups.fieldSprites.push(glow, label);
  }
  // field phase ticks / flux lines
  const linePts = [];
  for (let i = 0; i < 12; i++) {
    const a = i / 12 * TAU;
    linePts.push(new THREE.Vector3(Math.cos(a) * 0.30, Math.sin(a) * 0.30, 0.90));
    linePts.push(new THREE.Vector3(Math.cos(a) * 0.40, Math.sin(a) * 0.40, 0.90));
  }
  const fieldLineGeo = new THREE.BufferGeometry().setFromPoints(linePts);
  const fieldLineMat = new THREE.LineBasicMaterial({
    color: 0x8de8f2, transparent: true, opacity: 0.0,
    blending: THREE.AdditiveBlending, depthWrite: false, toneMapped: false
  });
  const lines = new THREE.LineSegments(fieldLineGeo, fieldLineMat);
  groups.fieldPoles.add(lines);
  groups.fieldMaterials.push(fieldLineMat);
  field.visible = false;

  // Induced-current arrows attached to the rotor bars.
  const arrows = new THREE.Group();
  arrows.name = 'bar-current-arrows';
  groups.currentArrows = arrows;
  const arrowGeo = new THREE.ConeGeometry(0.028, 0.105, 8);
  const arrowMat = new THREE.MeshBasicMaterial({
    color: 0xffc18c, transparent: true, opacity: 0.9,
    blending: THREE.AdditiveBlending, depthWrite: false, toneMapped: false
  });
  groups.currentArrows.userData.items = [];
  groups.currentArrows.userData.material = arrowMat;
  for (let i = 0; i < 12; i++) {
    const a = (i / 12) * TAU + 0.12;
    const holder = new THREE.Group();
    holder.rotation.z = a;
    const arrow = new THREE.Mesh(arrowGeo, arrowMat);
    arrow.rotation.x = Math.PI / 2;
    arrow.position.set(0.37, 0, 0.82);
    holder.add(arrow);
    arrows.add(holder);
    groups.currentArrows.userData.items.push({ holder, arrow, localAngle: a });
  }
  arrows.visible = false;
  groups.rotorSpin.add(arrows);
}

function updateFieldVisuals(dt) {
  if (!groups.field) return;
  const show = ui.fieldVisible && state.fieldAlpha > 0.01;
  groups.field.visible = show;
  groups.currentArrows.visible = show;
  const legend = D('fieldLegend');
  if (legend) {
    const legendShow = show && !ui.cinemaActive && ui.structure !== 'assembly';
    legend.classList.toggle('show', legendShow);
    if (legendShow) {
      setTxt('fieldLegendText', `n₁ 1500 · n ${state.nRpm.toFixed(0)} · s ${state.slip.toFixed(3)}`);
    }
  }
  if (!show) return;
  groups.field.rotation.z = state.displayFieldAngle;
  const speedPulse = 0.90 + 0.10 * Math.sin(state.simTime * 3.2);
  groups.fieldMaterials.forEach((mat, i) => {
    const base = i === 0 || i === 3 ? 1.0 : 0.82;
    mat.opacity = base * state.fieldAlpha * speedPulse;
  });
  if (groups.fieldBands) {
    groups.fieldBands.forEach((mat) => {
      mat.opacity = 0.26 * state.fieldAlpha * speedPulse;
    });
  }
  for (const s of groups.fieldSprites) {
    s.material.opacity = (s.isSprite ? 0.9 : 0.5) * state.fieldAlpha * speedPulse;
  }
  // Induced-current direction is set by the relative motion between field and bar.
  const mat = groups.currentArrows.userData.material;
  mat.opacity = 0.55 + 0.4 * state.fieldAlpha;
  const relSpeed = (state.fieldDisplayRpm - state.rotorDisplayRpm) / Math.max(1, Math.abs(state.fieldDisplayRpm));
  for (const item of groups.currentArrows.userData.items) {
    const worldBarAngle = item.localAngle + state.displayRotorAngle;
    const sign = Math.sin(worldBarAngle - state.displayFieldAngle + 0.4) * Math.sign(relSpeed || 1);
    item.arrow.rotation.x = sign >= 0 ? Math.PI / 2 : -Math.PI / 2;
    item.arrow.scale.setScalar(0.8 + 0.20 * Math.sin(state.simTime * 4.0 + item.localAngle * 2));
  }
}

// ===== 30. 强制气流示意 =====
function buildAirflow() {
  const count = 110;
  const rand = pseudoRandom(3109);
  const offsets = [];
  const positions = new Float32Array(count * 3);
  for (let i = 0; i < count; i++) {
    offsets.push({
      phase: rand(),
      angle: rand() * TAU,
      radius: 0.86 + rand() * 0.18,
      wobble: rand() * 2 - 1,
      speed: 0.55 + rand() * 0.5
    });
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  const mat = new THREE.PointsMaterial({
    color: 0x7edbe7, size: 0.026, transparent: true, opacity: 0.36,
    depthWrite: false, blending: THREE.AdditiveBlending, sizeAttenuation: true, toneMapped: false
  });
  const points = new THREE.Points(geo, mat);
  points.frustumCulled = false;
  const g = new THREE.Group();
  g.name = 'forced-airflow';
  g.add(points);
  g.visible = false;
  motorRoot.add(g);
  groups.airflow = g;
  groups.airflow.userData = { points, offsets, count, positions };
}

function updateAirflow(dt) {
  const data = groups.airflow?.userData;
  if (!data) return;
  const active = ui.airflow && state.nRpm > 15;
  groups.airflow.visible = ui.airflow;
  if (!ui.airflow) return;
  const t = performance.now() / 1000;
  const flowSpeed = (state.nRpm / MC.ratedSpeedRpm) * 0.075;
  data.offsets.forEach((o, i) => {
    const p = fract(o.phase + t * flowSpeed * o.speed);
    const z = lerp(-2.02, 2.22, p);
    const r = o.radius + 0.035 * Math.sin(p * 11 + o.wobble * 3);
    const a = o.angle + p * 1.1 + 0.12 * Math.sin(p * 8 + o.wobble);
    data.positions[i * 3] = Math.cos(a) * r;
    data.positions[i * 3 + 1] = Math.sin(a) * r * 0.68;
    data.positions[i * 3 + 2] = z;
  });
  data.points.geometry.attributes.position.needsUpdate = true;
  data.points.material.opacity = active ? 0.33 : 0.09;
}

// ===== 31. 工程标注锚点 =====
function buildAnnotations() {
  const mkAnchor = (parent, pos) => {
    const o = new THREE.Object3D();
    o.position.set(...pos);
    parent.add(o);
    return o;
  };
  annotationAnchors = {
    assembly: [
      {
        id: 'terminal', title: '接线盒 / TERMINAL', sub: '6 端子 · △ / Y 连片',
        anchor: mkAnchor(groups.terminal, [0.34, 0.23, 0.38]),
        offset: [78, -30], align: 'left'
      },
      {
        id: 'cowl', title: '风罩 / 径向风冷', sub: 'FAN COWL · IP44',
        anchor: mkAnchor(groups.fanCowl, [-0.38, 0.48, -1.52]),
        offset: [-88, 20], align: 'right'
      },
      {
        id: 'plate', title: '铭牌 / NAMEPLATE', sub: '380 V △ · 1.1 kW',
        anchor: mkAnchor(groups.nameplateGroup, [0.93, -0.055, 0.61]),
        offset: [78, 34], align: 'left'
      }
    ],
    section: [
      {
        id: 'stator', title: '定子铁芯 / 24 槽', sub: 'STATOR · 0.35 mm 气隙',
        anchor: mkAnchor(groups.stator, [-0.50, 0.35, 0.15]),
        offset: [-108, -22], align: 'right'
      },
      {
        id: 'winding', title: '三相端部绕组', sub: 'U / V / W · 铜',
        anchor: mkAnchor(groups.stator, [0.18, 0.36, 0.80]),
        offset: [98, -30], align: 'left'
      },
      {
        id: 'bar', title: '鼠笼导条 / 短路环', sub: '22 BARS · INDUCED CURRENT',
        anchor: mkAnchor(groups.rotorSpin, [0.28, 0.28, 0.16]),
        offset: [108, 36], align: 'left'
      },
      {
        id: 'bearing', title: '深沟球轴承 6205', sub: 'BEARING · GCr15',
        anchor: mkAnchor(groups.rotorSpin, [0.22, 0.08, 1.015]),
        offset: [100, -34], align: 'left'
      }
    ]
  };
  // exploded labels can reuse grouped anchors
  annotationAnchors.exploded = [
    {
      id: 'rotor', title: '转子组件 / ROTOR', sub: '转子抽出 · 鼠笼可见',
      anchor: mkAnchor(groups.rotorSpin, [0.26, 0.3, 0.1]),
      offset: [96, -26], align: 'left'
    },
    {
      id: 'stator2', title: '定子 / 机座切面', sub: 'STATOR CORE · 24 SLOTS',
      anchor: mkAnchor(groups.stator, [-0.55, 0.28, 0.1]),
      offset: [-100, -10], align: 'right'
    },
    {
      id: 'cap', title: '前端盖 / 轴承室', sub: 'DRIVE END SHIELD',
      anchor: mkAnchor(groups.frontCap, [0.34, 0.46, 1.28]),
      offset: [90, 32], align: 'left'
    },
    {
      id: 'fan2', title: '风罩 / 风扇', sub: 'COOLING ASSEMBLY',
      anchor: mkAnchor(groups.fanCowl, [-0.38, 0.42, -1.52]),
      offset: [-90, 28], align: 'right'
    }
  ];
}

// ===== 32. 更新所有可动视觉 =====
function updateMechanism(dtWall) {
  const dt = clamp(dtWall, 0, 0.05);
  const actualRpm = Math.max(0, state.nRpm);
  let nominalScale = 1;
  let rotorRate = actualRpm;
  let fieldRate = state.fieldOn ? SYNC_RPM : 0;

  if (ui.observation === 'real') {
    nominalScale = 1;
    rotorRate = actualRpm;
    fieldRate = state.fieldOn ? SYNC_RPM : 0;
  } else if (ui.observation === 'manual') {
    nominalScale = ui.manualScale;
    rotorRate = actualRpm * nominalScale;
    fieldRate = state.fieldOn ? SYNC_RPM * nominalScale : 0;
  } else {
    const targetDisplay = ui.observation === 'ultra' ? DISPLAY_ULTRA_RPM : DISPLAY_AUTO_RPM;
    // At or above the target, the rotor turns at a readable 30 / 12 r/min.
    // Below the target it rotates at its true scaled speed (down to stop).
    rotorRate = Math.min(actualRpm, targetDisplay);
    // The field keeps the synchronous display rate even when the rotor stalls.
    // This is the crucial visible slip. At rated speed it equals 30 × 1500/1400.
    fieldRate = state.fieldOn ? targetDisplay * (SYNC_RPM / MC.ratedSpeedRpm) : 0;
    nominalScale = targetDisplay / MC.ratedSpeedRpm;
  }

  state.displayScale = nominalScale;
  state.rotorDisplayRpm = ui.paused ? 0 : rotorRate;
  state.fieldDisplayRpm = ui.paused ? 0 : fieldRate;

  if (!ui.paused) {
    state.displayRotorAngle = wrapAngle(state.displayRotorAngle + rotorRate * RPM2RAD * dt);
    if (state.fieldOn) {
      state.displayFieldAngle = wrapAngle(state.displayFieldAngle + fieldRate * RPM2RAD * dt);
    }
  }

  const rotorAngle = state.displayRotorAngle;
  if (groups.rotorSpin) groups.rotorSpin.rotation.z = rotorAngle;
  if (groups.fan) groups.fan.rotation.z = rotorAngle;
  if (groups.bearingCages) {
    for (const cage of groups.bearingCages) cage.rotation.z = rotorAngle * 0.38;
  }
  updateFieldVisuals(dt);
  updateAirflow(dt);
  updateJamVisual(dt);
  updateFaultMarkerVisuals(dt);
  updateFan();
  updateSensorLedVisuals(dt);
  updateTerminalLid(dt);
  updateExplosion(dt);
  updateAnnotations();
}

function updateFan() {
  if (!groups.fanCowl) return;
  // The cooling fan uses the same angle as the rotor by construction.
  const active = state.fieldOn || state.nRpm > 1;
  const speedFactor = clamp(state.nRpm / MC.ratedSpeedRpm, 0, 1.3);
  if (groups.airflow) {
    groups.airflow.userData.points.material.opacity = ui.airflow ? (0.08 + speedFactor * 0.25) : 0;
  }
  // very subtle vibration amplitude for the guard is intentionally avoided
}

function updateJamVisual(dt) {
  if (!groups.jam) return;
  const show = state.fault === 'stall';
  if (show && !groups.jam.visible) {
    groups.jam.visible = true;
    groups.jam.scale.set(0.2, 0.2, 0.2);
  }
  if (!show && groups.jam.visible) {
    groups.jam.scale.multiplyScalar(1 - Math.min(1, dt * 12));
    if (groups.jam.scale.x < 0.05) groups.jam.visible = false;
  }
  if (groups.jam.visible) {
    const target = show ? 1 : 0;
    groups.jam.scale.x = damp(groups.jam.scale.x, target, 12, dt);
    groups.jam.scale.y = groups.jam.scale.x;
    groups.jam.scale.z = groups.jam.scale.x;
    const glow = groups.jam.userData.glow;
    if (glow) glow.material.opacity = (state.alarm ? 0.55 + 0.35 * Math.sin(state.simTime * 7) : 0.24) * groups.jam.scale.x;
  }
}

function updateFaultMarkerVisuals(dt) {
  if (groups.brokenBarMarkers) {
    groups.brokenBarMarkers.visible = state.fault === 'broken';
    if (groups.brokenBarMarkers.visible) {
      const p = 0.7 + 0.3 * Math.sin(state.simTime * 5);
      groups.brokenBarMarkers.traverse((o) => {
        if (o.isMesh && o.material && o.material.emissive) o.material.emissiveIntensity = p;
      });
    }
  }
  if (groups.bearingMarkers) {
    groups.bearingMarkers.visible = state.fault === 'bearing';
  }
  // Overheated winding material: copper heats toward a dull red, no global tint.
  const heat = state.fault === 'overheat'
    ? clamp((state.tempC - MC.steadyCaseC) / 25, 0, 1.4)
    : 0;
  for (const mat of groups.windingMats) {
    mat.emissive.setRGB(0.35 * heat + 0.01, 0.025 * heat, 0.005 * heat);
    mat.emissiveIntensity = heat > 0.02 ? 0.35 + heat * 1.2 : 0;
  }
  // machine body gets a very restrained thermal tint via alarm light only
}

function updateSensorLedVisuals(dt) {
  if (!groups.sensorLeds) return;
  for (const s of SENSORS) {
    const led = groups.sensorLeds[s.id];
    if (!led) continue;
    const reading = getSensorReading(s.id);
    let color;
    if (reading.status === 'alarm') color = new THREE.Color(0xff4e45);
    else if (reading.status === 'warn') color = new THREE.Color(0xffab5e);
    else color = new THREE.Color(s.kind === 'temp' || s.kind === 'vibration' ? 0x9bd4c0 : 0x70d8e5);
    const pulse = 0.75 + 0.25 * Math.sin(state.simTime * 4 + SENSORS.indexOf(s));
    led.color.copy(color).multiplyScalar(pulse);
    if (led.emissive) led.emissive.copy(color);
  }
}

function updateTerminalLid(dt) {
  if (!groups.terminalLid) return;
  const base = groups.terminalLid.userData.basePosition;
  const targetY = base.y + (ui.terminalOpen ? 0.35 : 0);
  const targetZ = base.z + (ui.terminalOpen ? 0.10 : 0);
  groups.terminalLid.position.y = damp(groups.terminalLid.position.y, targetY, 9, dt);
  groups.terminalLid.position.z = damp(groups.terminalLid.position.z, targetZ, 9, dt);
  const openAmount = clamp((groups.terminalLid.position.y - base.y) / 0.35, 0, 1);
  if (groups.terminalPosts) {
    for (const p of groups.terminalPosts) p.labelSprite.visible = openAmount > 0.25 && ui.selectedPart === 'terminal';
  }
  // Straps interpolate between △ and Y geometry.
  if (groups.terminalStraps) {
    for (const strap of groups.terminalStraps) {
      const target = ui.connection === 'delta' ? strap.userData.delta : strap.userData.wye;
      strap.position.lerp(target.pos, 1 - Math.exp(-10 * dt));
      strap.quaternion.slerp(target.quat, 1 - Math.exp(-10 * dt));
      strap.scale.lerp(target.scale, 1 - Math.exp(-10 * dt));
    }
  }
}

// ===== 33. 结构视图 / 爆炸图 =====
function setStructure(mode, source = 'ui') {
  if (!['assembly', 'section', 'exploded'].includes(mode)) return;
  const previous = ui.structure;
  if (previous === mode && !(mode === 'exploded' && ui.explodeTarget !== 1)) {
    // still update camera chip UI and visibility
  }
  const wasExploded = previous === 'exploded';
  const willExplode = mode === 'exploded';
  if (willExplode && !wasExploded) {
    ui.preExplodePaused = ui.paused;
    setPauseMechanism(true, { silent: true });
  } else if (!willExplode && wasExploded) {
    setPauseMechanism(ui.preExplodePaused, { silent: true });
  }
  ui.structure = mode;
  if (mode === 'exploded') {
    ui.explodeTarget = 1;
    ui.cameraPreset = null;
  } else {
    ui.explodeTarget = 0;
  }
  if (!groups.frameFull) return;
  groups.frameFull.visible = mode === 'assembly';
  groups.frameCut.visible = mode !== 'assembly';
  groups.frontCap.visible = mode !== 'section';
  groups.nameplateGroup.visible = mode === 'assembly';
  if (mode !== 'assembly') {
    // The terminal box and cables remain in context; the front bearing view is open.
  }
  if (mode === 'section') {
    if (ui.cinemaActive) { /* cinema has its own sequencing */ }
    setActiveCameraChip('section');
  } else if (mode === 'assembly') {
    setActiveCameraChip(source === 'camera' ? ui.cameraPreset : 'hero');
  } else if (mode === 'exploded') {
    setActiveCameraChip('rotor');
  }
  // If switching from section to exploded, re-show the front cover.
  if (ui.selectedSensor) {
    refreshSensorSheet();
  }
  syncStructureTabs();
  if (source !== 'init' && source !== 'cinema') {
    showToast(
      mode === 'assembly' ? '外观视图 · 完整机座与端盖' :
      mode === 'section' ? '机械切面 · 定子 / 气隙 / 鼠笼可见' :
      '分层爆炸 · 机构展示已暂停',
      'info'
    );
  }
}

function syncStructureTabs() {
  $$('.view-tab').forEach((b) => b.classList.toggle('active', b.dataset.structure === ui.structure));
}

function setPauseMechanism(paused, opts = {}) {
  ui.paused = !!paused;
  const btn = $('#pauseBtn');
  if (btn) {
    btn.classList.toggle('primary', ui.paused);
    btn.setAttribute('aria-pressed', String(ui.paused));
    const label = btn.querySelector('span:last-child');
    if (label) label.textContent = ui.paused ? '继续机构' : '暂停机构';
    const icon = btn.querySelector('.ctrl-ico');
    if (icon) icon.textContent = ui.paused ? '▶' : 'Ⅱ';
  }
  if (!opts.silent && !ui.cinemaActive) {
    showToast(ui.paused ? '机构展示已暂停 · 工况模型仍持续采样' : '机构展示继续 · 工况且与展示相位保持同源', 'info');
  }
}

function updateExplosion(dt) {
  const target = ui.explodeTarget;
  const prev = ui.explodeAmount;
  ui.explodeAmount = damp(ui.explodeAmount, target, target > ui.explodeAmount ? 3.2 : 3.7, dt);
  if (Math.abs(ui.explodeAmount - target) < 0.0006) ui.explodeAmount = target;
  if (Math.abs(prev - ui.explodeAmount) < 1e-5) return;
  for (const [, item] of animationGroups) {
    item.group.position.copy(item.base).addScaledVector(item.offset, ui.explodeAmount);
  }
  // Fan / cowl / caps have their own visibility interplay.
  if (groups.frontCap) groups.frontCap.visible = ui.structure !== 'section';
}

// ===== 34. 工程标注 DOM =====
let annotationEls = new Map();
function ensureAnnotationDom() {
  if (!annotationEls.size) {
    const layer = $('#annotationLayer');
    const container = document.createElement('div');
    container.className = 'annotation-labels';
    container.style.cssText = 'position:absolute;inset:0;pointer-events:none';
    layer.appendChild(container);
    const ns = 'http://www.w3.org/2000/svg';
    const all = [...annotationAnchors.assembly, ...annotationAnchors.section, ...annotationAnchors.exploded];
    const seen = new Set();
    for (const a of all) {
      if (seen.has(a.id)) continue;
      seen.add(a.id);
      const label = document.createElement('div');
      label.className = 'annotation-label';
      label.dataset.annotation = a.id;
      label.innerHTML = `<b>${a.title}</b>${a.sub} <i>/ 01</i>`;
      container.appendChild(label);
      const path = document.createElementNS(ns, 'path');
      path.setAttribute('data-annotation-path', a.id);
      $('#annotationLines').appendChild(path);
      annotationEls.set(a.id, { label, path, data: a });
    }
  }
}

function updateAnnotations() {
  if (!ui.annotations || (ui.fieldVisible && ui.structure !== 'assembly') || !annotationAnchors.assembly) {
    if (annotationEls.size) {
      for (const { label, path } of annotationEls.values()) {
        label.classList.remove('show');
        path.style.opacity = '0';
      }
    }
    return;
  }
  ensureAnnotationDom();
  const layer = $('#annotationLayer');
  if (!layer) return;
  const rect = layer.getBoundingClientRect();
  const svg = layer.querySelector('svg');
  if (svg) {
    svg.setAttribute('viewBox', `0 0 ${rect.width} ${rect.height}`);
    svg.setAttribute('preserveAspectRatio', 'none');
  }
  const mode = ui.structure === 'exploded' ? 'exploded' : ui.structure === 'section' ? 'section' : 'assembly';
  const visibleConfig = annotationAnchors[mode] || [];
  const visibleIds = new Set(visibleConfig.map((a) => a.id));
  const blocked = document.body.classList.contains('focus-mode') || ui.cinemaActive;
  for (const [id, el] of annotationEls) {
    if (!visibleIds.has(id) || blocked) {
      el.label.classList.remove('show');
      el.path.style.opacity = '0';
      continue;
    }
    const data = visibleConfig.find((a) => a.id === id);
    const world = data.anchor.getWorldPosition(new THREE.Vector3());
    const projected = world.clone().project(camera);
    const hidden = projected.z > 1 || projected.z < -1 ||
      Math.abs(projected.x) > 1.08 || Math.abs(projected.y) > 1.06;
    if (hidden) {
      el.label.classList.remove('show');
      el.path.style.opacity = '0';
      continue;
    }
    const ax = (projected.x * 0.5 + 0.5) * rect.width;
    const ay = (-projected.y * 0.5 + 0.5) * rect.height;
    const [dx, dy] = data.offset;
    let lx = ax + dx;
    let ly = ay + dy;
    lx = clamp(lx, 12, rect.width - 160);
    ly = clamp(ly, 12, rect.height - 50);
    const side = dx >= 0 ? -1 : 1;
    el.label.style.left = `${lx}px`;
    el.label.style.top = `${ly}px`;
    el.label.classList.add('show');
    const endX = dx >= 0 ? lx : lx + 145;
    const elbowX = ax + dx * 0.48;
    const d = `M ${ax.toFixed(1)} ${ay.toFixed(1)} L ${elbowX.toFixed(1)} ${(ay + dy * 0.25).toFixed(1)} L ${(dx >= 0 ? lx : lx + 3).toFixed(1)} ${ly.toFixed(1)}`;
    el.path.setAttribute('d', d);
    el.path.style.opacity = '0.72';
  }
}

// ===== 35. 选中框 / 部件选择 =====
let selectionBox = null;
let selectedPartName = '';
function ensureSelectionBox() {
  if (selectionBox) return;
  const geo = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
  const mat = new THREE.LineBasicMaterial({ color: 0x8debf4, transparent: true, opacity: 0.55, depthTest: false, toneMapped: false });
  selectionBox = new THREE.LineSegments(geo, mat);
  selectionBox.renderOrder = 25;
  selectionBox.visible = false;
  scene.add(selectionBox);
}
function updateSelectionBox() {
  if (!ui.selectedPart) {
    if (selectionBox) selectionBox.visible = false;
    return;
  }
  const group = partGroups.get(ui.selectedPart);
  if (!group) { if (selectionBox) selectionBox.visible = false; return; }
  ensureSelectionBox();
  const box = new THREE.Box3().setFromObject(group);
  if (!Number.isFinite(box.min.x) || box.isEmpty()) { selectionBox.visible = false; return; }
  const size = new THREE.Vector3(); box.getSize(size);
  const center = new THREE.Vector3(); box.getCenter(center);
  selectionBox.position.copy(center);
  selectionBox.scale.set(Math.max(0.08, size.x + 0.035), Math.max(0.08, size.y + 0.035), Math.max(0.08, size.z + 0.035));
  selectionBox.visible = true;
  selectionBox.material.opacity = 0.28 + 0.20 * Math.sin(performance.now() * 0.004);
  selectionBox.material.color.setHex(state.alarm ? 0xff6157 : 0x8debf4);
}

function isActuallyVisible(obj) {
  let o = obj;
  while (o) {
    if (o.visible === false) return false;
    o = o.parent;
  }
  return true;
}

function handleCanvasClick(event) {
  if (!renderer || !camera) return;
  const rect = canvasElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1
  );
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(ndc, camera);
  const hits = raycaster.intersectObjects(allPickables, false);
  const hit = hits.find((h) => isActuallyVisible(h.object));
  if (!hit) {
    clearSelection();
    return;
  }
  const partId = hit.object.userData.partId;
  if (!partId) return;
  if (SENSOR_MAP[partId]) {
    selectSensor(partId);
  } else {
    selectPart(partId);
  }
}

function selectPart(partId) {
  const meta = PART_META[partId];
  if (!meta) return;
  clearSensorSelection(false);
  ui.selectedPart = partId;
  ui.partSheetOpen = partId !== 'terminal';
  hideSensorSheet();
  ui.terminalOpen = partId === 'terminal';
  if (partId === 'terminal') {
    showTerminalCard(true);
    $('#partSheet')?.classList.remove('open');
  } else {
    showTerminalCard(false);
  }
  if (partId === 'nameplate') {
    setTimeout(() => openCard('nameplate'), 150);
  }
  if (ui.partSheetOpen) {
    const sheet = $('#partSheet');
    $('#partSheetTitle').textContent = meta.name;
    $('#partSheetEn').textContent = meta.en;
    renderPartSheet(partId);
    sheet.classList.add('open');
    setSelectedPartDataButton(partId);
  }
  selectedPartName = meta.name;
  if (ui.guideVisible && ui.guideStep <= 3) advanceGuideTo(4);
  updateSensorTagVisibility();
}

function setSelectedPartDataButton(partId) {
  const btn = $('#partSheetDataBtn');
  if (!btn) return;
  const mapping = {
    nameplate: 'nameplate', terminal: 'sensorLocations', windings: 'faultmap',
    statorCore: 'normalParams', rotorBars: 'faultmap', bearingFront: 'faultmap',
    sensor: 'sensorDetails'
  };
  const card = mapping[partId] || 'normalParams';
  btn.dataset.card = card;
  btn.textContent = partId === 'nameplate' ? '查看铭牌数据' : '相关数据';
}

function clearSelection() {
  ui.selectedPart = null;
  ui.selectedSensor = null;
  ui.partSheetOpen = false;
  ui.terminalOpen = false;
  showTerminalCard(false);
  hideSensorSheet();
  const sheet = $('#partSheet');
  if (sheet) sheet.classList.remove('open');
  clearSensorHighlight();
  updateSensorTagVisibility();
  if (selectionBox) selectionBox.visible = false;
  syncSensorUI();
}

function showTerminalCard(show) {
  const card = $('#terminalCard');
  if (!card) return;
  card.classList.toggle('open', !!show);
  if (show) {
    ui.terminalOpen = true;
    if (ui.guideVisible && ui.guideStep <= 5) advanceGuideTo(6);
  }
}

function focusPart(partId) {
  const meta = PART_META[partId];
  if (!meta) return;
  const pos = meta.focus || [5, 3, 5];
  const target = meta.target || [0, 0, 0];
  if (partId === 'statorCore' || partId === 'windings' || partId === 'rotorCore' || partId === 'rotorBars' || partId === 'bearingFront' || partId === 'bearingRear') {
    if (ui.structure === 'assembly') setStructure('section');
  }
  focusCamera(pos, target);
  setActiveCameraChip(null);
}

// ===== 36. 传感器选择 / 测量链 =====
function selectSensor(id) {
  const sensor = SENSOR_MAP[id];
  if (!sensor) return;
  if (ui.selectedSensor?.id === id && $('#sensorSheet')?.classList.contains('open')) {
    closeSensorSheet();
    return;
  }
  ui.selectedSensor = sensor;
  ui.selectedPart = id;
  ui.partSheetOpen = false;
  $('#partSheet')?.classList.remove('open');
  showTerminalCard(false);
  dismissGuide();
  fillSensorSheet();
  $('#sensorSheet')?.classList.add('open');
  focusCamera(sensor.camera.pos, sensor.camera.target);
  setActiveCameraChip(null);
  updateSensorHighlight(sensor);
  updateSignalPaths(sensor);
  updateSensorTagVisibility();
  syncSensorUI();
  updateSelectionBox();
  if (ui.guideVisible) advanceGuideTo(5);
}

function closeSensorSheet() {
  hideSensorSheet();
  clearSensorSelection(true);
}

function clearSensorSelection(clearPart = true) {
  ui.selectedSensor = null;
  clearSensorHighlight();
  if (groups.signalPaths) {
    for (const k in groups.signalPaths) groups.signalPaths[k].visible = false;
  }
  if (clearPart) ui.selectedPart = null;
  updateSensorTagVisibility();
  syncSensorUI();
}

function hideSensorSheet() {
  $('#sensorSheet')?.classList.remove('open');
}

function updateSensorHighlight(sensor) {
  if (!groups.sensorHighlights) return;
  for (const [key, item] of highlightObjects) {
    const match = key === sensor.partId || (sensor.partId === 'shaft' && key === 'rotor') ||
      (sensor.partId === 'statorCore' && key === 'statorCore') ||
      (sensor.partId === 'windings' && key === 'windings');
    item.object.visible = match;
    if (match) {
      const reading = getSensorReading(sensor.id);
      const alarm = reading.status === 'alarm';
      item.material.color.setHex(alarm ? 0xff564b : 0x72e4ef);
      item.material.depthTest = false;
      item.material.opacity = 0.12;
    }
  }
}

function clearSensorHighlight() {
  if (!groups.sensorHighlights) return;
  for (const item of highlightObjects.values()) {
    item.object.visible = false;
  }
}

function updateSignalPaths(sensor) {
  if (!groups.signalPaths) return;
  for (const k in groups.signalPaths) groups.signalPaths[k].visible = false;
  const g = groups.signalPaths[sensor.id];
  if (!g) return;
  g.visible = true;
  g.userData.line.visible = true;
  g.userData.pulse.material.opacity = 0.55;
}

function fillSensorSheet() {
  const sensor = ui.selectedSensor;
  if (!sensor) return;
  const reading = getSensorReading(sensor.id);
  $('#sensorKicker').textContent = `SENSOR ${sensor.idx} / MEASUREMENT CHAIN`;
  $('#sensorSheetTitle').textContent = sensor.name;
  $('#sensorSheetEn').textContent = sensor.en;
  $('#sensorSheetPos').textContent = sensor.position;
  $('#sensorSheetPrinciple').textContent = sensor.principle;
  $('#sensorSheetRange').textContent = `${sensor.range} · ${sensor.code}`;
  $('#sensorSheetPart').textContent = `${sensor.part} / ${sensor.partId}`;
  $('#sensorSheetValue').innerHTML = `${reading.display}<i>${sensor.unit}</i>`;
  const stateEl = $('#sensorSheetState');
  stateEl.className = `sensor-state ${reading.status}`;
  stateEl.textContent = reading.status === 'normal' ? 'NORMAL' : reading.status === 'warn' ? 'WARNING' : 'ALARM';
  $('#sensorSheetHints').innerHTML = sensor.hints.map((h) => `→ ${h}`).join('<br>');
  $('#sensorAbnormal').innerHTML = sensor.hints.map((h) => `<span>${h}</span>`).join('');
  $('#chainPart').textContent = sensor.part;
  $('#chainSensor').textContent = `${sensor.code} · ${reading.short}`;
  ['chainPart', 'chainSensor', 'chainDaq', 'chainFeature', 'chainSvm'].forEach((id, i) => {
    const node = $('#' + id);
    node.style.opacity = 1;
    node.style.transitionDelay = `${i * 45}ms`;
  });
  // measurement chain pulse direction
  const svmNode = $('#chainSvm');
  svmNode.style.color = reading.status === 'alarm' ? 'var(--red)' : '';
}

function refreshSensorSheet() {
  if (ui.selectedSensor) fillSensorSheet();
}

function syncSensorUI() {
  const selected = ui.selectedSensor?.id || null;
  $$('.sensor-cell').forEach((el) => el.classList.toggle('selected', el.dataset.sensor === selected));
  $$('.sensor-row').forEach((el) => el.classList.toggle('selected', el.dataset.sensor === selected));
}

function updateSensorTagVisibility() {
  const showAll = ui.cameraPreset === 'sensors';
  for (const sensor of SENSORS) {
    const tag = groups.sensorTags?.[sensor.id];
    if (!tag) continue;
    tag.visible = showAll || ui.selectedSensor?.id === sensor.id;
    const halo = groups.sensorHalos?.[sensor.id];
    if (halo) {
      halo.material.opacity = ui.selectedSensor?.id === sensor.id ? 0.55 : showAll ? 0.18 : 0;
    }
  }
}

// ============================================================================
// PART IV — measurement data, operator feedback & diagnostics
// ============================================================================

// ===== 37. 传感器读数（唯一数据源） =====
function getSensorReading(id) {
  const th = MC.thresholds;
  const off = !state.fieldOn;
  switch (id) {
    case 'hall': {
      const value = state.fieldOn ? state.currentA : 0;
      const status = value > th.currentTripA ? 'alarm' : value > th.currentWarnA ? 'warn' : 'normal';
      return { value, display: value.toFixed(2), short: '定子线电流', status, off };
    }
    case 'encoder': {
      const value = state.fieldOn ? state.nRpm : 0;
      let status = 'normal';
      if (state.fieldOn && state.nRpm < th.speedTripRpm) status = 'alarm';
      else if (state.fieldOn && state.nRpm < th.speedWarnRpm) status = 'warn';
      return { value, display: value.toFixed(0), short: '转子转速', status, off };
    }
    case 'pt100': {
      const value = state.tempC;
      const status = value > th.tempTripC ? 'alarm' : value > th.tempWarnC ? 'warn' : 'normal';
      return { value, display: value.toFixed(1), short: '壳体温度', status, off: false };
    }
    case 'vib': {
      const value = state.vibG;
      const status = value > th.vibTripG ? 'alarm' : value > th.vibWarnG ? 'warn' : 'normal';
      return { value, display: value.toFixed(3), short: '振动 RMS', status, off: false };
    }
    case 'photo': {
      const value = state.photoFreq;
      let status = 'normal';
      if (state.fieldOn && state.nRpm < 100) status = 'alarm';
      else if (!state.fieldOn) status = 'normal';
      return { value, display: value.toFixed(1), short: '通断脉冲', status, off };
    }
    default:
      return { value: 0, display: '—', short: '', status: 'normal', off: true };
  }
}

function computeStatusSnapshot() {
  if (!state.powerOn) {
    return { name: '已断电', en: 'POWER OFF', short: 'OFF', tone: 'off', tag: 'SAFE' };
  }
  if (state.tripped) {
    return { name: '保护停机', en: 'PROTECTION TRIP', short: 'TRIP', tone: 'alarm', tag: 'TRIP' };
  }
  if (!state.runCmd) {
    if (state.nRpm > 5) return { name: '惰转停机', en: 'COASTING', short: 'COAST', tone: 'warn', tag: 'COAST' };
    return { name: '待启动', en: 'READY', short: 'READY', tone: 'off', tag: 'READY' };
  }
  if (state.fault === 'stall' || (state.nRpm < 100 && state.sinceStart > 1.4)) {
    return { name: '卡滞堵转', en: 'LOCKED ROTOR', short: 'STALL', tone: 'alarm', tag: 'STALL' };
  }
  if (state.alarm) {
    return { name: '报警延时', en: 'ALARM DELAY', short: 'ALARM', tone: 'alarm', tag: 'ALARM' };
  }
  if (!state.fieldOn || (state.fault === 'normal' && state.nRpm < 1380 && state.loadPct < 106)) {
    return { name: '启动中', en: 'STARTING', short: 'START', tone: 'warn', tag: 'START' };
  }
  if (state.fault === 'overheat' && state.tempC > 63) {
    return { name: '绕组过热', en: 'OVERHEAT', short: 'HOT', tone: 'warn', tag: 'WARNING' };
  }
  if (state.fault === 'bearing') {
    return { name: '轴承磨损', en: 'BEARING WEAR', short: 'VIB', tone: 'warn', tag: 'WARNING' };
  }
  if (state.fault === 'broken') {
    return { name: '转子断条', en: 'BROKEN BAR', short: 'BAR', tone: 'warn', tag: 'WARNING' };
  }
  if (state.loadPct > 107) {
    return { name: '过载运行', en: 'OVERLOAD', short: 'OVER', tone: 'warn', tag: 'OVERLOAD' };
  }
  if (state.loadPct < 8) {
    return { name: '空载运行', en: 'NO LOAD', short: 'IDLE', tone: 'normal', tag: 'STABLE' };
  }
  return { name: '额定运行', en: 'RATED · S1', short: 'RATED', tone: 'normal', tag: 'STABLE' };
}

// ===== 38. 传感器 DOM / 波形 =====
let sensorDom = {};
function buildSensorUI() {
  if (sensorDom.built) return;
  const band = $('#sensorBand');
  const list = $('#sensorList');
  if (!band || !list) return;
  band.innerHTML = '';
  list.innerHTML = '';
  sensorDom.cells = {};
  sensorDom.rows = {};
  sensorDom.waves = {};
  for (const sensor of SENSORS) {
    const cell = document.createElement('button');
    cell.className = 'sensor-cell';
    cell.dataset.sensor = sensor.id;
    cell.setAttribute('type', 'button');
    cell.setAttribute('aria-label', `${sensor.name} 实时读数，点击查看测量链`);
    cell.innerHTML = `
      <div class="sc-top"><span class="sc-id">S${sensor.idx}</span><span class="sc-name">${sensor.short}</span><span class="sc-dot"></span></div>
      <div class="sc-main"><span class="sc-val">—</span><span class="sc-unit">${sensor.unit}</span></div>
      <div class="sc-range">${sensor.code} · ${sensor.range.split('·')[0].trim()}</div>
      <canvas class="sc-wave" width="90" height="28" aria-hidden="true"></canvas>`;
    cell.addEventListener('click', () => selectSensor(sensor.id));
    band.appendChild(cell);
    const row = document.createElement('button');
    row.className = 'sensor-row';
    row.dataset.sensor = sensor.id;
    row.setAttribute('type', 'button');
    row.innerHTML = `<span class="s-id">${sensor.idx}</span><span class="s-name">${sensor.name}</span><span class="s-val">—<i>${sensor.unit}</i></span><span class="s-dot"></span>`;
    row.addEventListener('click', () => selectSensor(sensor.id));
    list.appendChild(row);
    sensorDom.cells[sensor.id] = cell;
    sensorDom.rows[sensor.id] = row;
    sensorDom.waves[sensor.id] = cell.querySelector('canvas');
  }
  sensorDom.built = true;
}

function updateSensorUI() {
  buildSensorUI();
  for (const sensor of SENSORS) {
    const reading = getSensorReading(sensor.id);
    const cell = sensorDom.cells[sensor.id];
    const row = sensorDom.rows[sensor.id];
    if (!cell || !row) continue;
    cell.querySelector('.sc-val').textContent = reading.display;
    cell.querySelector('.sc-unit').textContent = sensor.unit;
    cell.classList.toggle('alarm', reading.status === 'alarm');
    cell.classList.toggle('warn', reading.status === 'warn');
    cell.classList.toggle('selected', ui.selectedSensor?.id === sensor.id);
    const dot = cell.querySelector('.sc-dot');
    dot.className = 'sc-dot' + (reading.status === 'alarm' ? ' alarm' : reading.status === 'warn' ? ' warn' : '');
    row.querySelector('.s-val').innerHTML = `${reading.display}<i>${sensor.unit}</i>`;
    row.classList.toggle('selected', ui.selectedSensor?.id === sensor.id);
    const rowDot = row.querySelector('.s-dot');
    rowDot.className = 's-dot' + (reading.status === 'alarm' ? ' alarm' : reading.status === 'warn' ? ' warn' : '');
    drawSensorWave(sensorDom.waves[sensor.id], sensor, reading);
  }
  $('#sensorBand').classList.toggle('off', !state.fieldOn);
}

function drawSensorWave(canvas, sensor, reading) {
  if (!canvas) return;
  const w = 90, h = 28;
  if (canvas.width !== w) { canvas.width = w; canvas.height = h; }
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, w, h);
  const color = reading.status === 'alarm' ? '#e45a52' : reading.status === 'warn' ? '#ffab5e' : '#70d8e5';
  ctx.strokeStyle = color;
  ctx.lineWidth = 0.85;
  ctx.globalAlpha = 0.56;
  ctx.beginPath();
  if (sensor.id === 'photo') {
    const hist = state.sensorHistory.photo.slice(-40);
    for (let i = 0; i < hist.length; i++) {
      const x = i / Math.max(1, hist.length - 1) * (w - 3) + 2;
      const y = hist[i] ? h * 0.32 : h * 0.7;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
  } else {
    let hist;
    if (sensor.id === 'hall') hist = state.sensorHistory.hall;
    else if (sensor.id === 'encoder') hist = state.sensorHistory.encoder;
    else if (sensor.id === 'pt100') hist = state.sensorHistory.pt100;
    else hist = state.sensorHistory.vib;
    hist = hist.slice(-48);
    let min = Infinity, max = -Infinity;
    for (const v of hist) { if (v < min) min = v; if (v > max) max = v; }
    const range = Math.max(1e-4, max - min);
    for (let i = 0; i < hist.length; i++) {
      const x = i / Math.max(1, hist.length - 1) * (w - 3) + 2;
      const y = h - 4 - ((hist[i] - min) / range) * (h - 8);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
  }
  ctx.stroke();
  ctx.globalAlpha = 0.25;
  ctx.strokeStyle = '#33414c';
  ctx.beginPath(); ctx.moveTo(2, h - 2); ctx.lineTo(w - 2, h - 2); ctx.stroke();
  ctx.globalAlpha = 1;
}

// ===== 39. 状态 / 面板刷新 =====
const domCache = new Map();
function D(id) {
  let el = domCache.get(id);
  if (!el) { el = document.getElementById(id); domCache.set(id, el); }
  return el;
}
function setTxt(id, text) {
  const el = D(id);
  if (el && el.textContent !== String(text)) el.textContent = String(text);
}
function setHTML(id, html) {
  const el = D(id);
  if (el && el.innerHTML !== html) el.innerHTML = html;
}
function setRangeValueIfIdle(el, value) {
  if (!el || ui.loadDragging) return;
  if (String(el.value) !== String(value)) {
    el.value = value;
    setRangeFill(el);
  }
}

function refreshAllUI(force = false) {
  if (!sceneReady && !force) return;
  if (!sensorDom.built) buildSensorUI();
  const status = computeStatusSnapshot();
  state.statusKey = status.short;
  state.uiNow = performance.now();
  const top = D('topState');
  if (top) {
    top.className = 'live-pill' + (status.tone === 'alarm' ? ' alarm' : status.tone === 'warn' ? ' warn' : '');
    setTxt('topStateText', status.name);
  }
  const stage = D('stage');
  stage?.classList.toggle('is-alarm', !!state.alarm || state.tripped);
  setTxt('statusName', status.name);
  setTxt('statusSub', status.en);
  setTxt('statusTag', status.tag);
  const statusTag = D('statusTag');
  statusTag.className = 'state-tag' + (status.tone === 'alarm' ? ' alarm' : status.tone === 'warn' ? ' warn' : '');
  const dot = D('statusDot');
  if (dot) {
    dot.style.background = status.tone === 'alarm' ? 'var(--red)' : status.tone === 'warn' ? 'var(--amber)' : status.tone === 'off' ? 'var(--muted)' : 'var(--green)';
    dot.style.boxShadow = status.tone === 'alarm' ? '0 0 9px rgba(228,90,82,.8)' : status.tone === 'warn' ? '0 0 9px rgba(255,171,94,.6)' : '0 0 9px rgba(131,211,164,.55)';
  }

  const I = state.fieldOn ? state.currentA : 0;
  setTxt('currentValue', I.toFixed(2));
  setTxt('speedValue', state.fieldOn ? state.nRpm.toFixed(0) : '0');
  setTxt('tempValue', state.tempC.toFixed(1));
  setTxt('torqueValue', state.torqueNm.toFixed(2));
  setTxt('slipValue', state.slip.toFixed(3));
  setTxt('powerValue', state.powerKW.toFixed(2));
  const currentValueEl = D('currentValue');
  if (currentValueEl) currentValueEl.style.color = I > 3.5 ? 'var(--red)' : I > 3.2 ? 'var(--amber)' : '';
  setTxt('speedSlipRef', state.slip.toFixed(3));
  setTxt('heroLoad', state.loadTarget.toFixed(0));
  const loadVal = D('loadValue');
  if (loadVal) loadVal.innerHTML = `${state.loadTarget.toFixed(0)}<i>%</i>`;
  const loadRange = D('loadRange');
  setRangeValueIfIdle(loadRange, state.loadTarget);
  setRangeFill(loadRange);
  const manual = D('manualScale');
  if (manual) {
    const display = ui.manualScale;
    setTxt('manualScaleValue', `${display.toFixed(3)}×`);
  }

  // Mechanism HUD
  const paused = ui.paused;
  const obsName = paused ? 'PAUSED · 已暂停' :
    ui.observation === 'auto' ? 'AUTO · 慢放' :
    ui.observation === 'ultra' ? 'ULTRA · 超慢' :
    ui.observation === 'manual' ? 'MANUAL · 手动' : 'REAL ×1';
  setTxt('hudMode', obsName);
  setTxt('hudActual', `${state.nRpm.toFixed(0)}r/min`);
  D('hudActual').innerHTML = `${state.nRpm.toFixed(0)}<i>r/min</i>`;
  D('hudDisplay').innerHTML = `${(ui.paused ? 0 : state.rotorDisplayRpm).toFixed(1)}<i>r/min</i>`;
  D('hudField').innerHTML = `${(ui.paused ? 0 : state.fieldDisplayRpm).toFixed(1)}<i>r/min</i>`;
  const ratio = state.fieldOn ? (ui.paused ? 0 : state.fieldDisplayRpm / Math.max(1, state.nRpm)) : 0;
  setTxt('hudScale', state.nRpm > 0.5 ? `${state.displayScale.toFixed(3)}×` : '—');
  setTxt('hudSlip', state.slip.toFixed(3));
  const slipElec = wrapSignedDeg((state.displayFieldAngle - state.displayRotorAngle) * MC.polePairs * RAD2DEG);
  setTxt('hudSlipAngle', `${slipElec.toFixed(1)}°`);

  // Phase track
  const fieldElec = wrapDeg(state.displayFieldAngle * MC.polePairs * RAD2DEG);
  const rotorElec = wrapDeg(state.displayRotorAngle * MC.polePairs * RAD2DEG);
  const fieldMarker = D('fieldMarker'); const rotorMarker = D('rotorMarker');
  if (fieldMarker) fieldMarker.style.left = `${(fieldElec / 360 * 100).toFixed(3)}%`;
  if (rotorMarker) rotorMarker.style.left = `${(rotorElec / 360 * 100).toFixed(3)}%`;
  setTxt('mechAngle', `${wrapDeg(state.displayRotorAngle * RAD2DEG).toFixed(1)}°`);
  setTxt('elecAngle', `${rotorElec.toFixed(1)}°`);
  setTxt('slipAngle', `${slipElec.toFixed(1)}°`);

  // Top quality select and buttons
  const powerBtn = D('powerBtn');
  if (powerBtn) {
    powerBtn.setAttribute('aria-pressed', String(state.powerOn));
    powerBtn.classList.toggle('primary', state.powerOn);
    const label = powerBtn.querySelector('span:last-child');
    if (label) label.textContent = state.powerOn ? '断电' : '合闸';
  }
  const startBtn = D('startBtn');
  if (startBtn) startBtn.disabled = state.tripped || (state.fieldOn && state.runCmd);
  const stopBtn = D('stopBtn');
  if (stopBtn) stopBtn.disabled = !state.powerOn || (!state.fieldOn && state.nRpm < 0.5);
  setTxt('faultHint', FAULT_META[state.fault]?.hint || '');

  // Fault chips
  $$('.condition').forEach((b) => b.classList.toggle('active', b.dataset.fault === state.fault));
  $$('.display-toggle').forEach((b) => {
    const map = { fieldToggle: 'fieldVisible', barsToggle: 'barsVisible', airToggle: 'airflow', annotToggle: 'annotations' };
    const key = map[b.id];
    if (key) {
      b.classList.toggle('active', !!ui[key]);
      b.setAttribute('aria-pressed', String(!!ui[key]));
    }
    if (b.id === 'terminalBtn') b.classList.toggle('active', ui.terminalOpen);
  });

  // Sensor band and list
  updateSensorUI();

  // Flow steps
  const stepBase = state.alarm ? 5 : Math.floor((state.simTime % 2.4) / 0.4);
  const activeStep = clamp(stepBase, 0, 5);
  $$('#flowStrip .flow-step').forEach((el) => {
    const idx = Number(el.dataset.flow);
    el.classList.toggle('active', idx === activeStep);
    el.classList.toggle('done', idx < activeStep && idx !== activeStep);
  });

  // Feature cells
  const f = state.feature;
  setTxt('fMean', f.currentMean.toFixed(2));
  setTxt('fVar', f.currentVar.toFixed(4));
  setTxt('fKurt', f.currentKurt.toFixed(2));
  setTxt('fRate', fmtSigned(f.speedRate, 2));
  setTxt('fErr', f.followError.toFixed(1));
  setTxt('fTemp', fmtSigned(f.tempSlope, 2));
  const featureHot = [
    f.currentVar > 0.008, f.currentKurt > 4.2,
    Math.abs(f.speedRate) > 1.2, f.followError > 20, Math.abs(f.tempSlope) > 0.12
  ];
  $$('.feature-cell').forEach((el, i) => el.classList.toggle('hot', !!featureHot[i]));

  // SVM widget
  const svmActive = state.fieldOn && state.sinceStart > 3.8;
  const svmMini = D('svmMini');
  const svmOverload = svmActive && !state.svmNormal && state.fault === 'overload';
  setTxt('svmVerdict', svmActive ? (state.svmNormal ? '正常类 · HEALTHY' : `${state.faultLabel} · FAULT`) : '待机 · STANDBY');
  const svmVerdict = D('svmVerdict');
  svmVerdict.className = 'svm-v' + (svmActive ? state.svmNormal ? '' : svmOverload ? ' warn' : ' fault' : '');
  const pct = svmActive ? Math.round(state.svmConfidence * 100) : 0;
  setTxt('svmConf', svmActive ? `${pct}%` : '—');
  D('svmBar').style.width = `${pct}%`;
  const svmBar = D('svmBar');
  svmBar.style.background = !svmActive ? 'var(--muted-2)' : state.svmNormal ? 'var(--cyan)' : svmOverload ? 'var(--amber)' : 'var(--red)';
  setTxt('svmFx', svmActive ? `f(x) = ${state.svmScore >= 0 ? '+' : '−'}${Math.abs(state.svmScore).toFixed(2)}` : 'f(x) = —');
  if (svmMini) svmMini.classList.toggle('fault', svmActive && !state.svmNormal && !svmOverload);
  if (svmMini) svmMini.classList.toggle('warn', svmOverload);

  // Alarm banner
  const banner = D('alarmBanner');
  if (banner) {
    if (state.alarm) {
      banner.classList.add('show');
      setTxt('alarmText', state.alarm.text);
      const alarmSub = state.alarm.executed
        ? `${state.alarm.threshold} · 当前 ${state.alarm.reading} · 保护停机已执行`
        : `${state.alarm.threshold} · 当前 ${state.alarm.reading} · 保护延时 ${state.alarm.countdown.toFixed(1)} s`;
      setTxt('alarmSub', alarmSub);
    } else {
      banner.classList.remove('show');
    }
  }
  updateAlarmPreview();
  updateAlarmBeacon();
  if (ui.selectedSensor) refreshSensorSheet();
  if (ui.selectedPart && !ui.selectedSensor && ui.partSheetOpen) updatePartSheetDynamic();
  if (ui.cinemaActive) updateCinemaHUD();
  setTxt('clockMeta', `LOCAL ${nowClock()}`);
  // thermal acceleration badge
  const thermal = $('#thermalAccel');
  if (thermal) {
    setTxt('thermalBadge', thermal.checked ? 'ACCEL ×10' : 'REAL ×1');
    setTxt('thermalLabel', thermal.checked ? '热过程 ×10' : '热过程实时');
    $('#thermalBadge')?.classList.toggle('active', thermal.checked);
  }
  // quick toolbar states mirror the full controls
  const fq = D('fieldQuickBtn'), aq = D('annotQuickBtn');
  if (fq) {
    fq.setAttribute('aria-pressed', String(ui.fieldVisible));
    fq.style.color = ui.fieldVisible ? 'var(--cyan)' : '';
    fq.style.borderColor = ui.fieldVisible ? 'rgba(112,216,229,.28)' : '';
  }
  if (aq) {
    aq.setAttribute('aria-pressed', String(ui.annotations));
    aq.style.color = ui.annotations ? '' : 'var(--muted-2)';
  }
}

function wrapSignedDeg(deg) {
  let v = ((deg + 180) % 360 + 360) % 360 - 180;
  return v;
}

function updateAlarmPreview() {
  const preview = D('alarmPreview');
  if (!preview) return;
  if (!state.alarmLog.length) {
    setHTML('alarmPreview', '<div class="alarm-empty">暂无报警 · 保护阈值 I&gt;3.5 A · n&lt;100 r/min · T&gt;75 ℃</div>');
    return;
  }
  const rows = state.alarmLog.slice(0, 2).map((a) => `
    <button class="alarm-item" data-open-card="alarms">
      <span class="ai-time">${a.time}</span>
      <span class="ai-type">${a.type === 'stall' ? '堵转 / 过流' : a.type === 'overheat' ? '超温' : a.type}</span>
      <span class="ai-val">${a.reading}</span>
    </button>`).join('');
  if (preview.dataset.signature !== rows) {
    preview.innerHTML = rows;
    preview.dataset.signature = rows;
    $$('.alarm-item', preview).forEach((el) => el.addEventListener('click', () => openCard('alarms')));
  }
}

function updateAlarmBeacon() {
  if (!groups.beaconMats) return;
  const mats = groups.beaconMats;
  let red = 0.05, amber = 0.03, green = 0.04;
  let halo = 0;
  if (state.alarm || state.tripped) {
    red = 0.55 + 0.45 * Math.abs(Math.sin(state.simTime * 6.5));
    amber = 0.02; green = 0.01;
    halo = 0.45 + 0.22 * Math.sin(state.simTime * 6.5);
    if (alarmLight) alarmLight.intensity = 2.4 + 1.8 * Math.max(0, Math.sin(state.simTime * 6.5));
  } else if (state.fault !== 'normal' || state.currentA > MC.thresholds.currentWarnA || state.tempC > MC.thresholds.tempWarnC) {
    amber = 0.65; red = 0.03; green = 0.02; halo = 0.2;
    if (alarmLight) alarmLight.intensity = 0;
  } else {
    green = 0.75;
    if (alarmLight) alarmLight.intensity = 0;
  }
  mats.red.emissiveIntensity = red * 2.1;
  mats.amber.emissiveIntensity = amber * 2.1;
  mats.green.emissiveIntensity = green * 2.1;
  mats.halo.material.opacity = halo;
  mats.halo.material.color.setHex(state.alarm || state.tripped ? 0xff554b : 0xffba71);
  if (alarmLight) alarmLight.color.setHex(state.alarm || state.tripped ? 0xff3d34 : 0xffab5e);
}

function renderPartSheet(partId) {
  const meta = PART_META[partId];
  if (!meta) return;
  const motion = partMotionText(partId);
  const desc = D('partSheetDesc');
  if (desc) desc.innerHTML = `${meta.desc}<div class="tt-motion" style="margin-top:8px;border-top:1px solid var(--line-soft);padding-top:7px">当前状态 <b>${motion}</b></div>`;
  setTxt('partSheetTitle', meta.name);
  setTxt('partSheetEn', meta.en);
  setHTML('partSheetMeta', [`材料 · ${meta.material}`, ...meta.specs].map((s) => `<span>${s}</span>`).join(''));
}
function updatePartSheetDynamic() {
  if (!ui.selectedPart) return;
  const motion = partMotionText(ui.selectedPart);
  const sheet = D('partSheetDesc');
  if (sheet) {
    const div = sheet.querySelector('.tt-motion b');
    if (div && div.textContent !== motion) div.textContent = motion;
  }
}

function partMotionText(partId) {
  const n = state.nRpm;
  const sensor = ui.selectedSensor;
  switch (partId) {
    case 'rotorCore': case 'rotorBars': case 'shaft':
      if (!state.fieldOn && n < 1) return `静止 · 断电，温度 ${state.tempC.toFixed(1)} ℃`;
      if (!state.fieldOn) return `断电惰转中 · ${n.toFixed(0)} r/min`;
      if (n < 1 && state.fault === 'stall') return `堵转停转 · s = 1.00，电流 ${state.currentA.toFixed(2)} A，保护延时中`;
      return `正常旋转中 · n ${n.toFixed(0)} r/min，转差率 ${state.slip.toFixed(3)}`;
    case 'fan':
      if (!state.fieldOn && n < 1) return '完全停转 · 无强制风冷';
      if (state.fault === 'stall') return '随转子堵转停转 · 气流中断';
      return `同轴同速旋转 · ${n.toFixed(0)} r/min · 风量随转速变化`;
    case 'bearingFront': case 'bearingRear': {
      const v = getSensorReading('vib');
      if (state.fault === 'bearing' && partId === 'bearingFront') return `滚道冲击 · 振动 RMS ${v.display} g · 轴承磨损预警`;
      return `随轴旋转 · 振动 RMS ${v.display} g`;
    }
    case 'windings':
      if (!state.fieldOn) return `断电 · 绕组温度随壳体回落（${state.tempC.toFixed(1)} ℃）`;
      if (state.fault === 'overheat') return `U/V/W 通流 · 温升 ${state.tempC.toFixed(1)} ℃ · 异常发热`;
      return `U/V/W 三相通流 · ${state.currentA.toFixed(2)} A · 壳体 ${state.tempC.toFixed(1)} ℃`;
    case 'statorCore':
      if (state.fault === 'overheat') return `铁芯与绕组热耦合 · ${state.tempC.toFixed(1)} ℃`;
      return `静止 · 形成 4 极旋转磁路 · ${state.tempC.toFixed(1)} ℃`;
    case 'frame': case 'base': case 'frontCap': case 'rearCap': case 'fanCowl':
    case 'nameplate': case 'daq':
      return `静止固定 · 壳体 ${state.tempC.toFixed(1)} ℃`;
    case 'terminal':
      return `固定 · 当前演示接法 ${ui.connection === 'delta' ? '△ 380 V' : 'Y 降压示意'}`;
    case 'hall': {
      const r = getSensorReading('hall');
      return `测 ${r.display} A · ${r.status === 'normal' ? '正常' : r.status === 'warn' ? '预警' : '越限'}`;
    }
    case 'encoder': return `测 ${getSensorReading('encoder').display} r/min`;
    case 'pt100': return `测 ${getSensorReading('pt100').display} ℃`;
    case 'vib': return `测 ${getSensorReading('vib').display} g RMS`;
    case 'photo': return `${getSensorReading('photo').display} p/s · ${state.nRpm < 100 && state.fieldOn ? '转子停转' : '脉冲存在'}`;
    default: return state.fieldOn ? `运行中 · n ${n.toFixed(0)} r/min` : '静止';
  }
}

// ===== 40. 操作动作 =====
function setPower(on) {
  if (on && state.tripped) {
    showToast('保护已锁定，请先复位报警', 'alarm');
    return;
  }
  state.powerOn = !!on;
  if (!on) {
    state.runCmd = false;
    state.fieldOn = false;
    state.fieldAlpha = 0;
    audioEngine?.powerOff();
    showToast('主电源已断开 · 转子惰转，温度缓慢回落', 'warn');
  } else {
    state.runCmd = false;
    showToast('主电源已合闸 · 待启动', 'info');
  }
  refreshAllUI(true);
}

function startMotor() {
  if (state.tripped) {
    showToast('保护已锁定，请先复位报警', 'alarm');
    return;
  }
  if (!state.powerOn) state.powerOn = true;
  if (!state.runCmd) {
    state.runCmd = true;
    state.sinceStart = 0;
    state.startTransient = 7.6;
    if (ui.paused) {
      ui.preStartPaused = true;
      setPauseMechanism(false, { silent: true });
    }
    flashCurrentGauge();
    if (ui.cameraPreset === 'hero' && !ui.cinemaActive) {
      const p = getResponsiveCamera('hero');
      const target = new THREE.Vector3(...p.target);
      const pos = new THREE.Vector3(...p.pos).sub(target).multiplyScalar(0.965).add(target);
      orbitRig.transitionTo(pos.toArray(), target.toArray(), 1.6);
    }
    if (state.fault === 'stall') {
      showToast('启动指令已执行 · 但输出轴仍被楔块卡住', 'warn');
    } else {
      showToast('启动指令已执行 · 旋转磁场建立中', 'info');
    }
  }
  if (ui.cinemaActive) { /* cinema continues */ }
  refreshAllUI(true);
}

function stopMotor() {
  if (!state.powerOn) return;
  state.runCmd = false;
  state.tripped = false;
  showToast('停机指令 · 定子磁场撤去，转子惰转', 'info');
  if (ui.cinemaActive) stopCinematic(true);
  refreshAllUI(true);
}

function flashCurrentGauge() {
  const el = $('#currentValue');
  if (!el) return;
  el.classList.remove('flash-inrush');
  void el.offsetWidth;
  el.classList.add('flash-inrush');
  setTimeout(() => el.classList.remove('flash-inrush'), 720);
}

function resetAlarmAndFault() {
  state.alarm = null;
  state.tripped = false;
  state.runCmd = false;
  state.fault = 'normal';
  state.loadTarget = 100;
  state.alarmCounters = { current: 0, speed: 0, temp: 0 };
  ui.selectedSensor = null;
  hideSensorSheet();
  clearSensorSelection(false);
  showToast('报警已复位 · 工况归零，请重新启动', 'info');
  audioEngine?.alarmOff();
  refreshAllUI(true);
}

function clearAlarmOnly() {
  state.alarm = null;
  state.tripped = false;
  state.alarmCounters = { current: 0, speed: 0, temp: 0 };
  audioEngine?.alarmOff();
}

function injectFault(faultName) {
  if (!FAULT_META[faultName]) return;
  if (state.alarm) {
    clearAlarmOnly();
  }
  state.tripped = false;
  state.fault = faultName;
  if (faultName === 'normal') {
    state.loadTarget = 100;
    showToast('工况已恢复：额定负载 / 健康运行', 'info');
  } else if (faultName === 'overload') {
    state.loadTarget = 112;
    showToast('过载工况注入 · 负载 112%，观察 T(s) 工作点下滑', 'warn');
  } else if (faultName === 'stall') {
    showToast('卡滞堵转注入 · 楔块已进入输出轴，转子将快速停转', 'alarm');
    audioEngine?.setFault('stall');
  } else if (faultName === 'overheat') {
    showToast('绕组过热注入 · 热过程带惯性，请观察温度斜率', 'warn');
  } else if (faultName === 'bearing') {
    showToast('轴承磨损注入 · 前端轴承滚道出现周期性冲击', 'warn');
  } else if (faultName === 'broken') {
    showToast('转子断条注入 · 两根导条断裂，电流出现 2sf 边频', 'warn');
  }
  if (!state.fieldOn && faultName !== 'normal') {
    showToast('电机当前未运行，启动后故障特征才会建立', 'info');
  }
  if (ui.guideVisible) advanceGuideTo(7);
  refreshAllUI(true);
}

function setObservationMode(mode, silent = false) {
  if (!['auto', 'ultra', 'manual', 'real'].includes(mode)) return;
  ui.observation = mode;
  syncObservationUI();
  if (mode === 'manual') {
    ui.manualScale = Number($('#manualScale').value) / 100;
  }
  if (!silent) {
    const labels = { auto: '自动慢放 · 目标 30 r/min', ultra: '超慢观察 · 目标 12 r/min', manual: `手动倍率 ${ui.manualScale.toFixed(3)}×`, real: '真实转速 1× · 可能出现高速模糊' };
    showToast(labels[mode], 'info');
  }
  if (ui.guideVisible && mode === 'auto') advanceGuideTo(5);
  refreshAllUI(true);
}

function toggleField(force = null) {
  ui.fieldVisible = force === null ? !ui.fieldVisible : !!force;
  if (ui.fieldVisible && ui.structure === 'assembly') {
    setStructure('section');
    setCameraPreset('field');
  }
  if (ui.fieldVisible) {
    state.displayFieldAngle = state.displayFieldAngle || 0;
  }
  updateSensorTagVisibility();
  refreshAllUI(true);
}

function toggleAnnotations(force = null) {
  ui.annotations = force === null ? !ui.annotations : !!force;
  refreshAllUI(true);
}

function toggleAirflow() {
  ui.airflow = !ui.airflow;
  if (ui.airflow && ui.structure === 'assembly') {
    setStructure('section');
  }
  refreshAllUI(true);
}

function toggleBars() {
  ui.barsVisible = !ui.barsVisible;
  if (groups.bars) groups.bars.visible = ui.barsVisible;
  refreshAllUI(true);
}

function setConnection(mode) {
  ui.connection = mode;
  $$('.connection-switch button').forEach((b) => b.classList.toggle('active', b.dataset.conn === mode));
  const note = $('#terminalNote');
  if (note) note.innerHTML = mode === 'delta'
    ? '当前为 <b>△ 380 V</b> 额定运行接法。'
    : '当前为 <b>Y 接降压启动</b> 教学示意；连片移至星点。';
  showToast(mode === 'delta' ? '切换至 △ 接法 · 额定运行' : '切换至 Y 接法 · 仅示意降压启动，不接入运行模型', 'info');
}

function toggleTerminal(force = null) {
  const next = force === null ? !ui.terminalOpen : !!force;
  if (next) {
    const was = ui.selectedPart;
    selectPart('terminal');
    ui.terminalOpen = true;
    showTerminalCard(true);
  } else {
    ui.terminalOpen = false;
    showTerminalCard(false);
  }
}

function setStructureFromTab(mode) {
  if (mode === ui.structure) {
    if (mode === 'exploded') {
      setStructure('assembly');
      setCameraPreset('hero');
      return;
    }
    return;
  }
  setStructure(mode);
  if (mode === 'assembly') {
    setCameraPreset('hero');
  } else if (mode === 'section') {
    setCameraPreset('section');
  } else if (mode === 'exploded') {
    ui.cameraPreset = 'rotor';
    updateCameraChips();
    const p = getResponsiveCamera('exploded');
    orbitRig.transitionTo(p.pos, p.target, 1.35);
  }
}

// ============================================================================
// PART V — pure canvas data views / enlarged card factory / data atlas
// ============================================================================
const C = {
  bg: '#070b10',
  text: '#dce7ec',
  muted: '#8b98a5',
  dim: '#586673',
  line: 'rgba(160,190,208,.12)',
  line2: 'rgba(160,190,208,.06)',
  cyan: '#70d8e5',
  cyanDim: 'rgba(112,216,229,.22)',
  amber: '#ffab5e',
  amberDim: 'rgba(255,171,94,.20)',
  red: '#e45a52',
  redDim: 'rgba(228,90,82,.20)',
  green: '#83d3a4',
  steel: '#b9c5cf',
  copper: '#c68155'
};
const CANVAS_MONO = '"Cascadia Code","SFMono-Regular","Consolas",monospace';
const CANVAS_SANS = '"Bahnschrift","DIN Alternate","Segoe UI","Microsoft YaHei",sans-serif';
const CANVAS_CN = '"Microsoft YaHei UI","PingFang SC","Noto Sans SC",sans-serif';

const cardRegistry = new Map();
function registerCard(def) {
  cardRegistry.set(def.id, def);
  return def;
}

function canvasBg(ctx, w, h, opts = {}) {
  ctx.save();
  const grd = ctx.createLinearGradient(0, 0, w, h);
  grd.addColorStop(0, '#0b1118');
  grd.addColorStop(0.55, '#070c11');
  grd.addColorStop(1, '#060a0e');
  ctx.fillStyle = grd;
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(160,190,208,.12)';
  ctx.lineWidth = 1;
  ctx.strokeRect(0.5, 0.5, w - 1, h - 1);
  // very subtle industrial corner ticks
  ctx.strokeStyle = 'rgba(112,216,229,.18)';
  const tick = Math.min(10, w * 0.025);
  ctx.beginPath();
  ctx.moveTo(8, 8 + tick); ctx.lineTo(8, 8); ctx.lineTo(8 + tick, 8);
  ctx.moveTo(w - 8 - tick, 8); ctx.lineTo(w - 8, 8); ctx.lineTo(w - 8, 8 + tick);
  ctx.stroke();
  ctx.restore();
}

function mono(ctx, size, color = C.steel, weight = 500, align = 'left') {
  ctx.font = `${weight} ${size}px ${CANVAS_MONO}`;
  ctx.fillStyle = color;
  ctx.textAlign = align;
  ctx.textBaseline = 'middle';
}
function zh(ctx, size, color = C.muted, weight = 500, align = 'left') {
  ctx.font = `${weight} ${size}px ${CANVAS_CN}`;
  ctx.fillStyle = color;
  ctx.textAlign = align;
  ctx.textBaseline = 'middle';
}
function vline(ctx, x, y0, y1, color = C.line, width = 1) {
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath(); ctx.moveTo(x, y0); ctx.lineTo(x, y1); ctx.stroke();
}
function hline(ctx, y, x0, x1, color = C.line, width = 1) {
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x1, y); ctx.stroke();
}
function dashedLine(ctx, x0, y0, x1, y1, color = C.redDim, dash = [4, 4], width = 1) {
  ctx.save(); ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = width;
  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); ctx.restore();
}
function miniStat(ctx, x, y, w, value, label, color = C.steel) {
  ctx.fillStyle = 'rgba(255,255,255,.018)';
  ctx.fillRect(x, y, w, 22);
  ctx.strokeStyle = 'rgba(160,190,208,.10)';
  ctx.strokeRect(x + .5, y + .5, w - 1, 21);
  mono(ctx, 7.2, color, 500, 'left');
  ctx.fillText(value, x + 5, y + 8);
  mono(ctx, 5.2, C.dim, 500, 'left');
  ctx.fillText(label, x + 5, y + 16);
}
function roundRectPath(ctx, x, y, w, h, r = 3) {
  const rr = Math.min(r, w / 2, h / 2);
  ctx.beginPath();
  ctx.moveTo(x + rr, y);
  ctx.arcTo(x + w, y, x + w, y + h, rr);
  ctx.arcTo(x + w, y + h, x, y + h, rr);
  ctx.arcTo(x, y + h, x, y, rr);
  ctx.arcTo(x, y, x + w, y, rr);
  ctx.closePath();
}
function getCanvasCtx(canvas, widthCss, heightCss, dpr = Math.min(window.devicePixelRatio || 1, 2)) {
  const w = Math.max(40, Math.round(widthCss));
  const h = Math.max(30, Math.round(heightCss));
  const pxW = Math.max(1, Math.round(w * dpr));
  const pxH = Math.max(1, Math.round(h * dpr));
  if (canvas.width !== pxW || canvas.height !== pxH) {
    canvas.width = pxW; canvas.height = pxH;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(pxW / w, 0, 0, pxH / h, 0, 0);
  return ctx;
}
function drawCurvePath(ctx, pts, xMap, yMap, color, lineWidth, fillArea = false, plot = null) {
  if (!pts.length) return;
  ctx.save();
  ctx.beginPath();
  pts.forEach((p, i) => {
    const x = xMap(p.t ?? p.x);
    const y = yMap(p.v ?? p.y);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  if (fillArea && plot) {
    const lastX = xMap(pts[pts.length - 1].t ?? pts[pts.length - 1].x);
    const firstX = xMap(pts[0].t ?? pts[0].x);
    ctx.lineTo(lastX, plot.y1);
    ctx.lineTo(firstX, plot.y1);
    ctx.closePath();
    ctx.fillStyle = color.replace(')', ',.10)').replace('rgb', 'rgba').replace('#70d8e5', 'rgba(112,216,229,.08)');
    if (color.startsWith('#')) {
      ctx.fillStyle = color + '14';
    }
    ctx.fill();
    ctx.beginPath();
    pts.forEach((p, i) => {
      const x = xMap(p.t ?? p.x);
      const y = yMap(p.v ?? p.y);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.stroke();
  ctx.restore();
}
function axisLabel(ctx, text, x, y, color = C.dim, align = 'center', size = 8) {
  mono(ctx, size, color, 500, align);
  ctx.fillText(text, x, y);
}
function trendRange() { return ui.modalCardId === 'trend' ? (ui.modalRange || 60) : 60; }

// ---- trend / three channel ----
function drawTrendCard(ctx, w, h, opts = {}) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  if (!large) {
    drawTrendCompact(ctx, w, h, opts);
    return;
  }
  const pad = { l: large ? 132 : 31, r: large ? 24 : 10, t: large ? 30 : 8, b: large ? 32 : 14 };
  const plot = { x0: pad.l, x1: w - pad.r, y0: pad.t, y1: h - pad.b };
  const now = state.simTime;
  const range = ui.modalCardId === 'trend' ? (ui.modalRange || 60) : 60;
  const t0 = now - range;
  const data = state.trend.filter((p) => p.t >= t0);
  const lanes = [
    { key: 'I', name: '定子电流 I', unit: 'A', color: C.cyan, min: 0, max: 6.5, warn: MC.thresholds.currentWarnA, trip: MC.thresholds.currentTripA, rated: MC.ratedCurrentA, val: state.fieldOn ? state.currentA : 0 },
    { key: 'n', name: '实际转速 n', unit: 'r/min', color: C.steel, min: 0, max: 1600, warn: MC.thresholds.speedWarnRpm, trip: MC.thresholds.speedTripRpm, rated: MC.ratedSpeedRpm, val: state.nRpm },
    { key: 'T', name: '壳体温度 T', unit: '℃', color: C.amber, min: 20, max: 90, warn: MC.thresholds.tempWarnC, trip: MC.thresholds.tempTripC, rated: MC.steadyCaseC, val: state.tempC }
  ];
  const laneGap = large ? 12 : 4;
  const totalHeight = plot.y1 - plot.y0;
  const laneH = (totalHeight - laneGap * 2) / 3;
  lanes.forEach((lane, li) => {
    const y0 = plot.y0 + li * (laneH + laneGap);
    const y1 = y0 + laneH;
    const mapY = (v) => y1 - clamp((v - lane.min) / (lane.max - lane.min), 0, 1) * (y1 - y0);
    const mapT = (t) => plot.x0 + clamp((t - t0) / range, 0, 1) * (plot.x1 - plot.x0);
    // lane background and grid
    ctx.fillStyle = li % 2 ? 'rgba(255,255,255,.012)' : 'rgba(255,255,255,.022)';
    ctx.fillRect(plot.x0, y0, plot.x1 - plot.x0, y1 - y0);
    hline(ctx, y0, plot.x0, plot.x1, C.line2);
    hline(ctx, y1, plot.x0, plot.x1, C.line2);
    const gridRows = large ? 3 : 2;
    for (let i = 1; i < gridRows; i++) {
      const yy = lerp(y0, y1, i / gridRows);
      hline(ctx, yy, plot.x0, plot.x1, C.line2);
    }
    // threshold and rated reference
    if (lane.trip >= lane.min && lane.trip <= lane.max) {
      dashedLine(ctx, plot.x0, mapY(lane.trip), plot.x1, mapY(lane.trip), li === 0 ? 'rgba(228,90,82,.48)' : 'rgba(228,90,82,.30)', large ? [5, 5] : [3, 4], 1);
      if (large) {
        axisLabel(ctx, `TRIP ${lane.trip}`, plot.x0 - 6, mapY(lane.trip), C.red, 'right', 7);
      }
    }
    if (lane.rated >= lane.min && lane.rated <= lane.max) {
      dashedLine(ctx, plot.x0, mapY(lane.rated), plot.x1, mapY(lane.rated), 'rgba(160,190,208,.17)', [2, 5], 1);
      if (large) axisLabel(ctx, `RATED ${lane.rated}`, plot.x0 - 6, mapY(lane.rated), C.dim, 'right', 6.5);
    }
    const points = data.map((p) => ({ t: p.t, v: p.key === 'I' ? p.I : p.key === 'n' ? p.n : p.T }));
    if (points.length) {
      // area
      ctx.beginPath();
      points.forEach((p, i) => {
        const xx = mapT(p.t), yy = mapY(p.v);
        if (i === 0) ctx.moveTo(xx, yy); else ctx.lineTo(xx, yy);
      });
      const lastX = mapT(points[points.length - 1].t);
      const firstX = mapT(points[0].t);
      ctx.lineTo(lastX, y1); ctx.lineTo(firstX, y1); ctx.closePath();
      ctx.fillStyle = li === 0 ? 'rgba(112,216,229,.07)' : li === 2 ? 'rgba(255,171,94,.05)' : 'rgba(185,197,207,.035)';
      ctx.fill();
      ctx.beginPath();
      points.forEach((p, i) => {
        const xx = mapT(p.t), yy = mapY(p.v);
        if (i === 0) ctx.moveTo(xx, yy); else ctx.lineTo(xx, yy);
      });
      ctx.strokeStyle = lane.color;
      ctx.globalAlpha = 0.9;
      ctx.lineWidth = large ? 1.5 : 1.1;
      ctx.stroke();
      ctx.globalAlpha = 1;
      // live tip
      const px = mapT(now), py = mapY(lane.val);
      ctx.fillStyle = lane.color;
      ctx.beginPath(); ctx.arc(px, py, large ? 3 : 2, 0, TAU); ctx.fill();
      ctx.globalAlpha = 0.25;
      ctx.beginPath(); ctx.arc(px, py, large ? 7 : 5, 0, TAU); ctx.fill();
      ctx.globalAlpha = 1;
    }
    // labels
    if (large) {
      ctx.fillStyle = lane.color;
      ctx.fillRect(16, y0 + 9, 10, 2);
      zh(ctx, 10, lane.color, 500, 'left');
      ctx.fillText(lane.name, 34, y0 + 12);
      mono(ctx, 17, C.text, 500, 'left');
      ctx.fillText(lane.val.toFixed(lane.key === 'T' ? 1 : lane.key === 'n' ? 0 : 2), 16, y0 + 38);
      mono(ctx, 8, C.dim, 500, 'left');
      ctx.fillText(lane.unit, 17, y0 + 54);
    } else {
      zh(ctx, 7, lane.color, 500, 'left');
      ctx.fillText(lane.key, 8, y0 + 8);
      mono(ctx, 7.5, C.text, 500, 'right');
      ctx.fillText(lane.val.toFixed(lane.key === 'T' ? 1 : lane.key === 'n' ? 0 : 2), w - 8, y0 + 8);
    }
  });
  // vertical grid and time labels
  const cols = large ? 6 : 3;
  for (let i = 0; i <= cols; i++) {
    const x = lerp(plot.x0, plot.x1, i / cols);
    if (i > 0 && i < cols) vline(ctx, x, plot.y0, plot.y1, C.line2);
    if (i === 0 || i === cols || large) {
      const seconds = Math.round(-range + range * i / cols);
      mono(ctx, large ? 8 : 6.5, C.dim, 500, i === 0 ? 'left' : i === cols ? 'right' : 'center');
      ctx.fillText(seconds === 0 ? 'NOW' : `${seconds}s`, x, h - pad.b + (large ? 11 : 8));
    }
  }
  if (large) {
    mono(ctx, 8, C.muted, 500, 'left');
    ctx.fillText('50 Hz SAMPLING · 100 PTS / 2 s · DISPLAY WINDOW', 13, 13);
    const legend = [['I', C.cyan], ['n', C.steel], ['T', C.amber]];
    let lx = w - 18;
    for (let i = legend.length - 1; i >= 0; i--) {
      mono(ctx, 8, legend[i][1], 500, 'right');
      ctx.fillText(legend[i][0], lx, 13);
      lx -= 17;
    }
  }
  if (opts.hoverX != null && opts.hoverX >= 0 && opts.hoverX <= 1) {
    const hx = plot.x0 + opts.hoverX * (plot.x1 - plot.x0);
    const ht = t0 + opts.hoverX * range;
    let nearest = null, nd = Infinity;
    for (const p of data) {
      const d = Math.abs(p.t - ht);
      if (d < nd) { nd = d; nearest = p; }
    }
    vline(ctx, hx, plot.y0, plot.y1, 'rgba(232,237,242,.42)', 1);
    if (nearest) {
      mono(ctx, large ? 8 : 7, C.text, 500, hx > w * 0.7 ? 'right' : 'left');
      ctx.fillText(`t ${nearest.t.toFixed(1)} s`, hx + (hx > w * 0.7 ? -6 : 6), plot.y0 + 6);
      mono(ctx, large ? 8 : 7, C.cyan, 500, hx > w * 0.7 ? 'right' : 'left');
      ctx.fillText(`I ${nearest.I.toFixed(2)} A  n ${nearest.n.toFixed(0)}  T ${nearest.T.toFixed(1)}℃`, hx + (hx > w * 0.7 ? -6 : 6), plot.y0 + (large ? 18 : 15));
    }
  }
}

function drawTrendCompact(ctx, w, h, opts) {
  const plot = { x0: 9, x1: w - 9, y0: 17, y1: h - 15 };
  const now = state.simTime;
  const t0 = now - 60;
  const data = state.trend.filter((p) => p.t >= t0);
  const lanes = [
    { key: 'I', color: C.cyan, min: 0, max: 6.5, value: state.fieldOn ? state.currentA : 0, digits: 2, unit: 'A', symbol: 'I' },
    { key: 'n', color: C.steel, min: 0, max: 1600, value: state.nRpm, digits: 0, unit: '', symbol: 'n' },
    { key: 'T', color: C.amber, min: 20, max: 90, value: state.tempC, digits: 1, unit: '℃', symbol: 'T' }
  ];
  const mapX = (t) => plot.x0 + clamp((t - t0) / 60, 0, 1) * (plot.x1 - plot.x0);
  const mapY = (v, l) => plot.y1 - clamp((v - l.min) / (l.max - l.min), 0, 1) * (plot.y1 - plot.y0);
  // Two horizontal guides keep the miniature legible without a full grid.
  hline(ctx, lerp(plot.y0, plot.y1, 0.5), plot.x0, plot.x1, C.line2);
  hline(ctx, plot.y1, plot.x0, plot.x1, C.line2);
  for (const lane of lanes) {
    const pts = data.map((p) => ({ t: p.t, v: p.key === 'I' ? p.I : p.key === 'n' ? p.n : p.T }));
    if (!pts.length) continue;
    ctx.beginPath();
    pts.forEach((p, i) => {
      const x = mapX(p.t), y = mapY(p.v, lane);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = lane.color;
    ctx.lineWidth = 1.12;
    ctx.globalAlpha = lane.key === 'n' ? 0.62 : 0.88;
    ctx.stroke();
    ctx.globalAlpha = 1;
    const lx = mapX(now), ly = mapY(lane.value, lane);
    ctx.fillStyle = lane.color;
    ctx.beginPath(); ctx.arc(lx, ly, 1.9, 0, TAU); ctx.fill();
  }
  // threshold lines
  dashedLine(ctx, plot.x0, mapY(MC.thresholds.currentTripA, lanes[0]), plot.x1, mapY(MC.thresholds.currentTripA, lanes[0]), 'rgba(228,90,82,.28)', [2, 4], 0.8);
  dashedLine(ctx, plot.x0, mapY(MC.thresholds.tempTripC, lanes[2]), plot.x1, mapY(MC.thresholds.tempTripC, lanes[2]), 'rgba(255,171,94,.20)', [2, 4], 0.8);
  // live legend values
  lanes.forEach((lane, i) => {
    const x = 9 + i * (w - 18) / 3;
    ctx.fillStyle = lane.color;
    ctx.fillRect(x, 5, 5, 1.6);
    mono(ctx, 7.2, C.steel, 500, 'left');
    const label = `${lane.symbol} ${lane.value.toFixed(lane.digits)}${lane.unit}`;
    ctx.fillText(label, x + 9, 6.6);
  });
  mono(ctx, 6, C.dim, 500, 'left'); ctx.fillText('−60s', plot.x0, h - 5);
  mono(ctx, 6, C.dim, 500, 'right'); ctx.fillText('NOW', plot.x1, h - 5);
  if (opts.hoverX != null) {
    const hx = plot.x0 + clamp(opts.hoverX, 0, 1) * (plot.x1 - plot.x0);
    vline(ctx, hx, plot.y0, plot.y1, 'rgba(232,237,242,.4)');
  }
}

// ---- torque–slip ----
function drawTorqueCard(ctx, w, h, opts = {}) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const pad = { l: large ? 48 : 26, r: large ? 18 : 9, t: large ? 26 : 9, b: large ? 30 : 15 };
  const plot = { x0: pad.l, x1: w - pad.r, y0: pad.t, y1: h - pad.b };
  const maxT = 18;
  const xMap = (s) => plot.x0 + clamp(s, 0, 1) * (plot.x1 - plot.x0);
  const yMap = (T) => plot.y1 - clamp(T / maxT, 0, 1) * (plot.y1 - plot.y0);
  // grid
  for (let i = 0; i <= 5; i++) {
    const x = lerp(plot.x0, plot.x1, i / 5);
    vline(ctx, x, plot.y0, plot.y1, C.line2);
    if (large) axisLabel(ctx, (i / 5).toFixed(1), x, plot.y1 + 12, C.dim, 'center', 7.5);
  }
  for (let i = 0; i <= 4; i++) {
    const y = lerp(plot.y0, plot.y1, i / 4);
    hline(ctx, y, plot.x0, plot.x1, C.line2);
    if (large) axisLabel(ctx, (maxT - i / 4 * maxT).toFixed(0), plot.x0 - 7, y, C.dim, 'right', 7);
  }
  // shaded under-curve
  const curve = [];
  for (let i = 0; i <= 150; i++) {
    const s = i / 150;
    curve.push({ s, T: torqueAtSlip(s) });
  }
  ctx.beginPath();
  curve.forEach((p, i) => { const x = xMap(p.s), y = yMap(p.T); if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
  ctx.lineTo(plot.x1, plot.y1); ctx.lineTo(plot.x0, plot.y1); ctx.closePath();
  const grd = ctx.createLinearGradient(0, plot.y0, 0, plot.y1);
  grd.addColorStop(0, 'rgba(112,216,229,.12)');
  grd.addColorStop(1, 'rgba(112,216,229,0)');
  ctx.fillStyle = grd; ctx.fill();
  ctx.beginPath();
  curve.forEach((p, i) => { const x = xMap(p.s), y = yMap(p.T); if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
  ctx.strokeStyle = C.cyan; ctx.lineWidth = large ? 1.8 : 1.2; ctx.stroke();

  // rated torque horizontal
  dashedLine(ctx, plot.x0, yMap(MC.ratedTorqueNm), plot.x1, yMap(MC.ratedTorqueNm), 'rgba(185,197,207,.18)', [3, 5], 1);
  if (large) axisLabel(ctx, 'Tn 7.5', plot.x1 - 4, yMap(MC.ratedTorqueNm) - 7, C.dim, 'right', 6.5);
  // reference markers
  const marks = [
    { s: 0, T: 0, label: 'n₁ · 同步', color: C.cyan, align: 'left' },
    { s: MC.ratedSlip, T: MC.ratedTorqueNm, label: '额定点', color: C.steel, align: 'right' },
    { s: MC.breakdownSlip, T: MC.maxTorqueFactor * MC.ratedTorqueNm, label: '最大转矩', color: C.amber, align: 'left' },
    { s: 1, T: MC.startTorqueFactor * MC.ratedTorqueNm, label: '起动', color: C.muted, align: 'right' }
  ];
  for (const m of marks) {
    const x = xMap(m.s), y = yMap(m.T);
    ctx.fillStyle = m.color;
    ctx.beginPath(); ctx.arc(x, y, large ? 2.8 : 1.8, 0, TAU); ctx.fill();
    canvasTag(ctx, m.label, x + (m.align === 'right' ? -5 : 5), y - 10, m.color, m.align, large ? 8 : 6.5);
  }
  // slip reference at rated
  const ratedX = xMap(MC.ratedSlip);
  dashedLine(ctx, ratedX, plot.y0, ratedX, plot.y1, 'rgba(185,197,207,.13)', [2, 4], 1);
  // current work point
  const workSlip = clamp(state.slip, 0, 1);
  const workT = state.fieldOn ? torqueAtSlip(workSlip) : 0;
  const wx = xMap(workSlip), wy = yMap(workT);
  if (state.fieldOn) {
    ctx.save();
    ctx.fillStyle = C.amber;
    ctx.shadowColor = 'rgba(255,171,94,.7)';
    ctx.shadowBlur = large ? 13 : 6;
    ctx.beginPath(); ctx.arc(wx, wy, large ? 4.5 : 2.8, 0, TAU); ctx.fill();
    ctx.restore();
    ctx.strokeStyle = 'rgba(255,171,94,.55)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(wx, wy, large ? 10 : 6, 0, TAU); ctx.stroke();
    if (large) {
      axisLabel(ctx, `工作点 s=${workSlip.toFixed(3)} · T=${workT.toFixed(2)} N·m`, clamp(wx + 12, plot.x0 + 5, plot.x1 - 190), clamp(wy - 15, plot.y0 + 8, plot.y1 - 8), C.amber, 'left', 8);
    }
  }
  if (large) {
    zh(ctx, 8, C.dim, 500, 'left');
    ctx.fillText('SLIP s = (n₁ − n) / n₁', plot.x0, h - 8);
    mono(ctx, 8, C.dim, 500, 'right');
    ctx.fillText('T / N·m', 13, plot.y0 + 2);
    axisLabel(ctx, 's = 0.00', plot.x0, plot.y1 + 15, C.dim, 'left', 7);
    axisLabel(ctx, 's = 1.00 · 堵转', plot.x1, plot.y1 + 15, C.red, 'right', 7);
    // small start/current labels
    mono(ctx, 7, C.muted, 500, 'left');
    ctx.fillText('T = 9550 P / n', 13, 13);
  } else {
    mono(ctx, 6.5, C.dim, 500, 'right');
    ctx.fillText('T–s', w - 8, 9);
  }
  if (opts.hoverX != null && opts.hoverX >= 0 && opts.hoverX <= 1) {
    const s = opts.hoverX;
    const x = xMap(s), y = yMap(torqueAtSlip(s));
    vline(ctx, x, plot.y0, plot.y1, 'rgba(232,237,242,.4)');
    ctx.fillStyle = C.text;
    ctx.beginPath(); ctx.arc(x, y, 2.8, 0, TAU); ctx.fill();
    canvasTag(ctx, `s ${s.toFixed(3)} / ${torqueAtSlip(s).toFixed(2)} N·m`, clamp(x + 7, plot.x0 + 4, plot.x1 - 140), clamp(y - 13, plot.y0 + 8, plot.y1 - 8), C.text, 'left', large ? 8 : 7);
  }
}
function canvasTag(ctx, text, x, y, color = C.muted, align = 'left', size = 8) {
  mono(ctx, size, color, 500, align);
  ctx.fillText(text, x, y);
}

// ---- SVM ----
function drawSvmCard(ctx, w, h, opts = {}) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const pad = { l: large ? 48 : 27, r: large ? 18 : 10, t: large ? 25 : 9, b: large ? 31 : 16 };
  const plot = { x0: pad.l, x1: w - pad.r, y0: pad.t, y1: h - pad.b };
  const xMin = 1.9, xMax = 6.3, yMin = -4.2, yMax = 1.5;
  const xMap = (x) => plot.x0 + clamp((x - xMin) / (xMax - xMin), 0, 1) * (plot.x1 - plot.x0);
  const yMap = (y) => plot.y1 - clamp((y - yMin) / (yMax - yMin), 0, 1) * (plot.y1 - plot.y0);
  // decision background heat cells
  const cols = large ? 22 : 12, rows = large ? 14 : 8;
  for (let i = 0; i < cols; i++) {
    for (let j = 0; j < rows; j++) {
      const x = xMin + (i + 0.5) / cols * (xMax - xMin);
      const y = yMax - (j + 0.5) / rows * (yMax - yMin);
      const f = svmModel.rawScore(x, y);
      if (f > 0) {
        ctx.fillStyle = `rgba(228,90,82,${0.018 + Math.min(0.055, f * 0.014)})`;
        const cw = (plot.x1 - plot.x0) / cols + 0.7;
        const ch = (plot.y1 - plot.y0) / rows + 0.7;
        ctx.fillRect(xMap(x - (xMax - xMin) / cols / 2), yMap(y + (yMax - yMin) / rows / 2), cw, ch);
      }
    }
  }
  // grid
  for (let i = 0; i <= 5; i++) {
    const x = lerp(plot.x0, plot.x1, i / 5);
    vline(ctx, x, plot.y0, plot.y1, C.line2);
    if (large) axisLabel(ctx, (xMin + i / 5 * (xMax - xMin)).toFixed(1), x, plot.y1 + 12, C.dim, 'center', 7);
  }
  for (let i = 0; i <= 4; i++) {
    const y = lerp(plot.y0, plot.y1, i / 4);
    hline(ctx, y, plot.x0, plot.x1, C.line2);
    if (large) axisLabel(ctx, (yMax - i / 4 * (yMax - yMin)).toFixed(1), plot.x0 - 7, y, C.dim, 'right', 7);
  }
  // decision line + margins, clipped
  ctx.save();
  ctx.beginPath(); ctx.rect(plot.x0, plot.y0, plot.x1 - plot.x0, plot.y1 - plot.y0); ctx.clip();
  const drawKernelLine = (level, color, dash, width) => {
    const w0 = svmModel.weights[0], w1 = svmModel.weights[1], b = svmModel.bias;
    const pts = [];
    if (Math.abs(w1) > 1e-5) {
      for (let i = 0; i <= 80; i++) {
        const x = xMin + (xMax - xMin) * i / 80;
        const zx = (x - 2.55) / 1.05;
        const y = ((level - b - w0 * zx) / w1) * 2.2;
        pts.push({ x, y });
      }
      ctx.save(); ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = width;
      ctx.beginPath();
      let started = false;
      for (const p of pts) {
        const px = xMap(p.x), py = yMap(p.y);
        if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
      }
      ctx.stroke(); ctx.restore();
    } else {
      const x = 2.55 + (level - b) * 1.05 / w0;
      vline(ctx, xMap(x), plot.y0, plot.y1, color, width);
    }
  };
  drawKernelLine(0, 'rgba(228,90,82,.68)', [5, 4], large ? 1.5 : 1);
  drawKernelLine(1, 'rgba(228,90,82,.23)', [3, 5], 1);
  drawKernelLine(-1, 'rgba(112,216,229,.22)', [3, 5], 1);
  ctx.restore();
  // training points
  for (const p of svmModel.training) {
    if (p.x < xMin || p.x > xMax || p.y < yMin || p.y > yMax) continue;
    const x = xMap(p.x), y = yMap(p.y);
    ctx.fillStyle = p.label > 0 ? 'rgba(228,90,82,.55)' : 'rgba(112,216,229,.6)';
    ctx.beginPath(); ctx.arc(x, y, large ? 2.2 : 1.4, 0, TAU); ctx.fill();
  }
  // support vectors
  for (const p of svmModel.support) {
    if (p.x < xMin || p.x > xMax || p.y < yMin || p.y > yMax) continue;
    const x = xMap(p.x), y = yMap(p.y);
    ctx.strokeStyle = p.label > 0 ? 'rgba(255,171,94,.72)' : 'rgba(131,211,164,.7)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(x, y, large ? 5 : 3.2, 0, TAU); ctx.stroke();
  }
  // current sample
  const cf = state.feature;
  const active = state.fieldOn && state.sinceStart > 3.8;
  const clampX = clamp(cf.currentMean, xMin, xMax);
  const clampY = clamp(cf.speedRate, yMin, yMax);
  const cx = xMap(clampX), cy = yMap(clampY);
  if (active) {
    const overload = !state.svmNormal && state.fault === 'overload';
    const markerColor = state.svmNormal ? C.cyan : overload ? C.amber : C.red;
    ctx.save();
    ctx.fillStyle = markerColor;
    ctx.shadowColor = state.svmNormal ? 'rgba(112,216,229,.8)' : overload ? 'rgba(255,171,94,.8)' : 'rgba(228,90,82,.8)';
    ctx.shadowBlur = large ? 16 : 7;
    ctx.save();
    ctx.translate(cx, cy); ctx.rotate(Math.PI / 4);
    const sz = large ? 4.4 : 3.2;
    ctx.fillRect(-sz / 2, -sz / 2, sz, sz);
    ctx.restore();
    ctx.lineWidth = 1;
    ctx.strokeStyle = state.svmNormal ? 'rgba(112,216,229,.4)' : overload ? 'rgba(255,171,94,.5)' : 'rgba(228,90,82,.5)';
    ctx.beginPath(); ctx.arc(cx, cy, large ? 11 : 7, 0, TAU); ctx.stroke();
    ctx.restore();
    if (large) {
      canvasTag(ctx, `CURRENT · I ${cf.currentMean.toFixed(2)} A / ${cf.speedRate.toFixed(2)} rpm·s⁻¹`, clamp(cx + 13, plot.x0 + 4, plot.x1 - 260), clamp(cy - 15, plot.y0 + 8, plot.y1 - 8), markerColor, 'left', 8);
    }
  }
  // axis labels + legend
  if (large) {
    mono(ctx, 8, C.dim, 500, 'center');
    ctx.fillText('电流均值 Iₘ / A', (plot.x0 + plot.x1) / 2, h - 9);
    ctx.save(); ctx.translate(13, (plot.y0 + plot.y1) / 2); ctx.rotate(-Math.PI / 2);
    mono(ctx, 8, C.dim, 500, 'center'); ctx.fillText('转速变化率 dn/dt / rpm·s⁻¹', 0, 0); ctx.restore();
    mono(ctx, 8, C.muted, 500, 'left');
    ctx.fillText(`RBF · 2D PROJECTION   SUPPORT VECTORS ${svmModel.support.length}   ACC ${(95.6).toFixed(1)}%`, 13, 13);
    // legend
    const lx = w - 18;
    mono(ctx, 8, C.dim, 500, 'right'); ctx.fillText('HEALTHY', lx - 13, 13);
    ctx.fillStyle = C.cyan; ctx.beginPath(); ctx.arc(lx - 92, 13, 2.4, 0, TAU); ctx.fill();
    mono(ctx, 8, C.dim, 500, 'right'); ctx.fillText('FAULT', lx - 130, 13);
    ctx.fillStyle = C.red; ctx.beginPath(); ctx.arc(lx - 160, 13, 2.4, 0, TAU); ctx.fill();
    // verdict
    if (active) {
      const verdict = state.svmNormal ? '正常类' : (state.faultLabel || '故障类');
      mono(ctx, 10, state.svmNormal ? C.green : C.red, 600, 'right');
      ctx.fillText(`${verdict} · ${(state.svmConfidence * 100).toFixed(0)}%`, w - 18, plot.y0 + 3);
    }
  } else {
    mono(ctx, 6.5, C.dim, 500, 'left'); ctx.fillText('SVM', 8, 8);
    mono(ctx, 6.5, state.svmNormal ? C.green : C.red, 500, 'right'); ctx.fillText(active ? `${state.svmNormal ? 'N' : 'F'} ${(state.svmConfidence * 100).toFixed(0)}%` : 'IDLE', w - 8, 8);
  }
  if (opts.hoverX != null && opts.hoverY != null) {
    // nearest training sample for inspection
    const xr = xMin + opts.hoverX * (xMax - xMin);
    const yr = yMax - opts.hoverY * (yMax - yMin);
    let best = null, bd = Infinity;
    for (const p of svmModel.training) {
      const d = (p.x - xr) ** 2 + ((p.y - yr) / 2) ** 2;
      if (d < bd) { bd = d; best = p; }
    }
    if (best) {
      const bx = xMap(best.x), by = yMap(best.y);
      ctx.strokeStyle = C.text; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(bx, by, large ? 7 : 5, 0, TAU); ctx.stroke();
      canvasTag(ctx, `${best.label > 0 ? 'FAULT' : 'NORMAL'} · I ${best.x.toFixed(2)} · dn/dt ${best.y.toFixed(2)}`, clamp(bx + 9, plot.x0 + 2, plot.x1 - 205), clamp(by - 12, plot.y0 + 9, plot.y1 - 5), C.text, 'left', large ? 8 : 6.5);
    }
  }
}

// ---- digital gauge ----
function drawGaugeCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const padX = large ? 34 : 15;
  const top = large ? 46 : 25;
  const gap = large ? 22 : 10;
  const laneH = (h - top - (large ? 32 : 18) - gap * 3) / 4;
  const gauges = [
    { name: '定子线电流 I', en: 'STATOR CURRENT', value: state.fieldOn ? state.currentA : 0, unit: 'A', min: 0, max: 7, warn: MC.thresholds.currentWarnA, trip: MC.thresholds.currentTripA, rated: MC.ratedCurrentA, format: 2, color: C.cyan },
    { name: '转子转速 n', en: 'ROTOR SPEED', value: state.nRpm, unit: 'r/min', min: 0, max: 1800, warn: MC.thresholds.speedWarnRpm, trip: MC.thresholds.speedTripRpm, rated: MC.ratedSpeedRpm, sync: 1500, format: 0, color: C.steel },
    { name: '壳体温度 T', en: 'FRAME TEMP.', value: state.tempC, unit: '℃', min: 20, max: 100, warn: MC.thresholds.tempWarnC, trip: MC.thresholds.tempTripC, rated: MC.steadyCaseC, format: 1, color: C.amber },
    { name: '输出转矩', en: 'OUTPUT TORQUE', value: state.torqueNm, unit: 'N·m', min: 0, max: 17, warn: 14, trip: 16.5, rated: MC.ratedTorqueNm, format: 2, color: C.cyan }
  ];
  if (large) {
    mono(ctx, 8, C.muted, 500, 'left'); ctx.fillText('DIGITAL INSTRUMENTS / 量程 · 阈值 · 额定对比', padX, 16);
    mono(ctx, 8, C.dim, 500, 'right'); ctx.fillText('LIVE · 50 Hz SOURCE', w - padX, 16);
  }
  gauges.forEach((g, i) => {
    const y = top + i * (laneH + gap);
    const barY = y + laneH * 0.56;
    const barH = Math.max(4, laneH * 0.17);
    const x0 = padX, x1 = w - padX;
    const map = (v) => x0 + clamp((v - g.min) / (g.max - g.min), 0, 1) * (x1 - x0);
    zh(ctx, large ? 10 : 8, C.muted, 500, 'left'); ctx.fillText(g.name, x0, y + 7);
    mono(ctx, large ? 23 : 14, C.text, 500, 'right');
    ctx.font = `500 ${large ? 23 : 14}px ${CANVAS_MONO}`;
    ctx.fillText(g.value.toFixed(g.format), x1, y + laneH * 0.22);
    const vw = ctx.measureText(g.value.toFixed(g.format)).width;
    mono(ctx, large ? 9 : 7, C.dim, 500, 'right'); ctx.fillText(g.unit, x1 - vw - 4, y + laneH * 0.22 + (large ? 3 : 1));
    // scale
    ctx.fillStyle = '#1a252f'; ctx.fillRect(x0, barY, x1 - x0, barH);
    const pct = clamp((g.value - g.min) / (g.max - g.min), 0, 1);
    const color = g.value > (g.trip ?? Infinity) ? C.red : g.value > (g.warn ?? Infinity) ? C.amber : g.color;
    ctx.fillStyle = color; ctx.fillRect(x0, barY, (x1 - x0) * pct, barH);
    // threshold ticks
    if (Number.isFinite(g.warn)) {
      ctx.fillStyle = C.amber; ctx.fillRect(map(g.warn) - 0.75, barY - 3, 1.5, barH + 6);
    }
    if (Number.isFinite(g.trip)) {
      ctx.fillStyle = C.red; ctx.fillRect(map(g.trip) - 0.75, barY - 4, 1.5, barH + 8);
    }
    if (Number.isFinite(g.rated)) {
      ctx.fillStyle = 'rgba(232,237,242,.6)'; ctx.fillRect(map(g.rated) - 0.6, barY - 2, 1.2, barH + 4);
    }
    if (g.sync) {
      ctx.fillStyle = 'rgba(112,216,229,.55)'; ctx.fillRect(map(g.sync) - 0.6, barY - 2, 1.2, barH + 4);
    }
    if (large) {
      mono(ctx, 7, C.dim, 500, 'left'); ctx.fillText(g.en, x0, y + 21);
      mono(ctx, 7, C.dim, 500, 'right');
      ctx.fillText(`${g.min}`, x0 + 1, barY + barH + 12);
      ctx.fillText(`${g.max}`, x1 - 1, barY + barH + 12);
      if (Number.isFinite(g.rated)) {
        mono(ctx, 7, C.muted, 500, 'center'); ctx.fillText(`额定 ${g.rated}`, map(g.rated), barY + barH + 12);
      }
      if (Number.isFinite(g.trip)) {
        mono(ctx, 7, C.red, 500, 'center'); ctx.fillText(`阈值 ${g.trip}`, map(g.trip), barY - 8);
      }
    }
    if (i < 3) hline(ctx, y + laneH + gap / 2, x0, x1, C.line2);
  });
}

// ---- confusion matrix ----
function drawConfusionCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const pad = large ? 42 : 20;
  const headerY = large ? 48 : 24;
  if (large) {
    mono(ctx, 8, C.muted, 500, 'left'); ctx.fillText('CONFUSION MATRIX / 教学验证集', pad, 17);
    mono(ctx, 8, C.dim, 500, 'right'); ctx.fillText('92 SAMPLES · BINARY 0/1', w - pad, 17);
  }
  const matrix = [[44, 2], [2, 44]];
  const total = 92;
  const accuracy = 88 / total;
  const maxV = 44;
  const cellW = large ? Math.min(160, (w - pad * 2 - 120) / 2) : 47;
  const cellH = large ? 120 : 36;
  const gridX = large ? pad + 96 : pad + 19;
  const gridY = large ? headerY + 28 : headerY + 11;
  const labelsX = ['预测正常', '预测故障'];
  const labelsY = ['实际正常', '实际故障'];
  const colors = ['rgba(112,216,229,', 'rgba(228,90,82,'];
  for (let r = 0; r < 2; r++) {
    for (let c = 0; c < 2; c++) {
      const v = matrix[r][c];
      const x = gridX + c * (cellW + 2);
      const y = gridY + r * (cellH + 2);
      ctx.fillStyle = `${colors[c]}${0.06 + (v / maxV) * (c === r ? 0.25 : 0.10)})`;
      ctx.fillRect(x, y, cellW, cellH);
      ctx.strokeStyle = 'rgba(160,190,208,.16)'; ctx.strokeRect(x + 0.5, y + 0.5, cellW - 1, cellH - 1);
      mono(ctx, large ? 32 : 12, c === r ? C.text : C.red, 500, 'center');
      ctx.fillText(String(v), x + cellW / 2, y + cellH / 2 + (large ? 5 : 1));
      if (large) {
        mono(ctx, 7, C.dim, 500, 'center');
        ctx.fillText(c === r ? 'CORRECT' : 'ERROR', x + cellW / 2, y + cellH - 14);
      }
    }
  }
  if (large) {
    zh(ctx, 9, C.dim, 500, 'center');
    ctx.fillText(labelsX[0], gridX + cellW / 2, gridY - 15);
    ctx.fillText(labelsX[1], gridX + cellW * 1.5 + 2, gridY - 15);
    zh(ctx, 9, C.dim, 500, 'right');
    ctx.fillText(labelsY[0], gridX - 16, gridY + cellH / 2);
    ctx.fillText(labelsY[1], gridX - 16, gridY + cellH * 1.5 + 2);
    // metrics
    const mx = gridX + cellW * 2 + 48;
    const my = gridY + 9;
    mono(ctx, 8, C.muted, 500, 'left'); ctx.fillText('ACCURACY', mx, my);
    mono(ctx, 38, C.green, 500, 'left'); ctx.fillText(`${(accuracy * 100).toFixed(1)}%`, mx, my + 35);
    mono(ctx, 8, C.dim, 500, 'left'); ctx.fillText(`TP 44 · TN 44 · FP 2 · FN 2`, mx, my + 65);
    mono(ctx, 8, C.dim, 500, 'left'); ctx.fillText(`PRECISION ${(44 / 46 * 100).toFixed(1)}%`, mx, my + 83);
    mono(ctx, 8, C.dim, 500, 'left'); ctx.fillText(`RECALL ${(44 / 46 * 100).toFixed(1)}%`, mx, my + 101);
    zh(ctx, 8, C.dim, 500, 'left');
    ctx.fillText('注：混淆矩阵为课程 SVM 的教学校准结果，不代表工业验证。', pad, h - 16);
  } else {
    mono(ctx, 6.5, C.dim, 500, 'right');
    ctx.fillText(`ACC ${(accuracy * 100).toFixed(1)}%`, w - 10, 9);
  }
}

// ---- diagnosis flow ----
function drawFlowCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const steps = [
    { t: '信号采集', en: 'ACQUIRE', v: '5 SENSORS / 50 Hz', color: C.cyan },
    { t: '预处理', en: 'FILTER', v: '低通 / 归一化', color: C.cyan },
    { t: '特征提取', en: 'FEATURES', v: '6 FEATURES / 2 s', color: C.cyan },
    { t: 'SVM 判决', en: 'SVM', v: `f(x) ${state.svmScore >= 0 ? '+' : '−'}${Math.abs(state.svmScore).toFixed(2)}`, color: state.svmNormal ? C.green : C.red },
    { t: '结论输出', en: 'VERDICT', v: state.svmNormal ? '正常类' : (state.faultLabel || '故障类'), color: state.svmNormal ? C.green : C.red },
    { t: '报警联动', en: 'ACTION', v: state.alarm ? (state.alarm.executed ? '保护停机' : '保护延时') : '监测中', color: state.alarm ? C.red : C.dim }
  ];
  const active = state.alarm ? 5 : Math.floor((state.simTime % 2.4) / 0.4);
  const pad = large ? 26 : 10;
  const top = large ? 52 : 26;
  const boxH = large ? 98 : Math.max(46, h * 0.36);
  const gap = large ? 12 : 3.5;
  const boxW = (w - pad * 2 - gap * 5) / 6;
  if (large) {
    mono(ctx, 8, C.muted, 500, 'left'); ctx.fillText('DIAGNOSTIC PIPELINE / 2 s SLIDING WINDOW', pad, 18);
    mono(ctx, 8, C.dim, 500, 'right'); ctx.fillText('SIGNAL → FEATURE → SVM → ACTION', w - pad, 18);
  }
  steps.forEach((s, i) => {
    const x = pad + i * (boxW + gap);
    const y = top;
    const isActive = i === active;
    const isDone = i < active;
    ctx.fillStyle = isActive ? 'rgba(112,216,229,.07)' : isDone ? 'rgba(255,255,255,.02)' : 'rgba(255,255,255,.01)';
    ctx.fillRect(x, y, boxW, boxH);
    ctx.strokeStyle = isActive ? 'rgba(112,216,229,.45)' : 'rgba(160,190,208,.10)';
    ctx.lineWidth = 1;
    ctx.strokeRect(x + 0.5, y + 0.5, boxW - 1, boxH - 1);
    if (isActive) {
      ctx.fillStyle = s.color;
      ctx.fillRect(x, y, boxW, 2);
    }
    mono(ctx, large ? 7 : 5.5, isActive ? C.cyan : C.dim, 500, 'left');
    ctx.fillText(`0${i + 1}`, x + (large ? 10 : 4), y + (large ? 12 : 7));
    zh(ctx, large ? 10 : 7.2, isActive ? C.text : C.muted, 500, 'left');
    ctx.fillText(s.t, x + (large ? 10 : 4), y + (large ? 30 : 17));
    if (large) {
      mono(ctx, 7, isActive ? s.color : C.dim, 500, 'left');
      ctx.fillText(s.en, x + 10, y + 47);
      zh(ctx, 7.5, isActive ? C.steel : C.dim, 500, 'left');
      ctx.fillText(s.v, x + 10, y + 68);
      mono(ctx, 6.5, C.dim, 500, 'left');
      ctx.fillText(i === 0 ? 'TH-01 … PH-05' : i === 2 ? 'Ī / σ² / K / dn/dt / err / dT/dt' : '', x + 10, y + 84);
    }
    if (i < 5) {
      ctx.strokeStyle = isActive ? 'rgba(112,216,229,.45)' : 'rgba(160,190,208,.12)';
      ctx.lineWidth = 1;
      const ax = x + boxW + 1.5, ay = y + boxH / 2;
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(ax + gap - 3, ay); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(ax + gap - 5, ay - 2.5); ctx.lineTo(ax + gap - 2, ay); ctx.lineTo(ax + gap - 5, ay + 2.5); ctx.stroke();
    }
  });
  if (!large) {
    const footY = Math.min(h - 8, top + boxH + 22);
    mono(ctx, 6, C.dim, 500, 'left');
    ctx.fillText(`f(x) ${state.svmScore >= 0 ? '+' : '−'}${Math.abs(state.svmScore).toFixed(2)} · ${state.svmNormal ? 'NORMAL' : (state.faultLabel || 'FAULT')} · ${(state.svmConfidence * 100).toFixed(0)}%`, pad, footY);
  }
  if (large) {
    const footY = top + boxH + 34;
    hline(ctx, footY - 14, pad, w - pad, C.line2);
    // live features row
    const feats = [
      ['电流均值', state.feature.currentMean.toFixed(2), 'A'],
      ['电流方差', state.feature.currentVar.toFixed(4), ''],
      ['电流峭度', state.feature.currentKurt.toFixed(2), ''],
      ['转速变化率', state.feature.speedRate.toFixed(2), 'rpm/s'],
      ['跟随误差', state.feature.followError.toFixed(1), 'r/min'],
      ['温度斜率', `${state.feature.tempSlope >= 0 ? '+' : ''}${state.feature.tempSlope.toFixed(2)}`, '℃/s']
    ];
    const cellW = (w - pad * 2) / 6;
    feats.forEach((f, i) => {
      const x = pad + i * cellW;
      mono(ctx, 7, C.dim, 500, 'left'); ctx.fillText(f[0], x, footY);
      mono(ctx, 12, i === 3 || i === 5 ? C.amber : C.steel, 500, 'left'); ctx.fillText(f[1], x, footY + 19);
      mono(ctx, 6.5, C.dim, 500, 'left'); ctx.fillText(f[2], x + (f[1].length * 7 + 4), footY + 19);
    });
    mono(ctx, 8, C.dim, 500, 'left');
    ctx.fillText(state.alarm ? `ALARM · ${state.alarm.text} · ${state.alarm.executed ? '保护停机' : '保护延时 ' + state.alarm.countdown.toFixed(1) + ' s'}` : '无越限 · 连续 15 个采样点消抖后触发保护', pad, h - 13);
  }
}

// ---- features table ----
function drawFeaturesCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const pad = large ? 30 : 12;
  const f = state.feature;
  if (!large) {
    const items = [
      ['电流均值', 'I MEAN', f.currentMean.toFixed(2), 'A', f.currentMean > 3.2],
      ['电流方差', 'I VAR', f.currentVar.toFixed(4), 'A²', f.currentVar > 0.006],
      ['电流峭度', 'KURT', f.currentKurt.toFixed(2), '', f.currentKurt > 4.2],
      ['转速变化率', 'dn/dt', `${f.speedRate >= 0 ? '+' : '−'}${Math.abs(f.speedRate).toFixed(1)}`, 'rpm/s', Math.abs(f.speedRate) > 1.2],
      ['跟随误差', 'ERROR', f.followError.toFixed(1), 'rpm', f.followError > 20],
      ['温度斜率', 'dT/dt', `${f.tempSlope >= 0 ? '+' : '−'}${Math.abs(f.tempSlope).toFixed(2)}`, '°C/s', Math.abs(f.tempSlope) > 0.12]
    ];
    const cols = 3, rows = 2;
    const gx = 8, gy = 22, gap = 4;
    const cw = (w - gx * 2 - gap * (cols - 1)) / cols;
    const ch = (h - gy - 8 - gap * (rows - 1)) / rows;
    mono(ctx, 6.5, C.dim, 500, 'left');
    ctx.fillText('6 FEATURES · 2 s WINDOW', gx, 13);
    items.forEach((it, i) => {
      const x = gx + (i % cols) * (cw + gap);
      const y = gy + Math.floor(i / cols) * (ch + gap);
      ctx.fillStyle = it[4] ? 'rgba(228,90,82,.06)' : 'rgba(255,255,255,.018)';
      ctx.fillRect(x, y, cw, ch);
      ctx.strokeStyle = it[4] ? 'rgba(228,90,82,.24)' : 'rgba(160,190,208,.10)';
      ctx.strokeRect(x + .5, y + .5, cw - 1, ch - 1);
      zh(ctx, 6.3, C.muted, 500, 'left'); ctx.fillText(it[0], x + 5, y + 10);
      mono(ctx, 5.5, C.dim, 500, 'left'); ctx.fillText(it[1], x + 5, y + 20);
      mono(ctx, 11, it[4] ? C.red : C.text, 500, 'right');
      ctx.fillText(it[2], x + cw - 5, y + ch - 12);
      mono(ctx, 5.5, C.dim, 500, 'right');
      ctx.fillText(it[3], x + cw - 5, y + ch - 4);
    });
    return;
  }
  if (large) {
    mono(ctx, 8, C.muted, 500, 'left'); ctx.fillText('SIX FEATURES / 2 s WINDOW · 100 POINTS @ 50 Hz', pad, 17);
    mono(ctx, 8, C.dim, 500, 'right'); ctx.fillText('FEATURE VECTOR x ∈ R⁶', w - pad, 17);
  }
  const rows = [
    { k: 'currentMean', label: '电流均值', en: 'I MEAN', value: f.currentMean.toFixed(3), unit: 'A', band: '1.9–2.5', norm: [1.9, 2.5], scale: [0, 6.5], warn: v => v > 3.2, bad: v => v > 3.5 },
    { k: 'currentVar', label: '电流方差', en: 'I VARIANCE', value: f.currentVar.toFixed(5), unit: 'A²', band: '< 0.002', norm: [0, 0.002], scale: [0, 0.02], warn: v => v > 0.006, bad: v => v > 0.012 },
    { k: 'currentKurt', label: '电流峭度', en: 'I KURTOSIS', value: f.currentKurt.toFixed(3), unit: '', band: '2.5–4.0', norm: [2.5, 4], scale: [0, 9], warn: v => v > 4.2, bad: v => v > 6 },
    { k: 'speedRate', label: '转速变化率', en: 'dn/dt', value: `${f.speedRate >= 0 ? '+' : '−'}${Math.abs(f.speedRate).toFixed(3)}`, unit: 'rpm/s', band: '−0.4–0.4', norm: [-0.4, 0.4], scale: [-4, 4], warn: v => Math.abs(v) > 1.2, bad: v => Math.abs(v) > 3 },
    { k: 'followError', label: '跟随误差', en: 'FOLLOW ERROR', value: f.followError.toFixed(2), unit: 'r/min', band: '< 8', norm: [0, 8], scale: [0, 300], warn: v => v > 20, bad: v => v > 100 },
    { k: 'tempSlope', label: '温度斜率', en: 'dT/dt', value: `${f.tempSlope >= 0 ? '+' : '−'}${Math.abs(f.tempSlope).toFixed(3)}`, unit: '℃/s', band: '−0.03–0.03', norm: [-0.03, 0.03], scale: [-0.3, 0.3], warn: v => Math.abs(v) > 0.08, bad: v => Math.abs(v) > 0.25 }
  ];
  const tableY = large ? 46 : 19;
  const rowH = large ? clamp((h - 118) / 6, 44, 74) : 23;
  const valueRightX = large ? pad + 382 : pad;
  const unitX = large ? valueRightX + 7 : pad;
  const bandX = large ? pad + 500 : pad;
  const barX = large ? pad + 660 : pad;
  if (large) {
    hline(ctx, tableY - 10, pad, w - pad, C.line);
    mono(ctx, 7, C.dim, 500, 'left'); ctx.fillText('FEATURE', pad, tableY);
    mono(ctx, 7, C.dim, 500, 'right'); ctx.fillText('CURRENT VALUE', valueRightX, tableY);
    mono(ctx, 7, C.dim, 500, 'left'); ctx.fillText('HEALTHY BAND', bandX, tableY);
    mono(ctx, 7, C.dim, 500, 'left'); ctx.fillText('RELATIVE LEVEL', barX, tableY);
  }
  rows.forEach((r, i) => {
    const y = tableY + (large ? 36 : 8) + i * rowH;
    const isWarn = r.warn(f[r.k]);
    const isBad = r.bad(f[r.k]);
    const color = isBad ? C.red : isWarn ? C.amber : C.steel;
    if (large) {
      ctx.fillStyle = i % 2 ? 'rgba(255,255,255,.012)' : 'transparent';
      ctx.fillRect(pad, y - rowH / 2, w - pad * 2, rowH - 1);
      hline(ctx, y + rowH / 2 - 2, pad, w - pad, C.line2);
    }
    mono(ctx, large ? 7 : 5.5, C.dim, 500, 'left');
    ctx.fillText(`0${i + 1}`, pad, y);
    zh(ctx, large ? 10 : 7, C.text, 500, 'left');
    ctx.fillText(r.label, pad + (large ? 24 : 10), y - (large ? 7 : 3));
    mono(ctx, large ? 6.8 : 5.2, C.dim, 500, 'left');
    ctx.fillText(r.en, pad + (large ? 24 : 10), y + (large ? 9 : 5));
    mono(ctx, large ? 15 : 8.5, color, 500, 'right');
    ctx.fillText(r.value, valueRightX, y + 1);
    if (large) {
      mono(ctx, 7.5, C.dim, 500, 'left'); ctx.fillText(r.unit, unitX, y + 2);
      mono(ctx, 8.5, C.muted, 500, 'left'); ctx.fillText(r.band, bandX, y + 1);
      const bx = barX, bw = Math.max(60, w - pad - bx - 10), bh = 4;
      ctx.fillStyle = '#1a252f'; ctx.fillRect(bx, y - 2, bw, bh);
      const map = (v) => bx + clamp((v - r.scale[0]) / (r.scale[1] - r.scale[0]), 0, 1) * bw;
      const valuePixel = map(f[r.k]);
      ctx.fillStyle = color; ctx.fillRect(bx, y - 2, Math.max(1, valuePixel - bx), bh);
      const n0 = map(r.norm[0]), n1 = map(r.norm[1]);
      ctx.fillStyle = 'rgba(131,211,164,.28)'; ctx.fillRect(n0, y - 6, Math.max(2, n1 - n0), 1.5);
      mono(ctx, 6.8, color, 500, 'right');
      ctx.fillText(isBad ? 'ALARM' : isWarn ? 'WARN' : 'OK', w - pad, y - 13);
    } else if (isWarn) {
      ctx.fillStyle = color; ctx.fillRect(w - 9, y - 2, 3, 3);
    }
    if (!large && i < 5) hline(ctx, y + rowH / 2, pad, w - pad, C.line2);
  });
  if (large) {
    mono(ctx, 7.5, C.dim, 500, 'left');
    ctx.fillText('说明：六个特征由同一 2 s / 100 点窗口计算；转速变化率与温度斜率使用窗口末端线性回归。', pad, h - 14);
  }
}

// ---- table helpers ----
function drawTable(ctx, w, h, opts) {
  const { headers, rows, widths, top = 46, rowH = 32, pad = 24, title, note, large = true, aligns = [], statusCol = -1, headerColor = C.dim } = opts;
  const totalW = widths.reduce((a, b) => a + b, 0);
  const xs = [];
  let acc = pad;
  for (const ww of widths) { xs.push(acc); acc += ww; }
  if (large) {
    mono(ctx, 7.5, C.muted, 500, 'left'); if (title) ctx.fillText(title, pad, 17);
    if (note) { mono(ctx, 7, C.dim, 500, 'right'); ctx.fillText(note, w - pad, 17); }
  }
  hline(ctx, top - 11, pad, w - pad, C.line);
  headers.forEach((h, i) => {
    mono(ctx, large ? 7 : 5.4, headerColor, 500, aligns[i] || 'left');
    ctx.fillText(h, xs[i] + (aligns[i] === 'right' ? widths[i] - 4 : 0), top);
  });
  rows.forEach((row, ri) => {
    const y = top + (large ? 24 : 12) + ri * rowH;
    if (ri % 2 === 0) {
      ctx.fillStyle = 'rgba(255,255,255,.014)';
      ctx.fillRect(pad, y - (large ? rowH / 2 : 6), totalW, large ? rowH - 2 : 11);
    }
    row.forEach((cell, ci) => {
      const align = aligns[ci] || 'left';
      const isStatus = ci === statusCol;
      if (ci === 0) {
        zh(ctx, large ? 9 : 6.4, C.steel, 500, align);
      } else if (isStatus) {
        const s = String(cell);
        const col = s === 'alarm' || s === 'ALARM' ? C.red : s === 'warn' || s === 'WARN' ? C.amber : s === 'normal' || s === 'OK' ? C.green : C.muted;
        mono(ctx, large ? 7.5 : 5.4, col, 500, align);
        ctx.fillText(s.toUpperCase(), xs[ci] + (align === 'right' ? widths[ci] - 4 : 0), y);
        return;
      } else {
        mono(ctx, large ? 8 : 5.6, C.muted, 500, align);
      }
      ctx.fillText(String(cell), xs[ci] + (align === 'right' ? widths[ci] - 4 : 0), y);
    });
    hline(ctx, y + rowH / 2 - (large ? 2 : 1), pad, w - pad, C.line2);
  });
  if (large && opts.footer) {
    mono(ctx, 7.5, C.dim, 500, 'left');
    ctx.fillText(opts.footer, pad, h - 14);
  }
}

// ---- table cards ----
function drawSensorDetailsCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  if (!large) {
    mono(ctx, 6.5, C.dim, 500, 'left'); ctx.fillText('5 SENSORS / LIVE READINGS', 10, 13);
    const rowH = (h - 30) / 5;
    SENSORS.forEach((s, i) => {
      const r = getSensorReading(s.id);
      const y = 25 + i * rowH + rowH / 2;
      const col = r.status === 'alarm' ? C.red : r.status === 'warn' ? C.amber : C.green;
      mono(ctx, 6, C.dim, 500, 'left'); ctx.fillText(s.code, 10, y);
      zh(ctx, 7, C.muted, 500, 'left'); ctx.fillText(s.name.replace('传感器', '').replace('贴片温度', '温度'), 66, y);
      mono(ctx, 8, col, 500, 'right'); ctx.fillText(`${r.display} ${s.unit}`, w - 20, y);
      ctx.fillStyle = col; ctx.beginPath(); ctx.arc(w - 10, y, 2, 0, TAU); ctx.fill();
      hline(ctx, y + rowH / 2 - 1, 10, w - 10, C.line2);
    });
    return;
  }
  const rows = SENSORS.map((s) => {
    const r = getSensorReading(s.id);
    return [s.code, s.name, `${r.display} ${s.unit}`, s.range.split('·')[0].trim(), r.status, s.position];
  });
  drawTable(ctx, w, h, {
    title: 'SENSOR DETAIL / 五路传感器实时参数',
    note: 'SHARED SOURCE · 50 Hz',
    headers: ['CODE', '传感器 / SENSOR', '实时读数', '量程', 'STATUS', '安装位置'],
    rows,
    widths: [large ? 64 : 36, large ? 150 : 78, large ? 86 : 48, large ? 130 : 72, large ? 66 : 38, large ? 290 : 110],
    top: large ? 54 : 20, rowH: large ? 44 : 17, pad: large ? 26 : 10, large,
    aligns: ['left', 'left', 'right', 'left', 'center', 'left'], statusCol: 4,
    footer: '悬停或点击三维传感器可查看测量原理、精度与异常指向。'
  });
}
function drawSensorLocationsCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const compact = h < 230;
  const rows = SENSORS.map((s) => [s.code, s.name, s.position, s.part, s.range]);
  drawTable(ctx, w, h, {
    title: 'MOUNTING MAP / 测量部位对照表',
    note: 'SENSOR → COMPONENT',
    headers: ['CODE', '传感器', '安装位置', '被测部件', '量程 / 精度'],
    rows,
    widths: compact ? [50, 112, 190, 98, 160] : [68, 150, 245, 130, 200],
    top: compact ? 33 : 54,
    rowH: compact ? 18 : 52,
    pad: compact ? 16 : 26,
    large,
    aligns: ['left', 'left', 'left', 'left', 'left'],
    footer: compact ? null : '测量链：被测部件 → 传感器 → DAQ-01 → 特征提取 → SVM 判决。'
  });
}
function drawNormalParamsCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const rows = [
    ['型号', 'Y90S-4 型三相异步电动机', '额定功率', '1.1 kW'],
    ['额定电压 / 接法', '380 V · △', '额定电流', '2.2 A'],
    ['同步转速', '1500 r/min (4P / 50 Hz)', '额定转速', '1400 r/min'],
    ['额定转差率', 's = 0.0667', '额定转矩', '9550 × 1.1 / 1400 ≈ 7.5 N·m'],
    ['最大转矩 / 起动转矩', '2.2 × Tn / 1.8 × Tn', '最大转矩转差', 's = 0.20'],
    ['功率因数 / 效率', '0.78 / 0.79', '防护 / 工作制', 'IP44 / S1'],
    ['定子 / 转子槽数', '24 / 22', '气隙（示意）', '0.35 mm'],
    ['轴承', '前端 6205-2Z / 后端 6204-2Z', '冷却', '后端径向风扇 + 风罩'],
    ['采样 / 窗口', '50 Hz / 100 点 (2 s)', '环境 / 稳态壳体', '25 ℃ / 55 ℃'],
    ['报警阈值', 'I > 3.5 A · n < 100 r/min · T > 75 ℃', '热时间常数', 'τ ≈ 45 s']
  ];
  if (!large) {
    mono(ctx, 6.5, C.dim, 500, 'left'); ctx.fillText('MOTOR_CONFIG / RATED', 9, 13);
    const compact = [
      ['型号', MC.model, '功率', '1.1 kW'],
      ['电压', '380 V △', '电流', '2.2 A'],
      ['转速', '1400 rpm', '同步', '1500 rpm'],
      ['转矩', '7.5 N·m', 's', '0.0667'],
      ['槽数', '24 / 22', '气隙', '0.35 mm'],
      ['轴承', '6205 / 6204', '防护', 'IP44'],
      ['采样', '50 Hz', '窗口', '100 pt'],
      ['阈值', '3.5 A', '温度', '75 ℃']
    ];
    const l1 = 9, v1 = 50, l2 = 166, v2 = 220;
    compact.forEach((row, i) => {
      const y = 29 + i * 13;
      mono(ctx, 5.6, C.dim, 500, 'left'); ctx.fillText(row[0], l1, y);
      mono(ctx, 6.1, C.steel, 500, 'left'); ctx.fillText(row[1], v1, y);
      mono(ctx, 5.6, C.dim, 500, 'left'); ctx.fillText(row[2], l2, y);
      mono(ctx, 6.1, C.steel, 500, 'left'); ctx.fillText(row[3], v2, y);
      hline(ctx, y + 6, l1, w - 9, C.line2);
    });
    return;
  }
  drawTable(ctx, w, h, {
    title: 'NAMEPLATE REFERENCE / 正常工况参数',
    note: 'MOTOR_CONFIG · SINGLE SOURCE',
    headers: ['项目', '数值', '项目', '数值'],
    rows,
    widths: [130, 255, 138, 250],
    top: 54, rowH: 34, pad: 26, large,
    aligns: ['left', 'left', 'left', 'left'],
    footer: '转差率与输出转矩均由此表派生：s = (1500 − n) / 1500，T = 9550 P / n。'
  });
}
function drawFaultMapCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const compact = h < 230;
  const allRows = [
    ['正常', '无', 'I 2.2 A · n 1400 rpm · T 55 ℃', '六特征正常簇'],
    ['卡滞堵转', '输出轴楔块', 'I 4–6 A · n→0 · s=1.00 · 温度上升', 'I 均值/方差/峭度↑ · dn/dt↑'],
    ['轴承磨损', '前端轴承滚道', '振动 RMS↑ · 周期性冲击 · 电流轻微波动', 'I 峭度↑ · 振动特征↑'],
    ['转子断条', '2 根鼠笼导条', '电流 2sf 脉动 · 转速轻微摆动', 'I 方差/峭度↑ · dn/dt↑'],
    ['绕组过热', '定子绕组匝间', 'T 快速上升 > 75 ℃ · I 略升 · n 略降', 'dT/dt 显著↑'],
    ['过载', '负载侧', 'n 沿 T(s) 下滑 · I 接近 3.5 A', '特征介于正常/故障 · 置信度下降']
  ];
  const rows = compact ? allRows.slice(0, 4) : allRows;
  drawTable(ctx, w, h, {
    title: 'FAULT SIGNATURE MAP / 故障特征对照',
    note: compact ? '+2 CONDITIONS IN ENLARGED VIEW' : 'CAUSE → SIGNAL → FEATURE',
    headers: ['工况', '故障部位', '参数变化（因果链）', 'SVM 特征变化'],
    rows,
    widths: compact ? [60, 96, 280, 190] : [100, 120, 300, 230],
    top: compact ? 33 : 54,
    rowH: compact ? 19 : 42,
    pad: compact ? 14 : 26,
    large,
    aligns: ['left', 'left', 'left', 'left'],
    footer: compact ? null : '功率因数、振动与 SVM 样本分布为教学设定；电流 2sf 边频为教学简化频域示意。'
  });
}
function drawSvmParamsCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const pad = large ? 30 : 12;
  const rows = [
    ['核函数', 'RBF（径向基；二维显示为教学切面）'],
    ['核参数 γ', '0.85（二维可视化投影）'],
    ['惩罚系数 C', '1.40'],
    ['特征维度', '6 维特征向量；图中显示 I均值 × dn/dt 投影'],
    ['训练 / 验证样本', '92 条（正常 46 + 故障 46）'],
    ['训练准确率', '95.6%（教学校准值）'],
    ['支持向量', `${svmModel.support.length} 个（图中圈出）`],
    ['类别输出', '正常类 / 故障类（二分类）'],
    ['判决函数', 'f(x) = Σ αᵢ yᵢ K(xᵢ,x) + b'],
    ['当前判决', state.svmNormal ? '正常类' : (state.faultLabel || '故障类')],
    ['置信度', `${(state.svmConfidence * 100).toFixed(1)}%`]
  ];
  if (!large) {
    mono(ctx, 6.3, C.dim, 500, 'left'); ctx.fillText('RBF / TEACHING SVM', 10, 13);
    const compactRows = [
      ['RBF', 'KERNEL'], ['γ 0.85', 'GAMMA'], ['C 1.40', 'PENALTY'],
      ['n 92', 'SAMPLES'], ['95.6%', 'ACCURACY'], ['6D → 2D', 'FEATURES'],
      [`SV ${svmModel.support.length}`, 'SUPPORT'], [`f ${state.svmScore.toFixed(2)}`, 'DECISION']
    ];
    const cw = (w - 24) / 2;
    compactRows.forEach((r, i) => {
      const x = 10 + (i % 2) * cw;
      const y = 32 + Math.floor(i / 2) * 28;
      miniStat(ctx, x, y, Math.min(cw - 8, 112), r[0], r[1], i > 5 ? C.amber : C.steel);
    });
    return;
  }
  if (large) {
    mono(ctx, 8, C.muted, 500, 'left'); ctx.fillText('SVM MODEL CARD / 参数与教学设定', pad, 17);
    mono(ctx, 8, C.dim, 500, 'right'); ctx.fillText('NOT FOR INDUSTRIAL USE', w - pad, 17);
  }
  const top = large ? 49 : 20;
  const rowH = large ? 37 : 16;
  rows.forEach((r, i) => {
    const y = top + i * rowH;
    if (i % 2 === 0) {
      ctx.fillStyle = 'rgba(255,255,255,.014)';
      ctx.fillRect(pad, y - (large ? 14 : 6), w - pad * 2, large ? rowH - 2 : 12);
    }
    zh(ctx, large ? 9 : 6.4, C.muted, 500, 'left');
    ctx.fillText(r[0], pad, y);
    mono(ctx, large ? 9 : 6.4, C.steel, 500, 'left');
    ctx.fillText(r[1], pad + (large ? 200 : 80), y);
    hline(ctx, y + (large ? 17 : 8), pad, w - pad, C.line2);
  });
  if (large) {
    mono(ctx, 7.5, C.dim, 500, 'left');
    ctx.fillText('说明：SVM 为课程演示用固定训练模型，二维散点为六维特征的线性化教学切面；故障样本、混淆矩阵与准确率均为教学设定，不构成工业诊断依据。', pad, h - 14);
  }
}
function drawAlarmsCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const pad = large ? 30 : 12;
  if (!state.alarmLog.length) {
    if (large) {
      mono(ctx, 8, C.muted, 500, 'left'); ctx.fillText('EVENT LOG / 报警记录', pad, 18);
      zh(ctx, 13, C.muted, 500, 'center');
      ctx.fillText('暂无报警记录', w / 2, h * 0.45);
      mono(ctx, 8, C.dim, 500, 'center');
      ctx.fillText('I > 3.5 A · n < 100 r/min · T > 75 ℃ · 连续越限消抖后触发', w / 2, h * 0.54);
      hline(ctx, h * 0.62, w * 0.29, w * 0.71, C.line2);
      zh(ctx, 8, C.dim, 500, 'center');
      ctx.fillText('触发后：报警条 · 三色声光灯闪烁 · 保护延时 4.2 s · SVM 判决入档', w / 2, h * 0.72);
    } else {
      mono(ctx, 6, C.dim, 500, 'center'); ctx.fillText('NO EVENTS', w / 2, h / 2);
    }
    return;
  }
  const rows = state.alarmLog.slice(0, large ? 12 : 5).map((a) => [a.time, a.type === 'stall' ? '堵转 / 过流' : a.type === 'overheat' ? '超温' : a.type, a.threshold, a.reading, a.svm, a.executed ? '已停机' : '已复位']);
  if (!large) {
    mono(ctx, 6.5, C.dim, 500, 'left'); ctx.fillText('EVENT LOG / LATCHED', 10, 13);
    rows.forEach((r, i) => {
      const y = 30 + i * 21;
      mono(ctx, 6.3, C.dim, 500, 'left'); ctx.fillText(r[0], 10, y);
      mono(ctx, 6.8, C.red, 500, 'left'); ctx.fillText(r[1], 53, y);
      mono(ctx, 6.3, C.muted, 500, 'right'); ctx.fillText(r[3], w - 12, y);
      mono(ctx, 5.5, C.dim, 500, 'left'); ctx.fillText(`${r[2]} · ${r[4]} · ${r[5]}`, 53, y + 10);
      hline(ctx, y + 15, 10, w - 10, C.line2);
    });
    return;
  }
  drawTable(ctx, w, h, {
    title: 'EVENT LOG / 报警与保护记录',
    note: 'LATCHED UNTIL RESET',
    headers: ['TIME', 'TYPE', 'THRESHOLD', 'READING', 'SVM', 'ACTION'],
    rows,
    widths: [90, 150, 170, 116, 140, 100],
    top: 54, rowH: 44, pad: 26, large,
    aligns: ['left', 'left', 'left', 'right', 'left', 'left'],
    footer: '保护延时用于保留卡滞堵转的电磁教学观察窗口；4.2 s 后执行停机保护。'
  });
}

// ---- nameplate ----
function drawNameplateCard(ctx, w, h) {
  canvasBg(ctx, w, h);
  const large = w > 560;
  const pad = large ? 60 : 16;
  const plateW = w - pad * 2;
  let plateH = plateW * 560 / 1024;
  if (plateH > h - (large ? 92 : 24)) {
    plateH = h - (large ? 92 : 24);
  }
  const plateW2 = plateH * 1024 / 560;
  const x = (w - plateW2) / 2;
  const y = large ? 38 : 10;
  ctx.save();
  ctx.shadowColor = 'rgba(0,0,0,.55)';
  ctx.shadowBlur = large ? 24 : 8;
  ctx.fillStyle = '#080a0b';
  ctx.fillRect(x - 4, y - 4, plateW2 + 8, plateH + 8);
  ctx.restore();
  ctx.save();
  ctx.translate(x, y);
  drawNameplateArt(ctx, plateW2, plateH);
  ctx.restore();
  if (large) {
    zh(ctx, 8, C.dim, 500, 'center');
    ctx.fillText('Y90S-4 教学虚构铭牌 · 参数口径与 3D 仿真、右侧面板、SVM 特征保持一致', w / 2, y + plateH + 25);
  }
}

// ---- data text builders ----
function dataTextTrend() {
  const now = state.simTime;
  const data = state.trend.filter((p) => p.t >= now - 60);
  const rows = ['| t (s) | I (A) | n (r/min) | T (℃) |', '|---:|---:|---:|---:|'];
  const seen = new Set();
  for (const p of data) {
    const key = Math.floor(p.t);
    if (seen.has(key)) continue;
    seen.add(key);
    rows.push(`| ${p.t.toFixed(1)} | ${p.I.toFixed(3)} | ${p.n.toFixed(1)} | ${p.T.toFixed(2)} |`);
  }
  return `## 三通道实时趋势\n\n${rows.join('\n')}\n\n窗口 60 s · 采样 50 Hz · 展示采样 10 Hz`;
}
function dataTextTorque() {
  return `## T–s 机械特性\n\n| 项目 | 数值 |\n|---|---:|\n| 当前转差率 s | ${state.slip.toFixed(4)} |\n| 当前转速 n | ${state.nRpm.toFixed(1)} r/min |\n| 当前电磁转矩 Tₑ | ${state.emTorqueNm.toFixed(2)} N·m |\n| 输出转矩（负载侧） | ${state.torqueNm.toFixed(2)} N·m |\n| 额定点 | s=${MC.ratedSlip.toFixed(4)}, T=${MC.ratedTorqueNm.toFixed(2)} N·m |\n| 最大转矩点 | s=${MC.breakdownSlip.toFixed(2)}, T=${(MC.maxTorqueFactor * MC.ratedTorqueNm).toFixed(2)} N·m |`;
}
function dataTextSvm() {
  return `## SVM 判决\n\n- 判决：${state.svmNormal ? '正常类' : (state.faultLabel || '故障类')}\n- 决策函数 f(x)：${state.svmScore.toFixed(3)}\n- 置信度：${(state.svmConfidence * 100).toFixed(1)}%\n- 特征：I均值 ${state.feature.currentMean.toFixed(3)} A / I方差 ${state.feature.currentVar.toFixed(5)} / I峭度 ${state.feature.currentKurt.toFixed(2)} / dn/dt ${state.feature.speedRate.toFixed(3)} rpm/s / 跟随误差 ${state.feature.followError.toFixed(2)} r/min / dT/dt ${state.feature.tempSlope.toFixed(3)} ℃/s\n- 核函数：RBF（二维投影显示）\n- 支持向量：${svmModel.support.length}`;
}
function dataTextGauges() {
  return `## 运行工况\n\n| 参数 | 当前值 | 参照 |\n|---|---:|---:|\n| 定子电流 | ${state.currentA.toFixed(2)} A | 额定 2.2 / 阈值 3.5 |\n| 转速 | ${state.nRpm.toFixed(0)} r/min | 同步 1500 / 额定 1400 |\n| 壳体温度 | ${state.tempC.toFixed(1)} ℃ | 稳态 55 / 阈值 75 |\n| 输出转矩 | ${state.torqueNm.toFixed(2)} N·m | 额定 7.50 |\n| 转差率 | ${state.slip.toFixed(4)} | 额定 0.0667 |\n| 输出功率 | ${state.powerKW.toFixed(2)} kW | 额定 1.10 |`;
}
function dataTextTable(headers, rows) {
  return `| ${headers.join(' | ')} |\n|${headers.map(() => '---').join('|')}|\n` + rows.map((r) => `| ${r.join(' | ')} |`).join('\n');
}

// ===== 41. 可放大卡片注册表（14 个视图） =====
registerCard({
  id: 'trend', title: '三通道实时趋势', sub: '定子电流 / 实际转速 / 壳体温度 · 最近 60 s',
  draw: drawTrendCard, dataText: dataTextTrend, live: true,
  hint: '悬停读取精确时刻 · 暂停后仍可悬停 · 选区可切换 10 / 30 / 60 s'
});
registerCard({
  id: 'gauges', title: '数字仪表盘', sub: '四参数量程 · 额定对比 · 报警阈值',
  draw: drawGaugeCard, dataText: dataTextGauges, live: true,
  hint: '水平标尺中白线为额定值，琥珀 / 红刻度为预警 / 跳闸阈值'
});
registerCard({
  id: 'torque', title: '转矩–转差特性', sub: 'T(s) 教学拟合 · 同步点 / 额定点 / 最大转矩点 / 当前工作点',
  draw: drawTorqueCard, dataText: dataTextTorque, live: true,
  hint: '当前工作点按实际转差率与 T(s) 曲线计算，不取净加速转矩'
});
registerCard({
  id: 'svm', title: 'SVM 决策散点', sub: '二维特征投影 · RBF 判决边界 · 支持向量 · 实时样本',
  draw: drawSvmCard, dataText: dataTextSvm, live: true,
  hint: '横向为电流均值，纵向为转速变化率；虚线为 ±1 间隔边界'
});
registerCard({
  id: 'confusion', title: '混淆矩阵', sub: '课程 SVM 验证集 · 92 个样本 · 准确率 95.6%',
  draw: drawConfusionCard, dataText: () => dataTextTable(['', '预测正常', '预测故障'], [['实际正常', 44, 2], ['实际故障', 2, 44]]) + '\n\n准确率 95.6% · 教学设定',
  live: false
});
registerCard({
  id: 'flow', title: '诊断流程', sub: '采集 → 预处理 → 特征 → SVM → 结论 → 报警联动',
  draw: drawFlowCard, dataText: () => `## 诊断流程\n\n1. 信号采集：5 路传感器 / 50 Hz\n2. 预处理：滤波 / 归一化\n3. 特征提取：2 s 窗口、100 点、6 特征\n4. SVM 判决：f(x)=${state.svmScore.toFixed(3)}\n5. 结论：${state.svmNormal ? '正常类' : state.faultLabel}\n6. 报警联动：${state.alarm ? (state.alarm.executed ? '保护停机' : '保护延时') : '监测中'}`,
  live: true
});
registerCard({
  id: 'features', title: '六特征实时表', sub: '2 s 窗口 · 100 点 · 电流均值 / 方差 / 峭度 / dn/dt / 跟随误差 / dT/dt',
  draw: drawFeaturesCard,
  dataText: () => `## 六特征\n\n${dataTextTable(['Feature', 'Value', 'Band'], [
    ['I mean (A)', state.feature.currentMean.toFixed(4), '1.9–2.5'],
    ['I var (A²)', state.feature.currentVar.toFixed(6), '<0.002'],
    ['I kurtosis', state.feature.currentKurt.toFixed(3), '2.5–4.0'],
    ['dn/dt (rpm/s)', state.feature.speedRate.toFixed(3), '−0.4–0.4'],
    ['Follow error (rpm)', state.feature.followError.toFixed(2), '<8'],
    ['dT/dt (℃/s)', state.feature.tempSlope.toFixed(4), '−0.03–0.03']
  ])}`, live: true
});
registerCard({
  id: 'sensorDetails', title: '传感器详情参数表', sub: '五路实体传感器 · 量程 / 精度 / 实时读数 / 状态',
  draw: drawSensorDetailsCard,
  dataText: () => `## 传感器详情\n\n${dataTextTable(['Code', 'Sensor', 'Value', 'Range', 'Status', 'Position'], SENSORS.map((s) => { const r = getSensorReading(s.id); return [s.code, s.name, `${r.display} ${s.unit}`, s.range, r.status, s.position]; }))}`,
  live: true
});
registerCard({
  id: 'nameplate', title: '电机铭牌', sub: 'Y90S-4 · 380 V △ · 1.1 kW · 1400 r/min',
  draw: drawNameplateCard,
  dataText: () => `## 铭牌\n\nY90S-4 型三相异步电动机\n\n- 额定功率：1.1 kW\n- 额定电压 / 接法：380 V / △\n- 额定电流：2.2 A\n- 额定转速：1400 r/min\n- 同步转速：1500 r/min（4P / 50 Hz）\n- 功率因数 / 效率：0.78 / 79%\n- 防护 / 工作制：IP44 / S1\n- 轴承：6205-2Z / 6204-2Z\n\n教学虚构铭牌，数据口径与 3D、图表和诊断一致。`,
  live: false
});
registerCard({
  id: 'alarms', title: '报警记录', sub: '阈值触发 / 保护延时 / SVM 判决入档',
  draw: drawAlarmsCard,
  dataText: () => state.alarmLog.length ? `## 报警记录\n\n${dataTextTable(['Time', 'Type', 'Threshold', 'Reading', 'SVM', 'Action'], state.alarmLog.map((a) => [a.time, a.type, a.threshold, a.reading, a.svm, a.executed ? '已停机' : '已复位']))}` : '## 报警记录\n\n暂无报警。',
  live: true
});
registerCard({
  id: 'normalParams', title: '正常工况参数表', sub: '单一数据源 MOTOR_CONFIG · 额定与阈值口径',
  draw: drawNormalParamsCard,
  dataText: () => `## 正常工况参数\n\n${dataTextTable(['项目', '数值'], [
    ['型号', MC.model], ['功率', `${MC.ratedPowerKW} kW`], ['电压 / 接法', `${MC.ratedVoltageV} V / △`],
    ['电流', `${MC.ratedCurrentA} A`], ['同步转速', `${MC.synchronousSpeedRpm} r/min`],
    ['额定转速', `${MC.ratedSpeedRpm} r/min`], ['额定转差率', MC.ratedSlip.toFixed(4)],
    ['额定转矩', `${MC.ratedTorqueNm.toFixed(2)} N·m`], ['槽数 定/转', `${MC.statorSlots}/${MC.rotorSlots}`],
    ['气隙', `${MC.airGapMm} mm（示意）`], ['采样', `${MC.sampleHz} Hz / ${MC.windowSize} 点`]
  ])}`, live: false
});
registerCard({
  id: 'faultmap', title: '故障特征对照表', sub: '故障部位 → 参数变化因果链 → SVM 特征变化',
  draw: drawFaultMapCard,
  dataText: () => `## 故障特征对照\n\n${dataTextTable(['工况', '部位', '参数变化', 'SVM特征'], [
    ['正常', '—', 'I 2.2 A / n 1400 / T 55℃', '正常簇'],
    ['卡滞堵转', '输出轴楔块', 'I↑ 4–6 A, n→0, s=1.00, T↑', 'I均值/方差/峭度↑, dn/dt↑'],
    ['轴承磨损', '前端轴承', '振动RMS↑, 周期冲击', 'I峭度↑, 振动↑'],
    ['转子断条', '鼠笼导条', '2sf 电流脉动, 转速摆动', 'I方差/峭度↑, dn/dt↑'],
    ['绕组过热', '定子绕组', 'T↑ >75℃, I↑, n↓', 'dT/dt↑'],
    ['过载', '负载侧', 'n↓ 沿 T(s), I→3.5A', '介于正常/故障, 置信度↓']
  ])}`, live: false
});
registerCard({
  id: 'sensorLocations', title: '传感器安装位置对照表', sub: '测量什么 / 装在哪里 / 指向哪个部件',
  draw: drawSensorLocationsCard,
  dataText: () => `## 传感器安装位置\n\n${dataTextTable(['Code', '传感器', '安装位置', '被测部件', '量程'], SENSORS.map((s) => [s.code, s.name, s.position, s.part, s.range]))}`,
  live: false
});
registerCard({
  id: 'svmParams', title: 'SVM 参数说明表', sub: 'RBF 核 · C / γ · 样本集 · 支持向量 · 教学声明',
  draw: drawSvmParamsCard,
  dataText: () => `## SVM 参数（教学）\n\n- Kernel: RBF\n- γ: 0.85（二维显示投影）\n- C: 1.40\n- Features: 6 维，图中显示 Ī × dn/dt\n- Samples: 92（正常 46 / 故障 46）\n- Accuracy: 95.6%（教学校准）\n- Support vectors: ${svmModel.support.length}\n- 当前 f(x): ${state.svmScore.toFixed(3)}\n- 当前置信度: ${(state.svmConfidence * 100).toFixed(1)}%`,
  live: true
});

const ATLAS_ORDER = ['trend', 'gauges', 'torque', 'svm', 'confusion', 'flow', 'features', 'sensorDetails', 'sensorLocations', 'nameplate', 'normalParams', 'faultmap', 'alarms', 'svmParams'];
const ATLAS_WIDE = new Set(['faultmap', 'sensorLocations']);

// ===== 42. 迷你图 / 侧栏火花线 =====
function prepareCanvasByClient(canvas, dpr = Math.min(window.devicePixelRatio || 1, 2)) {
  if (!canvas) return null;
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 2 || rect.height < 2) return null;
  const ctx = getCanvasCtx(canvas, rect.width, rect.height, dpr);
  return { ctx, w: rect.width, h: rect.height };
}

function drawAllMiniCharts(force = false) {
  const map = [
    ['trend', D('miniTrend')],
    ['torque', D('miniTorque')],
    ['svm', D('miniSvm')]
  ];
  for (const [id, canvas] of map) {
    if (!canvas) continue;
    const prep = prepareCanvasByClient(canvas);
    if (!prep) continue;
    const card = cardRegistry.get(id);
    if (!card) continue;
    try { card.draw(prep.ctx, prep.w, prep.h, { large: false }); } catch (err) { console.warn('chart draw failed', id, err); }
  }
  drawGaugeSpark(D('miniCurrent'), state.sensorHistory.hall, C.cyan, MC.thresholds.currentTripA);
  drawGaugeSpark(D('miniSpeed'), state.sensorHistory.encoder, C.steel, 100);
}

function drawGaugeSpark(canvas, values, color, threshold) {
  if (!canvas) return;
  const prep = prepareCanvasByClient(canvas);
  if (!prep) return;
  const { ctx, w, h } = prep;
  ctx.clearRect(0, 0, w, h);
  const hist = values.slice(-80);
  if (!hist.length) return;
  let min = Math.min(...hist), max = Math.max(...hist);
  if (min === max) { min -= 1; max += 1; }
  const range = max - min;
  ctx.beginPath();
  hist.forEach((v, i) => {
    const x = 1 + i / Math.max(1, hist.length - 1) * (w - 2);
    const y = h - 3 - (v - min) / range * (h - 6);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = color;
  ctx.globalAlpha = 0.55;
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.globalAlpha = 1;
  if (Number.isFinite(threshold) && threshold >= min && threshold <= max) {
    const ty = h - 3 - (threshold - min) / range * (h - 6);
    ctx.strokeStyle = 'rgba(228,90,82,.35)';
    ctx.setLineDash([2, 3]);
    ctx.beginPath(); ctx.moveTo(0, ty); ctx.lineTo(w, ty); ctx.stroke();
    ctx.setLineDash([]);
  }
}

// ===== 43. 放大卡片 / 模态层 =====
let modalResizeTimer = 0;
function openCard(id) {
  const card = cardRegistry.get(id);
  if (!card) { showToast(`未知数据卡：${id}`, 'warn'); return; }
  ui.modalCardId = id;
  ui.modalPaused = false;
  ui.hoverX = null;
  ui.hoverY = null;
  const layer = D('modalLayer');
  layer.classList.add('open');
  setTxt('modalTitle', card.title);
  setTxt('modalSub', card.sub);
  setTxt('modalHint', card.hint || 'ESC 关闭 · 双击数据卡 / Enter 打开');
  setTxt('modalMeta', card.live ? 'LIVE · SOURCE 50 Hz / 2 s' : 'STATIC · TEACHING CONFIG');
  const pauseBtn = D('modalPause');
  pauseBtn.classList.toggle('hidden', !card.live);
  pauseBtn.textContent = '暂停';
  D('modalOptions').classList.remove('show');
  D('modalOptions').innerHTML = '';
  if (id === 'trend') {
    D('modalOptions').classList.add('show');
    D('modalOptions').innerHTML = `
      <span>时间窗口</span>
      <select id="modalRange">
        <option value="10">10 s</option><option value="30">30 s</option>
        <option value="60" selected>60 s</option>
      </select>
      <span style="color:var(--muted-2)">drag / hover crosshair to inspect</span>`;
    $('#modalRange').addEventListener('change', (e) => {
      ui.modalRange = Number(e.target.value);
      drawModal();
    });
  }
  // next frame to guarantee layout dimensions
  requestAnimationFrame(() => {
    drawModal();
    layer.querySelector('.modal')?.focus?.();
  });
}

function closeModal() {
  const layer = D('modalLayer');
  if (!layer.classList.contains('open')) return;
  layer.classList.remove('open');
  ui.modalCardId = null;
  ui.modalPaused = false;
  ui.hoverX = null;
  ui.hoverY = null;
  if (document.fullscreenElement) document.exitFullscreen?.().catch(() => {});
}

function drawModal() {
  if (!ui.modalCardId) return;
  const card = cardRegistry.get(ui.modalCardId);
  if (!card) return;
  const canvas = D('modalCanvas');
  const wrap = canvas?.parentElement;
  if (!canvas || !wrap) return;
  const rect = wrap.getBoundingClientRect();
  if (rect.width < 20 || rect.height < 20) return;
  const ctx = getCanvasCtx(canvas, wrap.clientWidth, wrap.clientHeight, Math.min(window.devicePixelRatio || 1, 2.5));
  try {
    card.draw(ctx, rect.width, rect.height, {
      large: true,
      hoverX: ui.hoverX,
      hoverY: ui.hoverY
    });
  } catch (err) {
    console.warn('modal draw failed', err);
  }
}

function toggleModalPause() {
  if (!ui.modalCardId) return;
  ui.modalPaused = !ui.modalPaused;
  D('modalPause').textContent = ui.modalPaused ? '继续' : '暂停';
  if (!ui.modalPaused) drawModal();
}

async function copyCardData() {
  const card = cardRegistry.get(ui.modalCardId);
  if (!card) return;
  const text = typeof card.dataText === 'function' ? card.dataText() : String(card.dataText || '');
  try {
    await navigator.clipboard.writeText(text);
    showToast('数据已复制 · Markdown', 'info');
  } catch {
    const ta = document.createElement('textarea');
    ta.value = text;
    ta.style.cssText = 'position:fixed;top:-9999px;left:-9999px';
    document.body.appendChild(ta);
    ta.select();
    try { document.execCommand('copy'); showToast('数据已复制 · Markdown', 'info'); }
    catch { showToast('复制失败，请手动选择内容', 'warn'); }
    ta.remove();
  }
}

function exportCardPng() {
  const card = cardRegistry.get(ui.modalCardId);
  if (!card) return;
  const expW = 1600;
  const expH = card.id === 'nameplate' ? 1080 : 920;
  const off = document.createElement('canvas');
  off.width = expW; off.height = expH;
  const ctx = off.getContext('2d');
  try {
    card.draw(ctx, expW, expH, { large: true });
    // Add a compact export header above the chart body by composing a small band.
    ctx.save();
    ctx.fillStyle = 'rgba(5,9,13,.82)';
    ctx.fillRect(0, 0, expW, 50);
    ctx.strokeStyle = 'rgba(112,216,229,.26)';
    ctx.beginPath(); ctx.moveTo(0, 50); ctx.lineTo(expW, 50); ctx.stroke();
    ctx.font = `500 16px ${CANVAS_SANS}`;
    ctx.fillStyle = '#e8edf2';
    ctx.fillText(card.title, 28, 22);
    ctx.font = `500 11px ${CANVAS_MONO}`;
    ctx.fillStyle = '#8794a3';
    ctx.fillText(card.sub, 28, 39);
    ctx.textAlign = 'right';
    ctx.fillStyle = '#70d8e5';
    ctx.font = `500 11px ${CANVAS_MONO}`;
    ctx.fillText('THE HEART OF DRIVE · Y90S-4 · 3D DIAGNOSTIC LAB', expW - 28, 29);
    ctx.restore();
    // Synchronous data-URL conversion keeps the download inside the user's
    // click activation (toBlob callbacks lose activation under file:// in some
    // Chromium builds and can be silently throttled).
    const url = off.toDataURL('image/png');
    const a = document.createElement('a');
    a.href = url;
    a.download = `heart-of-drive_${card.id}_${Date.now()}.png`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    showToast('PNG 已导出', 'info');
  } catch (err) {
    console.warn(err);
    showToast('PNG 导出失败', 'warn');
  }
}

async function toggleModalFullscreen() {
  const wrap = $('.modal-canvas-wrap');
  if (!wrap) return;
  try {
    if (!document.fullscreenElement) await wrap.requestFullscreen();
    else await document.exitFullscreen();
    setTimeout(drawModal, 200);
  } catch {
    showToast('浏览器拒绝了全屏请求', 'warn');
  }
}

function bindModalEvents() {
  D('modalClose')?.addEventListener('click', closeModal);
  D('modalLayer')?.addEventListener('click', (e) => {
    if (e.target === D('modalLayer')) closeModal();
  });
  D('modalPause')?.addEventListener('click', toggleModalPause);
  D('modalCopy')?.addEventListener('click', copyCardData);
  D('modalExport')?.addEventListener('click', exportCardPng);
  D('modalFullscreen')?.addEventListener('click', toggleModalFullscreen);
  const canvas = D('modalCanvas');
  canvas?.addEventListener('pointermove', (e) => {
    const rect = canvas.getBoundingClientRect();
    ui.hoverX = clamp((e.clientX - rect.left) / rect.width, 0, 1);
    ui.hoverY = clamp((e.clientY - rect.top) / rect.height, 0, 1);
    drawModal();
  });
  canvas?.addEventListener('pointerleave', () => {
    ui.hoverX = null; ui.hoverY = null; drawModal();
  });
  window.addEventListener('resize', () => {
    clearTimeout(modalResizeTimer);
    modalResizeTimer = setTimeout(() => { if (ui.modalCardId) drawModal(); }, 120);
  });
  document.addEventListener('fullscreenchange', () => setTimeout(drawModal, 140));
  // global delegated card opening
  document.addEventListener('click', (e) => {
    if (e.button !== 0) return;
    const target = e.target.closest('[data-open-card], .chart-card');
    if (!target) return;
    if (e.target.closest('.modal-layer')) return;
    const id = target.dataset.card || target.dataset.openCard || target.dataset.card;
    if (id) {
      // a chart card expand icon must not propagate into an accidental DOM reflow
      openCard(id === 'faultMap' ? 'faultmap' : id);
    }
  });
  document.addEventListener('dblclick', (e) => {
    const target = e.target.closest('[data-open-card], .chart-card, .main-gauge, .sub-gauge');
    if (!target || e.target.closest('.layer')) return;
    const id = target.dataset.card || target.dataset.openCard;
    if (id) openCard(id === 'faultMap' ? 'faultmap' : id);
  });
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' && !e.target.closest('input,select,textarea')) {
      const target = e.target.closest('[data-open-card], .chart-card');
      if (target) {
        e.preventDefault();
        const id = target.dataset.card || target.dataset.openCard;
        if (id) openCard(id === 'faultMap' ? 'faultmap' : id);
      }
    }
  });
  D('sensorZoomBtn')?.addEventListener('click', () => openCard('sensorDetails'));
  D('partSheetDataBtn')?.addEventListener('click', (e) => {
    const card = e.currentTarget.dataset.card || 'normalParams';
    openCard(card);
  });
}

// ===== 44. 数据图谱 / 答辩大屏 =====
function buildAtlas() {
  const grid = D('atlasGrid');
  if (!grid || grid.dataset.built === '1') return;
  grid.dataset.built = '1';
  grid.innerHTML = '';
  const refs = new Map();
  for (const id of ATLAS_ORDER) {
    const card = cardRegistry.get(id);
    if (!card) continue;
    const el = document.createElement('button');
    el.className = 'atlas-card' + (ATLAS_WIDE.has(id) ? ' wide' : '');
    el.type = 'button';
    el.dataset.atlasCard = id;
    el.innerHTML = `<div class="atlas-card-head"><b>${card.title}</b><span>${card.live ? 'LIVE' : 'STATIC'} · ENLARGE ⤢</span></div><canvas></canvas><span class="atlas-badge">${id.toUpperCase()} / DATA VIEW</span>`;
    el.addEventListener('click', () => openCard(id));
    grid.appendChild(el);
    refs.set(id, { el, canvas: el.querySelector('canvas'), card });
  }
  atlasRefs = refs;
  requestAnimationFrame(drawAtlasCards);
}
let atlasRefs = new Map();

function drawAtlasCards() {
  if (!ui.atlasOpen) return;
  for (const [id, item] of atlasRefs) {
    const prep = prepareCanvasByClient(item.canvas);
    if (!prep) continue;
    try { item.card.draw(prep.ctx, prep.w, prep.h, { large: false }); } catch (err) { /* ignore one card */ }
  }
  const time = D('atlasSubTime');
  if (time) time.textContent = nowClock();
}

function openAtlas() {
  ui.atlasOpen = true;
  const layer = D('atlasLayer');
  buildAtlas();
  layer.classList.add('open');
  document.body.classList.add('atlas-open');
  requestAnimationFrame(() => requestAnimationFrame(drawAtlasCards));
}

function closeAtlas() {
  if (D('modalLayer')?.classList.contains('open')) closeModal();
  ui.atlasOpen = false;
  D('atlasLayer')?.classList.remove('open');
  document.body.classList.remove('atlas-open');
}

async function fullscreenAtlas() {
  const el = D('atlasLayer');
  try {
    if (!document.fullscreenElement) await el.requestFullscreen();
    else await document.exitFullscreen();
    setTimeout(drawAtlasCards, 240);
  } catch { showToast('浏览器拒绝了全屏请求', 'warn'); }
}

// ===== 45. 帮助 / 声音面板 / 引导 / toast =====
let toastTimer = 0;
function showToast(message, type = 'info') {
  const stack = D('toastStack');
  if (!stack) return;
  const el = document.createElement('div');
  el.className = 'toast ' + type;
  el.textContent = message;
  stack.appendChild(el);
  while (stack.children.length > 4) stack.firstElementChild.remove();
  setTimeout(() => {
    el.style.opacity = '0';
    el.style.transition = 'opacity .25s';
    setTimeout(() => el.remove(), 280);
  }, 3200);
}

const GUIDE_STEPS = [
  { title: '先转一圈，观察这台电机。', desc: '在 3D 视图中拖拽旋转视角，滚轮靠近或推远；右侧参数会保持同步。' },
  { title: '悬停部件，读出它的材料与运动状态。', desc: '把指针停在机座、定子端部、转子或轴承上；部件信息卡会说明它此刻正在做什么。' },
  { title: '换一只眼睛看结构。', desc: '切换「机械切面」，定子铁芯、24 槽三相绕组、气隙、鼠笼导条和轴承会同时出现。' },
  { title: '把这台机器拆成装配顺序。', desc: '点击「分层爆炸」，风罩、风扇、后盖、转子组件和前端盖会沿轴向排列；再次点击归位。' },
  { title: '让旋转磁场现身。', desc: '开启「旋转磁场」，观察 1500 r/min 同步磁场与 1400 r/min 转子之间真实存在的转差。' },
  { title: '点开一路传感器，看完整测量链。', desc: '底部五路状态带、右侧列表与三维模型共用同一数据源；选中后会高亮被测部件。' },
  { title: '注入一次卡滞堵转。', desc: '看电流升高、光电脉冲消失、转子停转，而磁场仍以同步速旋转。报警与保护会联动。' },
  { title: '最后，打开数据图谱。', desc: '所有图表、散点、特征表和报警记录都能放大，并导出 PNG 或复制 Markdown 数据用于课程报告。' }
];

function showGuide() {
  if (ui.guideVisible) return;
  ui.guideVisible = true;
  ui.guideStep = 0;
  renderGuide();
  D('guide')?.classList.add('show');
}
function renderGuide() {
  const step = GUIDE_STEPS[ui.guideStep % GUIDE_STEPS.length];
  setTxt('guideTitle', step.title);
  setTxt('guideDesc', step.desc);
  const dots = D('guideDots');
  if (dots) dots.innerHTML = GUIDE_STEPS.map((_, i) => `<i class="${i <= ui.guideStep ? 'on' : ''}"></i>`).join('');
  const next = D('guideNext');
  if (next) next.textContent = ui.guideStep >= GUIDE_STEPS.length - 1 ? '开始使用' : '下一步 →';
}
function advanceGuide() {
  if (!ui.guideVisible) return;
  if (ui.guideStep >= GUIDE_STEPS.length - 1) {
    dismissGuide();
    return;
  }
  ui.guideStep++;
  renderGuide();
}
function advanceGuideTo(step) {
  if (!ui.guideVisible) return;
  ui.guideStep = Math.max(ui.guideStep, step);
  if (ui.guideStep >= GUIDE_STEPS.length) ui.guideStep = GUIDE_STEPS.length - 1;
  renderGuide();
}
function dismissGuide() {
  ui.guideVisible = false;
  D('guide')?.classList.remove('show');
}
function toggleHelp(show = null) {
  const layer = D('helpLayer');
  const next = show === null ? !layer.classList.contains('open') : show;
  layer.classList.toggle('open', next);
}
function openSoundPopover() {
  const pop = D('soundPop');
  const btn = D('soundBtn');
  if (!pop || !btn) return;
  const rect = btn.getBoundingClientRect();
  pop.style.left = `${Math.max(10, Math.min(window.innerWidth - 210, rect.right - 196))}px`;
  pop.style.top = `${rect.bottom + 7}px`;
  pop.classList.toggle('open');
}
function closeSoundPopover() { D('soundPop')?.classList.remove('open'); }

// ===== 46. Web Audio 程序化声音 =====
class AudioEngine {
  constructor() {
    this.ctx = null;
    this.nodes = {};
    this.enabled = false;
    this.volume = 0.42;
    this.mode = 'condition';
    this.started = false;
    this.nextBearingClick = 0;
    this.nextEventTick = 0;
    this.alarmTimers = [];
    this.updateTimer = null;
    this.schedulerTimer = null;
  }

  ensureGraph() {
    if (this.ctx) return true;
    const Ctx = window.AudioContext || window.webkitAudioContext;
    if (!Ctx) return false;
    try {
      const ctx = new Ctx();
      this.ctx = ctx;
      const master = ctx.createGain();
      master.gain.value = 0;
      master.connect(ctx.destination);
      this.nodes.master = master;

      // 100 Hz electromagnetic hum with low-order harmonics.
      const humBus = ctx.createGain();
      humBus.gain.value = 0;
      const humFilter = ctx.createBiquadFilter();
      humFilter.type = 'lowpass'; humFilter.frequency.value = 520; humFilter.Q.value = 0.7;
      humBus.connect(humFilter).connect(master);
      this.nodes.humBus = humBus;
      const humSpecs = [
        [100, 'sawtooth', 0.028],
        [200, 'sine', 0.012],
        [300, 'sine', 0.005],
        [50, 'sine', 0.006]
      ];
      this.nodes.hums = [];
      for (const [freq, type, gain] of humSpecs) {
        const osc = ctx.createOscillator();
        osc.type = type;
        osc.frequency.value = freq;
        const g = ctx.createGain();
        g.gain.value = gain;
        osc.connect(g).connect(humBus);
        osc.start();
        this.nodes.hums.push({ osc, gain: g, base: gain, baseFreq: freq });
      }
      // Air / fan broadband noise
      const noiseBuf = ctx.createBuffer(2, ctx.sampleRate * 2, ctx.sampleRate);
      for (let ch = 0; ch < 2; ch++) {
        const data = noiseBuf.getChannelData(ch);
        let last = 0;
        for (let i = 0; i < data.length; i++) {
          const white = Math.random() * 2 - 1;
          last = 0.985 * last + 0.015 * white;
          data[i] = last * 3.2;
        }
      }
      this.nodes.noiseBuf = noiseBuf;
      const fan = ctx.createBufferSource();
      fan.buffer = noiseBuf;
      fan.loop = true;
      const fanFilter = ctx.createBiquadFilter();
      fanFilter.type = 'bandpass'; fanFilter.frequency.value = 720; fanFilter.Q.value = 0.6;
      const fanGain = ctx.createGain(); fanGain.gain.value = 0;
      fan.connect(fanFilter).connect(fanGain).connect(master);
      fan.start();
      this.nodes.fan = fan; this.nodes.fanGain = fanGain; this.nodes.fanFilter = fanFilter;
      // Locked-rotor low-frequency rumble
      const sub = ctx.createOscillator();
      sub.type = 'sine'; sub.frequency.value = 47;
      const subGain = ctx.createGain(); subGain.gain.value = 0;
      sub.connect(subGain).connect(master);
      sub.start();
      this.nodes.sub = sub; this.nodes.subGain = subGain;

      this.schedulerTimer = setInterval(() => this.scheduler(), 150);
      this.updateTimer = setInterval(() => this.updateParams(), 180);
      return true;
    } catch (err) {
      console.warn('Audio init failed', err);
      return false;
    }
  }

  async toggle() {
    if (this.enabled) {
      this.disable();
      return false;
    }
    const ok = this.ensureGraph();
    if (!ok) {
      showToast('当前浏览器不支持 Web Audio', 'warn');
      return false;
    }
    try { await this.ctx.resume(); } catch { /* ignored */ }
    this.enabled = true;
    const t = this.ctx.currentTime;
    this.nodes.master.gain.cancelScheduledValues(t);
    this.nodes.master.gain.setTargetAtTime(this.volume, t, 0.08);
    this.updateParams();
    return true;
  }

  disable() {
    this.enabled = false;
    if (this.ctx) {
      const t = this.ctx.currentTime;
      this.nodes.master.gain.cancelScheduledValues(t);
      this.nodes.master.gain.setTargetAtTime(0, t, 0.05);
    }
    this.alarmOff();
  }

  setVolume(v) {
    this.volume = clamp(v, 0, 1);
    if (this.ctx) {
      const t = this.ctx.currentTime;
      this.nodes.master.gain.setTargetAtTime(this.enabled ? this.volume : 0, t, 0.06);
    }
  }

  setMode(mode) {
    this.mode = mode;
    this.updateParams();
  }

  isSlowPresentation() {
    return ui.observation !== 'real';
  }

  updateParams() {
    if (!this.ctx || !this.nodes.master) return;
    const t = this.ctx.currentTime;
    const slow = this.isSlowPresentation();
    const continuous = !slow || this.mode === 'condition';
    const field = state.fieldOn;
    const currentNorm = clamp((state.fieldOn ? state.currentA : 0) / MC.ratedCurrentA, 0, 3.2);
    const nNorm = clamp((state.fieldOn ? state.nRpm : 0) / MC.ratedSpeedRpm, 0, 1.2);
    const humTarget = this.enabled && continuous && field ? 0.008 + 0.014 * Math.min(1.8, currentNorm) : 0;
    this.nodes.humBus.gain.setTargetAtTime(humTarget, t, 0.12);
    for (const h of this.nodes.hums) {
      h.osc.frequency.setTargetAtTime(h.baseFreq + (state.nRpm / 1500) * 0.7, t, 0.2);
      const gainFactor = this.enabled && continuous ? 1 : 0;
      h.gain.gain.setTargetAtTime(h.base * gainFactor, t, 0.1);
    }
    const fanTarget = this.enabled && continuous && field ? nNorm * 0.022 : 0;
    this.nodes.fanGain.gain.setTargetAtTime(fanTarget, t, 0.15);
    this.nodes.fanFilter.frequency.setTargetAtTime(320 + nNorm * 900, t, 0.2);
    const stall = this.enabled && field && state.nRpm < 100 && currentNorm > 1.5;
    this.nodes.subGain.gain.setTargetAtTime(stall ? 0.04 : 0, t, 0.1);
    // event mode: very sparse mechanical click marks time in slow displays
    this.nodes.master.gain.setTargetAtTime(this.enabled ? this.volume : 0, t, 0.08);
  }

  noiseBurst(duration = 0.2, freq = 900, gainAmt = 0.05) {
    if (!this.ctx || !this.enabled) return;
    const ctx = this.ctx, t = ctx.currentTime;
    const src = ctx.createBufferSource();
    src.buffer = this.nodes.noiseBuf;
    const filter = ctx.createBiquadFilter();
    filter.type = 'bandpass'; filter.frequency.value = freq; filter.Q.value = 1.2;
    const gain = ctx.createGain();
    gain.gain.setValueAtTime(gainAmt, t);
    gain.gain.exponentialRampToValueAtTime(0.0001, t + duration);
    src.connect(filter).connect(gain).connect(this.nodes.master);
    src.start(t);
    src.stop(t + duration + 0.02);
  }

  playTick(strength = 0.02) {
    if (!this.ctx || !this.enabled) return;
    const ctx = this.ctx, t = ctx.currentTime;
    const osc = ctx.createOscillator();
    const g = ctx.createGain();
    osc.type = 'triangle';
    osc.frequency.setValueAtTime(260, t);
    osc.frequency.exponentialRampToValueAtTime(90, t + 0.06);
    g.gain.setValueAtTime(strength, t);
    g.gain.exponentialRampToValueAtTime(0.0001, t + 0.075);
    osc.connect(g).connect(this.nodes.master);
    osc.start(t); osc.stop(t + 0.08);
  }

  scheduler() {
    if (!this.ctx || !this.enabled) return;
    const now = this.ctx.currentTime;
    if (state.fault === 'bearing' && state.fieldOn && now >= this.nextBearingClick) {
      this.noiseBurst(0.035, 1800 + Math.random() * 900, 0.026);
      this.nextBearingClick = now + 0.23 + Math.random() * 0.12;
    }
    if (this.mode === 'event' && this.isSlowPresentation() && now >= this.nextEventTick) {
      this.playTick(0.012);
      this.nextEventTick = now + 1.1;
    }
  }

  powerOn() {
    if (!this.ctx || !this.enabled) return;
    this.noiseBurst(0.12, 1100, 0.035);
    const t = this.ctx.currentTime;
    const osc = this.ctx.createOscillator();
    const g = this.ctx.createGain();
    osc.type = 'sine';
    osc.frequency.setValueAtTime(86, t);
    osc.frequency.exponentialRampToValueAtTime(43, t + 0.32);
    g.gain.setValueAtTime(0.035, t);
    g.gain.exponentialRampToValueAtTime(0.0001, t + 0.38);
    osc.connect(g).connect(this.nodes.master);
    osc.start(t); osc.stop(t + 0.4);
  }

  powerOff() {
    if (!this.ctx || !this.enabled) return;
    this.noiseBurst(0.18, 340, 0.02);
  }

  alarmOn() {
    if (!this.ctx || !this.enabled) return;
    this.alarmOff();
    const beep = () => {
      if (!this.ctx || !this.enabled) return;
      const t = this.ctx.currentTime;
      for (const delay of [0, 0.16]) {
        const osc = this.ctx.createOscillator();
        const g = this.ctx.createGain();
        osc.type = 'square';
        osc.frequency.value = 880;
        g.gain.setValueAtTime(0.0001, t + delay);
        g.gain.exponentialRampToValueAtTime(0.014, t + delay + 0.012);
        g.gain.exponentialRampToValueAtTime(0.0001, t + delay + 0.10);
        osc.connect(g).connect(this.nodes.master);
        osc.start(t + delay); osc.stop(t + delay + 0.12);
      }
    };
    beep();
    this.alarmTimers.push(setInterval(beep, 1200));
  }

  alarmOff() {
    for (const timer of this.alarmTimers) clearInterval(timer);
    this.alarmTimers = [];
  }

  protectionStop() {
    if (!this.ctx || !this.enabled) return;
    this.noiseBurst(0.4, 120, 0.04);
    this.updateParams();
  }

  setFault(fault) {
    this.updateParams();
  }
}
const audioEngine = new AudioEngine();

// ===== 47. 电影演示 / 22 s =====
const CINEMA_DURATION = 24.0;
function buildCinematicSequence() {
  return [
    {
      t: 0,
      title: 'THE HEART OF DRIVE',
      sub: '三相异步电动机 · 电磁成力，转差生矩',
      fn: () => {
        ui.fieldVisible = false;
        setStructure('assembly', 'cinema');
        setPauseMechanism(false, { silent: true });
        setCameraPreset('hero');
        setObservationMode('auto', true);
        syncDisplayToggleButtons();
      }
    },
    {
      t: 4.2,
      title: 'ROTATING FIELD / 旋转磁场',
      sub: '同步磁场 1500 r/min · 转子 1400 r/min · s ≈ 0.067',
      fn: () => {
        setStructure('section', 'cinema');
        ui.fieldVisible = true;
        syncDisplayToggleButtons();
        setObservationMode('ultra', true);
        setPauseMechanism(false, { silent: true });
        setCameraPreset('field');
      }
    },
    {
      t: 9.4,
      title: 'ASSEMBLY / 精密结构',
      sub: 'Y90S-4 · 1.1 kW · 380 V △ · 2.2 A · 1400 r/min · 4P / 50 Hz',
      fn: () => {
        ui.fieldVisible = false;
        syncDisplayToggleButtons();
        setStructure('exploded', 'cinema');
        ui.rotorExtract = true;
        const p = getResponsiveCamera('exploded');
        orbitRig.transitionTo(p.pos, p.target, 1.2);
      }
    },
    {
      t: 15.2,
      title: 'FAULT INJECTION / 卡滞堵转',
      sub: '转子停转 · 磁场仍以同步速旋转 · 电流升至高位',
      fn: () => {
        setStructure('assembly', 'cinema');
        setPauseMechanism(false, { silent: true });
        setCameraPreset('hero');
        setObservationMode('auto', true);
        ui.fieldVisible = false;
        syncDisplayToggleButtons();
        injectFault('stall');
      }
    },
    {
      t: 18.1,
      title: 'DIAGNOSIS / SVM 判决',
      sub: 'I↑ / dn/dt↑ / 光电脉冲消失 → 故障类 · 保护联动',
      fn: () => {
        setCameraPreset('hero', false, 0.9);
      }
    },
    {
      t: 19.6,
      title: 'RECOVERY / 复位重启',
      sub: '清除故障 · 重新合闸 · 磁场重新建立',
      fn: () => {
        injectFault('normal');
        if (!state.powerOn) state.powerOn = true;
        state.runCmd = true;
        state.tripped = false;
        state.sinceStart = 0;
        state.startTransient = 7.6;
        ui.fieldVisible = false;
        syncDisplayToggleButtons();
        setStructure('assembly', 'cinema');
        setCameraPreset('hero');
      }
    }
  ];
}
let cinematicSequence = [];
function startCinematic() {
  if (ui.cinemaActive) { stopCinematic(false); return; }
  ui.cinemaActive = true;
  ui.cinemaStart = performance.now() / 1000;
  ui.cinemaStep = 0;
  ui.guideVisible = false;
  D('guide')?.classList.remove('show');
  closeModal(); closeAtlas(); toggleHelp(false);
  // Reset to the film's stable rated baseline. This is a deliberate demonstration
  // preset; subsequent motion still runs through the same electromechanical model.
  state.alarm = null;
  state.tripped = false;
  state.fault = 'normal';
  state.loadTarget = 100;
  state.loadPct = 100;
  state.tempC = MC.steadyCaseC;
  state.omega = SYNC_RPM * (1 - MC.ratedSlip) * RPM2RAD;
  state.powerOn = true;
  state.runCmd = true;
  state.fieldOn = true;
  state.sinceStart = 10;
  state.startTransient = 0;
  ui.fieldVisible = false;
  ui.terminalOpen = false;
  ui.selectedSensor = null;
  clearSensorSelection(true);
  clearSelection();
  setPauseMechanism(false, { silent: true });
  if (groups.bars) groups.bars.visible = true;
  syncDisplayToggleButtons();
  syncObservationUI();
  cinematicSequence = buildCinematicSequence();
  D('cinemaHud').classList.add('active');
  D('cinemaBtn')?.classList.add('primary');
  const captureCancel = (e) => {
    if (!ui.cinemaActive) return;
    stopCinematic(true);
  };
  const keyCancel = (e) => {
    if (!ui.cinemaActive) return;
    if (['Shift', 'Control', 'Alt', 'Meta'].includes(e.key)) return;
    stopCinematic(true);
  };
  ui._cinemaPointerCancel = captureCancel;
  ui._cinemaKeyCancel = keyCancel;
  document.addEventListener('pointerdown', captureCancel, true);
  document.addEventListener('keydown', keyCancel, true);
  setCameraPreset('hero', false, 1.8);
  showToast('电影演示 · 点击任意位置或操作控件可随时中断', 'info');
  updateCinemaHUD();
}

function stopCinematic(interrupted = false) {
  if (!ui.cinemaActive) return;
  ui.cinemaActive = false;
  D('cinemaHud')?.classList.remove('active');
  D('cinemaBtn')?.classList.remove('primary');
  if (ui._cinemaPointerCancel) {
    document.removeEventListener('pointerdown', ui._cinemaPointerCancel, true);
    ui._cinemaPointerCancel = null;
  }
  if (ui._cinemaKeyCancel) {
    document.removeEventListener('keydown', ui._cinemaKeyCancel, true);
    ui._cinemaKeyCancel = null;
  }
  if (interrupted) showToast('电影演示已中断 · 当前工况与用户输入已保留', 'info');
}

function updateCinematic() {
  if (!ui.cinemaActive) return;
  const elapsed = performance.now() / 1000 - ui.cinemaStart;
  while (ui.cinemaStep < cinematicSequence.length && elapsed >= cinematicSequence[ui.cinemaStep].t) {
    const step = cinematicSequence[ui.cinemaStep];
    try { step.fn(); } catch (err) { console.warn(err); }
    ui.cinemaStep++;
    setTxt('cinemaTitle', step.title);
    setTxt('cinemaSub', step.sub);
  }
  const progress = clamp(elapsed / CINEMA_DURATION, 0, 1);
  const bar = D('cinemaProgress');
  if (bar) bar.style.width = `${progress * 100}%`;
  if (elapsed >= CINEMA_DURATION) stopCinematic(false);
}

function updateCinemaHUD() {
  if (!ui.cinemaActive) return;
  const elapsed = performance.now() / 1000 - ui.cinemaStart;
  const progress = clamp(elapsed / CINEMA_DURATION, 0, 1);
  const bar = D('cinemaProgress');
  if (bar) bar.style.width = `${progress * 100}%`;
}

function syncDisplayToggleButtons() {
  $$('.display-toggle').forEach((b) => {
    const map = { fieldToggle: 'fieldVisible', barsToggle: 'barsVisible', airToggle: 'airflow', annotToggle: 'annotations' };
    const key = map[b.id];
    if (key) {
      b.classList.toggle('active', !!ui[key]);
      b.setAttribute('aria-pressed', String(!!ui[key]));
    }
  });
}

function syncObservationUI() {
  $$('.obs-control .seg button').forEach((b) => b.classList.toggle('active', b.dataset.obs === ui.observation));
  $('#manualWrap')?.classList.toggle('show', ui.observation === 'manual');
}

// ===== 48. 交互绑定 / 悬停 / 时间轴 =====
let hoverRayTime = 0;
let lastPointer = { x: 0, y: 0 };
let phaseDrag = null;

function buildPartPickList() {
  finalizePickables();
}

function bindUI() {
  bindModalEvents();

  // ---- top bar ----
  $('#qualitySelect')?.addEventListener('change', (e) => {
    const q = e.target.value;
    updateRendererQuality(q);
    if (q === 'high') ui.autoDowngraded = false;
    showToast(q === 'auto' ? '画质：自动 · 依据帧率保留机械细节' : q === 'high' ? '画质：高画质 · 阴影 / 反射优先' : '画质：流畅 · 降低阴影与粒子成本', 'info');
  });
  $('#soundBtn')?.addEventListener('click', async () => {
    await toggleAudio();
  });
  $('#volumeRange')?.addEventListener('input', (e) => {
    const v = Number(e.target.value) / 100;
    ui.volume = v;
    audioEngine.setVolume(v);
    setTxt('volumeValue', String(e.target.value));
    setRangeFill(e.target);
  });
  $$('.sound-pop [data-soundmode]').forEach((b) => b.addEventListener('click', () => {
    ui.soundMode = b.dataset.soundmode;
    audioEngine.setMode(ui.soundMode);
    $$('.sound-pop [data-soundmode]').forEach((x) => x.classList.toggle('active', x === b));
    showToast(
      ui.soundMode === 'condition' ? '慢放时保留工况音效：声音跟随真实转速，画面慢放' :
      ui.soundMode === 'event' ? '慢放时关闭连续电机音，仅保留低音量逐次提示' :
      '慢放模式静音：只保留报警提示', 'info');
  }));
  document.addEventListener('click', (e) => {
    if (!e.target.closest('#soundPop') && !e.target.closest('#soundBtn')) closeSoundPopover();
  });
  $('#atlasBtn')?.addEventListener('click', openAtlas);
  $('#atlasQuickBtn')?.addEventListener('click', openAtlas);
  $('#cinemaBtn')?.addEventListener('click', startCinematic);
  $('#presentBtn')?.addEventListener('click', () => setFocusMode(true));
  $('#helpBtn')?.addEventListener('click', () => toggleHelp(true));
  $('#helpClose')?.addEventListener('click', () => toggleHelp(false));
  $('#helpLayer')?.addEventListener('click', (e) => { if (e.target.id === 'helpLayer') toggleHelp(false); });
  $('#atlasClose')?.addEventListener('click', closeAtlas);
  $('#atlasRefresh')?.addEventListener('click', () => { drawAtlasCards(); showToast('数据图谱已刷新', 'info'); });
  $('#atlasFullscreen')?.addEventListener('click', fullscreenAtlas);
  $('#atlasLayer')?.addEventListener('click', (e) => { if (e.target.id === 'atlasLayer') closeAtlas(); });
  $('#focusExit')?.addEventListener('click', () => setFocusMode(false));

  // ---- structure / cameras ----
  $$('.view-tab').forEach((b) => b.addEventListener('click', () => {
    if (ui.cinemaActive) stopCinematic(true);
    dismissGuide();
    setStructureFromTab(b.dataset.structure);
  }));
  $$('.cam-chip').forEach((b) => b.addEventListener('click', () => {
    if (ui.cinemaActive) stopCinematic(true);
    dismissGuide();
    applyCameraPreset(b.dataset.camera);
  }));
  $('#cameraSelect')?.addEventListener('change', (e) => {
    if (ui.cinemaActive) stopCinematic(true);
    dismissGuide();
    applyCameraPreset(e.target.value);
  });
  $('#resetViewBtn')?.addEventListener('click', resetView);
  $('#fieldQuickBtn')?.addEventListener('click', () => toggleField());
  $('#annotQuickBtn')?.addEventListener('click', () => toggleAnnotations());

  // ---- transport ----
  $('#powerBtn')?.addEventListener('click', () => setPower(!state.powerOn));
  $('#startBtn')?.addEventListener('click', startMotor);
  $('#stopBtn')?.addEventListener('click', stopMotor);
  $('#pauseBtn')?.addEventListener('click', () => setPauseMechanism(!ui.paused));
  const loadRange = $('#loadRange');
  loadRange?.addEventListener('pointerdown', () => { ui.loadDragging = true; });
  loadRange?.addEventListener('input', (e) => {
    state.loadTarget = Number(e.target.value);
    setRangeFill(e.target);
    refreshAllUI(true);
  });
  loadRange?.addEventListener('change', () => { ui.loadDragging = false; });
  window.addEventListener('pointerup', () => { ui.loadDragging = false; });
  $$('.obs-control .seg button').forEach((b) => b.addEventListener('click', () => setObservationMode(b.dataset.obs)));
  $('#manualScale')?.addEventListener('input', (e) => {
    ui.manualScale = Number(e.target.value) / 100;
    setTxt('manualScaleValue', `${ui.manualScale.toFixed(3)}×`);
    setRangeFill(e.target);
    if (ui.observation !== 'manual') setObservationMode('manual', true);
    refreshAllUI(true);
  });

  // ---- scenario ----
  $$('.condition').forEach((b) => b.addEventListener('click', () => { dismissGuide(); injectFault(b.dataset.fault); }));
  $('#thermalAccel')?.addEventListener('change', (e) => {
    refreshAllUI(true);
    showToast(e.target.checked ? '热过程演示 ×10 加速已启用' : '热过程已切换为真实时间尺度', e.target.checked ? 'warn' : 'info');
  });
  $('#fieldToggle')?.addEventListener('click', () => toggleField());
  $('#barsToggle')?.addEventListener('click', toggleBars);
  $('#airToggle')?.addEventListener('click', toggleAirflow);
  $('#annotToggle')?.addEventListener('click', () => toggleAnnotations());
  $('#terminalBtn')?.addEventListener('click', () => toggleTerminal());

  // ---- sensors / sheets ----
  $('#sensorSheetClose')?.addEventListener('click', closeSensorSheet);
  $('#sensorFocusBtn')?.addEventListener('click', () => {
    if (!ui.selectedSensor) return;
    const sensor = ui.selectedSensor;
    if (ui.structure === 'assembly') setStructure('section');
    const meta = PART_META[sensor.partId];
    if (meta) {
      focusCamera(meta.focus || sensor.camera.pos, meta.target || sensor.camera.target);
    }
    showToast(`聚焦测量部位 · ${sensor.part}`, 'info');
  });
  $('#partSheetClose')?.addEventListener('click', clearSelection);
  $('#partSheetFocusBtn')?.addEventListener('click', () => {
    if (ui.selectedPart) focusPart(ui.selectedPart);
  });
  $('#terminalClose')?.addEventListener('click', () => { showTerminalCard(false); });
  $$('.connection-switch button').forEach((b) => b.addEventListener('click', () => setConnection(b.dataset.conn)));
  $('#alarmResetBtn')?.addEventListener('click', resetAlarmAndFault);

  // ---- timeline ----
  const phaseTrack = $('#phaseTrack');
  if (phaseTrack) {
    phaseTrack.tabIndex = 0;
    phaseTrack.setAttribute('role', 'slider');
    phaseTrack.setAttribute('aria-label', '机构相位时间轴');
    phaseTrack.addEventListener('pointerdown', (e) => {
      if (ui.cinemaActive) stopCinematic(true);
      phaseDrag = { wasPaused: ui.paused };
      setPauseMechanism(true, { silent: true });
      phaseTrack.setPointerCapture?.(e.pointerId);
      updatePhaseFromEvent(e);
      e.preventDefault();
    });
    phaseTrack.addEventListener('pointermove', (e) => {
      if (!phaseDrag) return;
      updatePhaseFromEvent(e);
    });
    const end = () => {
      if (!phaseDrag) return;
      const wasPaused = phaseDrag.wasPaused;
      phaseDrag = null;
      if (!wasPaused) setPauseMechanism(false, { silent: true });
    };
    phaseTrack.addEventListener('pointerup', end);
    phaseTrack.addEventListener('pointercancel', end);
    phaseTrack.addEventListener('keydown', (e) => {
      if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') {
        e.preventDefault();
        nudgePhase(e.key === 'ArrowLeft' ? -5 : 5);
      }
    });
  }
  $$('.step-btns button').forEach((b) => b.addEventListener('click', () => nudgePhase(Number(b.dataset.step))));

  // ---- guide ----
  $('#guideNext')?.addEventListener('click', advanceGuide);
  $('#guideClose')?.addEventListener('click', dismissGuide);

  // ---- hover / canvas ----
  canvasElement?.addEventListener('pointermove', handleHoverMove, { passive: true });
  canvasElement?.addEventListener('pointerleave', hideTooltip);
  canvasElement?.addEventListener('pointerdown', hideTooltip, { passive: true });

  // accessibility for data cards
  $$('[data-open-card]').forEach((el) => {
    if (!el.matches('button')) { el.tabIndex = 0; el.setAttribute('role', 'button'); }
  });
  const featureGrid = $('#featureGrid');
  if (featureGrid) featureGrid.setAttribute('aria-label', '放大六特征实时数值表');

  // initial visual state
  document.querySelectorAll('.range').forEach(setRangeFill);
  setRangeFill($('#volumeRange'));
  setTxt('volumeValue', $('#volumeRange')?.value || '42');
  audioEngine.setVolume(ui.volume);
  syncObservationUI();
  syncDisplayToggleButtons();
  updateSensorTagVisibility();
  syncStructureTabs();
  setPauseMechanism(false, { silent: true });
  setRangeFill($('#loadRange'));
  setRangeFill($('#manualScale'));
  $$('.sound-pop [data-soundmode]').forEach((b) => b.classList.toggle('active', b.dataset.soundmode === ui.soundMode));
  refreshAllUI(true);
}

function updatePhaseFromEvent(e) {
  const track = e.currentTarget || $('#phaseTrack');
  if (!track) return;
  const rect = track.getBoundingClientRect();
  const f = clamp((e.clientX - rect.left) / rect.width, 0, 1);
  const targetFieldMech = (f * TAU) / MC.polePairs;
  let delta = targetFieldMech - state.displayFieldAngle;
  delta = ((delta + Math.PI) % TAU + TAU) % TAU - Math.PI;
  state.displayFieldAngle = wrapAngle(state.displayFieldAngle + delta);
  state.displayRotorAngle = wrapAngle(state.displayRotorAngle + delta);
  updateVisualAnglesNow();
}

function nudgePhase(degrees) {
  const delta = degrees * DEG2RAD / MC.polePairs;
  state.displayFieldAngle = wrapAngle(state.displayFieldAngle + delta);
  state.displayRotorAngle = wrapAngle(state.displayRotorAngle + delta);
  updateVisualAnglesNow();
}

function updateVisualAnglesNow() {
  if (groups.rotorSpin) groups.rotorSpin.rotation.z = state.displayRotorAngle;
  if (groups.fan) groups.fan.rotation.z = state.displayRotorAngle;
  if (groups.field) groups.field.rotation.z = state.displayFieldAngle;
  if (groups.currentArrows) {
    const mat = groups.currentArrows.userData.material;
    if (mat) mat.opacity = 0.8;
  }
  updateFieldVisuals(0.016);
  updateAnnotations();
  refreshAllUI(true);
}

function handleHoverMove(event) {
  if (!sceneReady || ui.cinemaActive || orbitRig?.dragging) { hideTooltip(); return; }
  if (document.body.classList.contains('atlas-open') || ui.modalCardId) return;
  const now = performance.now();
  if (now - hoverRayTime < 70) { positionTooltip(event); return; }
  hoverRayTime = now;
  lastPointer = { x: event.clientX, y: event.clientY };
  const rect = canvasElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1
  );
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(ndc, camera);
  const hits = raycaster.intersectObjects(allPickables, false);
  const hit = hits.find((h) => isActuallyVisible(h.object));
  if (!hit) {
    hideTooltip();
    return;
  }
  const partId = hit.object.userData.partId;
  if (!partId) { hideTooltip(); return; }
  showPartTooltip(partId, event);
}

function showPartTooltip(partId, event) {
  const meta = PART_META[partId];
  if (!meta) { hideTooltip(); return; }
  ui.hoverPartId = partId;
  const sensor = SENSORS.find((s) => s.id === partId);
  const reading = sensor ? getSensorReading(sensor.id) : null;
  const type = sensor ? `SENSOR · ${sensor.code}` : meta.type;
  const name = sensor ? sensor.name : meta.name;
  const en = sensor ? sensor.en : meta.en;
  const desc = sensor ? sensor.principle : meta.desc;
  const specs = sensor ? [`量程 ${sensor.range.split('·')[0].trim()}`, sensor.part, '50 Hz'] : meta.specs;
  const motion = sensor
    ? `实时读数 <b>${reading.display} ${sensor.unit}</b> · ${reading.status === 'alarm' ? '越限' : reading.status === 'warn' ? '预警' : '正常'}`
    : partMotionText(partId);
  const tip = D('partTooltip');
  if (!tip) return;
  tip.innerHTML = `
    <div class="tt-top"><div><div class="tt-name">${name}</div><div class="tt-en">${en}</div></div><span class="tt-type">${type}</span></div>
    <div class="tt-desc">${desc}</div>
    <div class="tt-meta"><span>材料 · ${meta.material}</span>${specs.map((s) => `<span>${s}</span>`).join('')}</div>
    <div class="tt-motion">当前状态 <b>${motion}</b></div>`;
  tip.classList.add('show');
  positionTooltip(event);
  if (canvasElement) canvasElement.style.cursor = 'pointer';
}

function positionTooltip(event) {
  const tip = D('partTooltip');
  if (!tip || !tip.classList.contains('show')) return;
  const stageRect = stageElement.getBoundingClientRect();
  const x = event.clientX - stageRect.left;
  const y = event.clientY - stageRect.top;
  const tw = 242;
  const th = tip.offsetHeight || 130;
  let left = x + 16;
  let top = y + 16;
  if (left + tw > stageRect.width - 10) left = x - tw - 16;
  if (top + th > stageRect.height - 10) top = y - th - 16;
  left = clamp(left, 10, stageRect.width - tw - 10);
  top = clamp(top, 10, stageRect.height - th - 10);
  tip.style.left = `${left}px`;
  tip.style.top = `${top}px`;
}

function hideTooltip() {
  ui.hoverPartId = null;
  D('partTooltip')?.classList.remove('show');
  if (canvasElement && !orbitRig?.dragging) canvasElement.style.cursor = 'grab';
}

function applyCameraPreset(name) {
  if (!['hero', 'section', 'field', 'rotor', 'sensors'].includes(name)) return;
  if (name !== 'field' && ui.fieldViewActive) {
    ui.fieldViewActive = false;
    setObservationMode(ui.observationBeforeField || 'auto', true);
  }
  if (name === 'hero') {
    setStructure('assembly', 'camera');
    setCameraPreset('hero');
  } else if (name === 'section') {
    if (ui.structure !== 'section') setStructure('section', 'camera');
    setCameraPreset('section');
  } else if (name === 'field') {
    if (!ui.fieldViewActive) ui.observationBeforeField = ui.observation;
    ui.fieldViewActive = true;
    if (ui.structure !== 'section') setStructure('section', 'camera');
    ui.fieldVisible = true;
    setObservationMode('ultra', true);
    syncObservationUI();
    syncDisplayToggleButtons();
    setCameraPreset('field');
    showToast('旋转磁场观察 · 超慢 12 r/min · 磁场以 12.9 r/min 继续领先', 'info');
  } else if (name === 'rotor') {
    setStructure('exploded', 'camera');
    setCameraPreset('rotor');
    ui.rotorExtract = true;
    updateCameraChips();
    setActiveCameraChip('rotor');
  } else if (name === 'sensors') {
    if (ui.structure === 'exploded') setStructure('assembly', 'camera');
    setCameraPreset('sensors');
    setActiveCameraChip('sensors');
    updateSensorTagVisibility();
    showToast('五路传感器布局 · 点击任意安装点查看测量链', 'info');
  }
  refreshAllUI(true);
}

async function toggleAudio(force = null) {
  const on = force === null ? !ui.audioEnabled : !!force;
  if (on && !ui.audioEnabled) {
    const enabled = await audioEngine.toggle();
    ui.audioEnabled = enabled;
    if (enabled) openSoundPopover();
  } else if (!on && ui.audioEnabled) {
    audioEngine.disable();
    ui.audioEnabled = false;
    closeSoundPopover();
  }
  const btn = D('soundBtn');
  if (btn) {
    btn.setAttribute('aria-pressed', String(ui.audioEnabled));
    btn.style.color = ui.audioEnabled ? 'var(--cyan)' : '';
  }
  showToast(ui.audioEnabled ? '声音已开启 · 100 Hz 电磁嗡鸣 + 风扇气流' : '声音已关闭', 'info');
}

function setFocusMode(on) {
  ui.focusMode = !!on;
  document.body.classList.toggle('focus-mode', ui.focusMode);
  setTimeout(resizeRenderer, 60);
  if (ui.focusMode) {
    hideTooltip();
    showToast('答辩模式 · 仅保留 3D 视图，ESC / 按钮退出', 'info');
  }
}

// ===== 49. 键盘快捷键 =====
function bindKeyboard() {
  document.addEventListener('keydown', (e) => {
    const tag = (e.target?.tagName || '').toLowerCase();
    const editable = ['input', 'select', 'textarea'].includes(tag) || e.target?.isContentEditable;
    if (e.key === 'Escape') {
      if (D('helpLayer')?.classList.contains('open')) { toggleHelp(false); return; }
      if (ui.modalCardId) { closeModal(); return; }
      if (ui.atlasOpen) { closeAtlas(); return; }
      if (ui.focusMode) { setFocusMode(false); return; }
      if (ui.selectedSensor) { closeSensorSheet(); return; }
      if (ui.terminalOpen) { showTerminalCard(false); ui.terminalOpen = false; return; }
      return;
    }
    if (editable) return;
    if (ui.modalCardId || ui.atlasOpen || D('helpLayer')?.classList.contains('open')) return;
    const buttonFocused = tag === 'button' || !!e.target?.closest?.('button,[role="button"]');
    if (e.key === ' ' && !e.repeat) {
      if (buttonFocused) return;
      e.preventDefault();
      setPauseMechanism(!ui.paused);
      return;
    }
    if (e.repeat) return;
    const key = e.key.toLowerCase();
    if (key === 'r') { resetView(); }
    else if (key === 'c') { setStructureFromTab(ui.structure === 'section' ? 'assembly' : 'section'); }
    else if (key === 'e') { setStructureFromTab(ui.structure === 'exploded' ? 'assembly' : 'exploded'); }
    else if (key === 'f') { toggleField(); }
    else if (key === 'm') { toggleAudio(); }
    else if (key === 'p') { setFocusMode(!ui.focusMode); }
    else if (e.shiftKey && key === 'd') { toggleDebugPanel(); }
    else if (e.key === '?') { toggleHelp(true); }
  });
}

function resetView() {
  if (ui.cinemaActive) stopCinematic(true);
  if (ui.structure === 'exploded') setStructure('assembly', 'camera');
  setCameraPreset('hero', false, 1.15);
  setActiveCameraChip('hero');
  refreshAllUI(true);
}

function toggleDebugPanel() {
  ui.debug = !ui.debug;
  D('debugPanel')?.classList.toggle('show', ui.debug);
  if (ui.debug) updateDebugPanel();
}

function updateDebugPanel(force = false) {
  if (!ui.debug) return;
  const el = D('debugPanel');
  if (!el) return;
  const now = performance.now();
  if (!force && now - (ui._debugLast || 0) < 220) return;
  ui._debugLast = now;
  const info = renderer?.info?.render || { calls: 0, triangles: 0 };
  el.innerHTML = `<h5>DEV DIAGNOSTICS · SHIFT+D</h5>
    <div class="d-grid">
      <span>FPS <b>${ui.fps.toFixed(0)}</b></span><span>DPR <b>${currentPixelRatio.toFixed(2)}</b></span>
      <span>draw calls <b>${info.calls}</b></span><span>tris <b>${(info.triangles || 0).toLocaleString()}</b></span>
      <span>n actual <b>${state.nRpm.toFixed(1)}</b></span><span>n display <b>${state.rotorDisplayRpm.toFixed(1)}</b></span>
      <span>field display <b>${state.fieldDisplayRpm.toFixed(1)}</b></span><span>scale <b>${state.displayScale.toFixed(4)}</b></span>
      <span>θ rotor <b>${(state.displayRotorAngle * RAD2DEG).toFixed(1)}°</b></span><span>θ field <b>${(state.displayFieldAngle * RAD2DEG).toFixed(1)}°</b></span>
      <span>slip <b>${state.slip.toFixed(4)}</b></span><span>fault <b>${state.fault}</b></span>
      <span>I <b>${state.currentA.toFixed(3)} A</b></span><span>T <b>${state.tempC.toFixed(2)} ℃</b></span>
      <span>alarm <b>${state.alarm ? state.alarm.phase || 'latched' : 'none'}</b></span><span>quality <b>${ui.quality}${ui.autoDowngraded ? '+d' : ''}</b></span>
    </div>
    <div class="d-feat">x = [ Imean ${state.feature.currentMean.toFixed(3)}, var ${state.feature.currentVar.toFixed(5)}, kurt ${state.feature.currentKurt.toFixed(2)}, dn/dt ${state.feature.speedRate.toFixed(2)}, err ${state.feature.followError.toFixed(1)}, dT/dt ${state.feature.tempSlope.toFixed(3)} ]\nf(x) = ${state.svmScore.toFixed(3)} · conf ${(state.svmConfidence * 100).toFixed(1)}%</div>`;
}

// ===== 50. 视觉效果脉冲 / 主循环 =====
function updateHighlightAndSignalPulse(dt) {
  if (groups.sensorHighlights) {
    for (const item of highlightObjects.values()) {
      if (!item.object.visible) continue;
      item.material.opacity = 0.16 + 0.10 * (0.5 + 0.5 * Math.sin(state.simTime * 4.2));
    }
  }
  if (groups.signalPaths) {
    for (const key in groups.signalPaths) {
      const g = groups.signalPaths[key];
      if (!g.visible) continue;
      const { curve, pulse, line } = g.userData;
      const t = fract(state.simTime * 0.33);
      pulse.position.copy(curve.getPoint(t));
      pulse.material.opacity = 0.42 + 0.25 * Math.sin(state.simTime * 6);
      const reading = getSensorReading(key);
      const baseColor = SENSOR_MAP[key]?.color || '#70d8e5';
      const color = reading.status === 'alarm' ? '#ff5a4f' : reading.status === 'warn' ? '#ffab5e' : baseColor;
      line.material.color.set(color);
      pulse.material.map = makeGlowTexture(color);
    }
  }
  if (groups.sensorHalos) {
    for (const sensor of SENSORS) {
      const halo = groups.sensorHalos[sensor.id];
      if (!halo) continue;
      const selected = ui.selectedSensor?.id === sensor.id;
      const target = selected ? 0.26 + 0.16 * Math.sin(state.simTime * 5) : (ui.cameraPreset === 'sensors' ? 0.10 + 0.04 * Math.sin(state.simTime * 2.7 + SENSORS.indexOf(sensor)) : 0);
      halo.material.opacity = damp(halo.material.opacity, target, 7, dt);
    }
  }
  if (groups.currentArrows?.visible) {
    groups.currentArrows.userData.material.opacity = 0.48 + 0.35 * (0.5 + 0.5 * Math.sin(state.simTime * 4.2));
  }
  if (dust) {
    dust.rotation.y += dt * 0.003;
  }
  updateSelectionBox();
}

let lastFrame = performance.now();
let physicsAccumulator = 0;
let nextTrendAt = 0;
let nextUiAt = 0;
let nextChartAt = 0;
let nextFeatureAt = 0;
let nextAtlasAt = 0;
let fpsSampleTime = 0;
let fpsSampleFrames = 0;
let bootHidden = false;
let lowFpsTime = 0;
const FIXED_DT = 1 / 50;

function nowSeconds() { return performance.now() / 1000; }

function animationLoop(nowMs) {
  requestAnimationFrame(animationLoop);
  if (!sceneReady) return;
  const now = nowMs / 1000;
  let rawDt = now - (ui.lastFrameTime || now);
  ui.lastFrameTime = now;
  if (!Number.isFinite(rawDt) || rawDt < 0) rawDt = 0;
  // Keep wall-clock and simulation aligned even on low-frame-rate machines:
  // the fixed 50 Hz integrator may catch up in bounded substeps, while a long
  // background stall is discarded rather than replayed as a spiral.
  const dt = Math.min(rawDt, 0.35);
  const cameraDt = Math.min(rawDt, 0.22);

  // FPS / auto quality
  fpsSampleFrames++;
  if (!fpsSampleTime) fpsSampleTime = now;
  if (now - fpsSampleTime >= 1) {
    ui.fps = fpsSampleFrames / (now - fpsSampleTime);
    fpsSampleFrames = 0;
    fpsSampleTime = now;
    if (ui.quality === 'auto' && !ui.autoDowngraded) {
      if (ui.fps < 42) lowFpsTime += 1; else lowFpsTime = Math.max(0, lowFpsTime - 0.5);
      if (lowFpsTime >= 4) applyAutoPerfDownscale();
    }
  }

  if (document.hidden) {
    renderer.render(scene, camera);
    return;
  }

  // 50 Hz physical model and 50 Hz acquisition; no independent random animation.
  physicsAccumulator += dt;
  let substeps = 0;
  while (physicsAccumulator >= FIXED_DT && substeps < 18) {
    updatePhysics(FIXED_DT);
    pushSample(true);
    physicsAccumulator -= FIXED_DT;
    substeps++;
  }
  if (physicsAccumulator > 0.2) physicsAccumulator = 0.2;
  if (substeps === 0 && dt === 0) { /* first frame */ }
  if (state.simTime >= nextTrendAt) {
    nextTrendAt = state.simTime + 0.10;
    state.trend.push({ t: state.simTime, I: state.currentA, n: state.nRpm, T: state.tempC });
    while (state.trend.length > 780) state.trend.shift();
  }
  if (state.simTime >= nextFeatureAt) {
    nextFeatureAt = state.simTime + 0.18;
    computeFeatures();
  }
  updateMechanism(dt);
  updateCinematic();
  updateHighlightAndSignalPulse(dt);
  orbitRig.update(cameraDt);
  updateAnnotations();
  updateSensorTagVisibility();
  updateAlarmBeacon();

  if (now >= nextUiAt) {
    nextUiAt = now + 0.105;
    refreshAllUI();
  }
  if (now >= nextChartAt) {
    nextChartAt = now + 0.22;
    drawAllMiniCharts();
    if (ui.atlasOpen) drawAtlasCards();
    if (ui.modalCardId && !ui.modalPaused) drawModal();
  }
  updateDebugPanel();
  if (!bootHidden) {
    bootHidden = true;
    requestAnimationFrame(() => {
      setTimeout(() => D('boot-screen')?.classList.add('done'), 160);
    });
  }
  try { renderer.render(scene, camera); } catch (err) { console.error(err); }
}

function applyAutoPerfDownscale() {
  if (!ui.autoDowngraded) {
    ui.autoDowngraded = true;
    currentPixelRatio = Math.min(window.devicePixelRatio || 1, 1.25);
    renderer.setPixelRatio(currentPixelRatio);
    renderer.setSize(stageW, stageH, false);
    renderer.shadowMap.enabled = false;
    if (keyLight) keyLight.castShadow = false;
    if (dust) dust.visible = false;
    showToast('自动画质：检测到持续低帧率，已降低阴影 / 像素比 / 粒子成本', 'info');
  }
}

// ===== 51. 启动 =====
function startApplication() {
  const ok = initThree();
  if (!ok) return;
  // Development handle. Deliberately undocumented in the visitor UI; provides
  // read/write access for automated visual regression and physics checks.
  window.__heartOfDrive = {
    THREE, scene, state, ui, MC, SENSORS, svmModel, groups, camera,
    setStructure, setCameraPreset, injectFault, setPower, startMotor, stopMotor,
    resetAlarmAndFault, computeFeatures, getSensorReading, setPauseMechanism,
    applyCameraPreset, setObservationMode, toggleField, selectPart, selectSensor, focusPart,
    advanceSimulation(seconds) {
      const steps = Math.max(1, Math.round(seconds / FIXED_DT));
      for (let i = 0; i < steps; i++) {
        updatePhysics(FIXED_DT);
        pushSample(true);
      }
      computeFeatures();
      refreshAllUI(true);
      return { t: state.simTime, I: state.currentA, n: state.nRpm, temp: state.tempC };
    }
  };
  bindUI();
  bindKeyboard();
  window.addEventListener('resize', resizeRenderer);
  if ('ResizeObserver' in window) {
    const ro = new ResizeObserver(() => resizeRenderer());
    ro.observe(stageElement);
  }
  document.addEventListener('visibilitychange', () => {
    ui.lastFrameTime = performance.now() / 1000;
    physicsAccumulator = 0;
  });
  // initial cards
  requestAnimationFrame(() => {
    drawAllMiniCharts(true);
    refreshAllUI(true);
    drawAtlasCards();
  });
  setTimeout(() => {
    if (!ui.guideVisible && !ui.cinemaActive) showGuide();
    setTimeout(() => { if (ui.guideVisible && !ui.cinemaActive) dismissGuide(); }, 24000);
  }, 1100);
  requestAnimationFrame((t) => {
    ui.lastFrameTime = t / 1000;
    animationLoop(t);
  });
}

window.addEventListener('error', (event) => {
  console.error('Uncaught application error:', event.error || event.message);
  const fallback = document.getElementById('fallback');
  if (fallback && !sceneReady) {
    fallback.classList.add('show');
    const p = document.getElementById('fallbackText');
    if (p) p.textContent = `初始化失败：${event.message || event.error || '未知错误'}`;
    document.getElementById('boot-screen')?.classList.add('done');
  }
});

startApplication();
