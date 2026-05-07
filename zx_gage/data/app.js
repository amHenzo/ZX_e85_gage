const els = {
  status: document.getElementById('status'),
  frontG: document.getElementById('frontG'),
  rearG: document.getElementById('rearG'),
  leftG: document.getElementById('leftG'),
  rightG: document.getElementById('rightG'),
  zG: document.getElementById('zG'),
  temp: document.getElementById('temp'),
  hum: document.getElementById('hum'),
  fps: document.getElementById('fps')
};

const graph = document.getElementById('graph');
const graphCtx = graph.getContext('2d');
const gauge = document.getElementById('gauge');
const gaugeCtx = gauge.getContext('2d');
const samples = [];
const maxSamples = 240;
const cacheKey = 'zx-gage-samples-v2';

function fixed(value, digits) {
  return Number(value || 0).toFixed(digits);
}

function forces(sample) {
  const lateral = -(sample.x || 0);
  const frontRear = -(sample.y || 0);
  return {
    left: lateral < 0 ? -lateral : 0,
    right: lateral > 0 ? lateral : 0,
    front: frontRear > 0 ? frontRear : 0,
    rear: frontRear < 0 ? -frontRear : 0
  };
}

function saveSamples() {
  try {
    localStorage.setItem(cacheKey, JSON.stringify(samples));
  } catch (e) {
  }
}

function updateValues(sample) {
  if (!sample) return;
  const g = forces(sample);
  els.frontG.textContent = fixed(g.front, 2);
  els.rearG.textContent = fixed(g.rear, 2);
  els.leftG.textContent = fixed(g.left, 2);
  els.rightG.textContent = fixed(g.right, 2);
  els.zG.textContent = fixed(sample.z, 2);
  els.temp.textContent = fixed(sample.temp, 1);
  els.hum.textContent = fixed(sample.hum, 1);
  els.fps.textContent = fixed(sample.fps, 1);
  drawGauge(sample);
}

function pushSample(sample) {
  sample.t = Date.now();
  samples.push(sample);
  if (samples.length > maxSamples) samples.shift();
  updateValues(sample);
  saveSamples();
  drawGraph();
}

function drawGauge(sample) {
  const w = gauge.width;
  const h = gauge.height;
  const cx = w / 2;
  const cy = h / 2;
  const r = Math.min(w, h) * 0.42;
  const x = Math.max(-1, Math.min(1, -(sample.x || 0)));
  const y = Math.max(-1, Math.min(1, -(sample.y || 0)));

  gaugeCtx.clearRect(0, 0, w, h);
  gaugeCtx.strokeStyle = '#2f363d';
  gaugeCtx.lineWidth = 1;
  gaugeCtx.beginPath();
  gaugeCtx.arc(cx, cy, r, 0, Math.PI * 2);
  gaugeCtx.moveTo(cx - r, cy);
  gaugeCtx.lineTo(cx + r, cy);
  gaugeCtx.moveTo(cx, cy - r);
  gaugeCtx.lineTo(cx, cy + r);
  gaugeCtx.stroke();

  gaugeCtx.fillStyle = '#9ca8b3';
  gaugeCtx.font = '12px system-ui';
  gaugeCtx.textAlign = 'center';
  gaugeCtx.fillText('Front', cx, cy - r - 8);
  gaugeCtx.fillText('Rear', cx, cy + r + 16);
  gaugeCtx.textAlign = 'left';
  gaugeCtx.fillText('Left', cx - r - 28, cy + 4);
  gaugeCtx.fillText('Right', cx + r + 8, cy + 4);

  gaugeCtx.fillStyle = '#f5f5f5';
  gaugeCtx.beginPath();
  gaugeCtx.arc(cx + x * r, cy - y * r, 5, 0, Math.PI * 2);
  gaugeCtx.fill();
}

function drawGraph() {
  const w = graph.width;
  const h = graph.height;
  const pad = 26;
  graphCtx.clearRect(0, 0, w, h);
  graphCtx.strokeStyle = '#263039';
  graphCtx.lineWidth = 1;
  graphCtx.beginPath();
  for (let g = -2; g <= 2; g++) {
    const y = pad + (2 - g) * (h - pad * 2) / 4;
    graphCtx.moveTo(pad, y);
    graphCtx.lineTo(w - pad, y);
  }
  graphCtx.stroke();
  graphCtx.fillStyle = '#9ca8b3';
  graphCtx.font = '12px system-ui';
  graphCtx.fillText('+2G', 4, pad + 4);
  graphCtx.fillText('0G', 7, h / 2 + 4);
  graphCtx.fillText('-2G', 4, h - pad + 4);
  drawLine('x', '#f4d35e');
  drawLine('y', '#ee6c4d');
  drawLine('z', '#7bdff2');
}

function drawLine(key, color) {
  if (samples.length < 2) return;
  const w = graph.width;
  const h = graph.height;
  const pad = 26;
  graphCtx.strokeStyle = color;
  graphCtx.lineWidth = 2;
  graphCtx.beginPath();
  samples.forEach((sample, i) => {
    const x = pad + i * (w - pad * 2) / (maxSamples - 1);
    const clamped = Math.max(-2, Math.min(2, sample[key] || 0));
    const y = pad + (2 - clamped) * (h - pad * 2) / 4;
    if (i === 0) graphCtx.moveTo(x, y);
    else graphCtx.lineTo(x, y);
  });
  graphCtx.stroke();
}

document.getElementById('download').addEventListener('click', () => {
  const rows = ['time_ms,x_g,y_g,z_g,temp_c,humidity_pct,fps'];
  samples.forEach(sample => rows.push([
    sample.t,
    sample.x,
    sample.y,
    sample.z,
    sample.temp,
    sample.hum,
    sample.fps
  ].join(',')));
  const blob = new Blob([rows.join('\n')], { type: 'text/csv' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'zx-gage-log.csv';
  a.click();
  URL.revokeObjectURL(a.href);
});

const events = new EventSource('/events');
events.addEventListener('open', () => { els.status.textContent = 'live'; });
events.addEventListener('error', () => { els.status.textContent = 'reconnecting'; });
events.addEventListener('state', event => pushSample(JSON.parse(event.data)));

try {
  const cached = JSON.parse(localStorage.getItem(cacheKey) || '[]');
  cached.slice(-maxSamples).forEach(sample => samples.push(sample));
  updateValues(samples[samples.length - 1]);
} catch (e) {
}

drawGauge({ x: 0, y: 0 });
drawGraph();
