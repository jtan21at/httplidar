use axum::{extract::State, response::Html, routing::get, Json, Router};
use lidar_ld19::detect::{analyze, Classifier, Cluster};
use lidar_ld19::{LD19, DIR_ROUND};
use serde::Serialize;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const PORT: &str = "/dev/ttyUSB0";
const MAX_RANGE_M: f64 = 4.0;
const CALIBRATION: Duration = Duration::from_secs(30);
const ANGLE_BINS: usize = 360;
const MATCH_TOLERANCE_M: f64 = 0.15;
const MIN_BG_SAMPLES: u32 = 5;
const MODEL_PATH: &str = "human_rf.bin";
const TRACK_MATCH_M: f64 = 0.40;
const TRACK_TTL_SCANS: u32 = 30;
const BIND_ADDR: &str = "0.0.0.0:8080";

// ── JSON types ────────────────────────────────────────────────────────────────

#[derive(Clone, Serialize, Default)]
struct ScanSnapshot {
    phase: String,
    calibration_progress: f64,
    scan_number: u64,
    timestamp_ms: u64,
    /// [x_m, y_m, is_motion (0.0 or 1.0)]
    points: Vec<[f64; 3]>,
    /// 360 background distance bins; null = insufficient calibration samples
    background: Vec<Option<f64>>,
    clusters: Vec<ClusterJson>,
    humans_detected: usize,
}

#[derive(Clone, Serialize)]
struct ClusterJson {
    centroid: [f64; 2],
    is_human: bool,
    features: FeaturesJson,
    points: Vec<[f64; 2]>,
}

#[derive(Clone, Serialize)]
struct FeaturesJson {
    width: f64,
    depth: f64,
    curvature: f64,
    point_count: f64,
}

// ── Internal state ────────────────────────────────────────────────────────────

type Shared = Arc<RwLock<ScanSnapshot>>;

enum Phase {
    Calibrating { started: Instant, samples: Vec<Vec<f64>> },
    Detecting { background: Vec<Option<f64>> },
}

struct HumanTrack {
    centroid: (f64, f64),
    missed: u32,
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() {
    env_logger::init();

    let shared: Shared = Arc::new(RwLock::new(ScanSnapshot::default()));

    let shared_lidar = shared.clone();
    thread::spawn(move || lidar_thread(shared_lidar));

    let app = Router::new()
        .route("/", get(index))
        .route("/api/scan", get(scan_handler))
        .with_state(shared);

    let listener = tokio::net::TcpListener::bind(BIND_ADDR)
        .await
        .expect("Failed to bind");
    println!("Listening on http://{BIND_ADDR}  (open in a browser on any device)");

    axum::serve(listener, app).await.unwrap();
}

async fn index() -> Html<&'static str> {
    Html(HTML)
}

async fn scan_handler(State(shared): State<Shared>) -> Json<ScanSnapshot> {
    Json(shared.read().unwrap().clone())
}

// ── LIDAR thread ──────────────────────────────────────────────────────────────

fn lidar_thread(shared: Shared) {
    let mut lidar = match LD19::open(PORT) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("Cannot open {PORT}: {e}");
            return;
        }
    };

    let classifier = Classifier::load_or_fallback(MODEL_PATH);
    let mut phase = Phase::Calibrating {
        started: Instant::now(),
        samples: vec![Vec::new(); ANGLE_BINS],
    };
    let mut raw_points: Vec<(f64, f64, bool)> = Vec::with_capacity(500);
    let mut last_dir: u16 = u16::MAX;
    let mut scan_number: u64 = 0;
    let mut tracks: Vec<HumanTrack> = Vec::new();

    println!("Calibrating for {}s — keep scene static...", CALIBRATION.as_secs());

    loop {
        for raw in lidar.poll() {
            let wrapped = last_dir != u16::MAX
                && raw.dir < last_dir
                && last_dir > DIR_ROUND / 2;
            last_dir = raw.dir;

            if wrapped {
                scan_number += 1;
                phase = finish_calibration_if_ready(phase);

                let snap = build_snapshot(&raw_points, &mut phase, &classifier, &mut tracks, scan_number);
                *shared.write().unwrap() = snap;
                raw_points.clear();
            }

            if raw.len == 0 {
                continue;
            }
            let meters = raw.len as f64 / 1000.0;
            if meters > MAX_RANGE_M {
                continue;
            }
            let deg = raw.dir as f64 / 100.0;
            let bin = (deg.floor() as usize) % ANGLE_BINS;
            let rad = deg.to_radians();
            let x = rad.cos() * meters;
            let y = rad.sin() * meters;

            let is_motion = match &mut phase {
                Phase::Calibrating { samples, .. } => {
                    samples[bin].push(meters);
                    false
                }
                Phase::Detecting { background } => match background[bin] {
                    Some(bg) => (meters - bg).abs() > MATCH_TOLERANCE_M,
                    None => true,
                },
            };
            raw_points.push((x, y, is_motion));
        }
    }
}

fn finish_calibration_if_ready(phase: Phase) -> Phase {
    match phase {
        Phase::Calibrating { started, samples } if started.elapsed() >= CALIBRATION => {
            let background: Vec<Option<f64>> = samples
                .iter()
                .map(|s| {
                    if (s.len() as u32) < MIN_BG_SAMPLES {
                        return None;
                    }
                    let mut v = s.clone();
                    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    Some(v[v.len() / 2])
                })
                .collect();
            let locked = background.iter().filter(|b| b.is_some()).count();
            println!("Calibration done — {}/{} bins locked. Motion detection active.", locked, ANGLE_BINS);
            Phase::Detecting { background }
        }
        p => p,
    }
}

fn build_snapshot(
    raw_points: &[(f64, f64, bool)],
    phase: &mut Phase,
    classifier: &Classifier,
    tracks: &mut Vec<HumanTrack>,
    scan_number: u64,
) -> ScanSnapshot {
    let timestamp_ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);

    let points: Vec<[f64; 3]> = raw_points
        .iter()
        .map(|&(x, y, m)| [x, y, if m { 1.0 } else { 0.0 }])
        .collect();

    match phase {
        Phase::Calibrating { started, .. } => {
            let progress =
                (started.elapsed().as_secs_f64() / CALIBRATION.as_secs_f64()).min(1.0);
            ScanSnapshot {
                phase: "calibrating".into(),
                calibration_progress: progress,
                scan_number,
                timestamp_ms,
                points,
                background: vec![],
                clusters: vec![],
                humans_detected: 0,
            }
        }
        Phase::Detecting { background } => {
            let foreground: Vec<(f64, f64)> = raw_points
                .iter()
                .filter(|p| p.2)
                .map(|&(x, y, _)| (x, y))
                .collect();
            let mut clusters = analyze(&foreground, classifier);
            update_tracks(tracks, &mut clusters);
            let humans_detected = clusters.iter().filter(|c| c.is_human).count();
            let clusters = clusters_to_json(clusters);
            ScanSnapshot {
                phase: "detecting".into(),
                calibration_progress: 1.0,
                scan_number,
                timestamp_ms,
                points,
                background: background.clone(),
                clusters,
                humans_detected,
            }
        }
    }
}

fn clusters_to_json(clusters: Vec<Cluster>) -> Vec<ClusterJson> {
    clusters
        .into_iter()
        .map(|c| ClusterJson {
            centroid: [c.centroid.0, c.centroid.1],
            is_human: c.is_human,
            features: FeaturesJson {
                width: c.features.width,
                depth: c.features.depth,
                curvature: c.features.curvature,
                point_count: c.features.point_count,
            },
            points: c.points.iter().map(|&(x, y)| [x, y]).collect(),
        })
        .collect()
}

fn update_tracks(tracks: &mut Vec<HumanTrack>, clusters: &mut [Cluster]) {
    let mut matched = vec![false; tracks.len()];

    for c in clusters.iter_mut() {
        let mut best: Option<(usize, f64)> = None;
        for (ti, t) in tracks.iter().enumerate() {
            if matched[ti] {
                continue;
            }
            let dx = c.centroid.0 - t.centroid.0;
            let dy = c.centroid.1 - t.centroid.1;
            let d = (dx * dx + dy * dy).sqrt();
            if d <= TRACK_MATCH_M && best.map_or(true, |(_, bd)| d < bd) {
                best = Some((ti, d));
            }
        }
        if let Some((ti, _)) = best {
            matched[ti] = true;
            tracks[ti].centroid = c.centroid;
            tracks[ti].missed = 0;
            c.is_human = true;
        } else if c.is_human {
            tracks.push(HumanTrack { centroid: c.centroid, missed: 0 });
            matched.push(true);
        }
    }

    let mut i = 0;
    while i < tracks.len() {
        if !matched[i] {
            tracks[i].missed += 1;
        }
        if tracks[i].missed > TRACK_TTL_SCANS {
            tracks.swap_remove(i);
            matched.swap_remove(i);
        } else {
            i += 1;
        }
    }
}

// ── Embedded web UI ───────────────────────────────────────────────────────────

const HTML: &str = r#"<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>LD19 LIDAR Monitor</title>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: #0d1117;
      color: #e6edf3;
      font-family: 'Courier New', monospace;
      display: flex;
      height: 100vh;
      overflow: hidden;
    }
    #main {
      flex: 1;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 8px;
    }
    canvas { border: 1px solid #21262d; border-radius: 4px; }
    #legend {
      display: flex;
      gap: 16px;
      font-size: 11px;
      color: #8b949e;
    }
    .dot {
      display: inline-block;
      width: 8px; height: 8px;
      border-radius: 50%;
      margin-right: 4px;
      vertical-align: middle;
    }
    #panel {
      width: 240px;
      background: #161b22;
      border-left: 1px solid #21262d;
      padding: 16px;
      overflow-y: auto;
      display: flex;
      flex-direction: column;
      gap: 16px;
    }
    h3 {
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 1px;
      color: #8b949e;
      margin-bottom: 8px;
    }
    .stat-row {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      margin-bottom: 4px;
      font-size: 13px;
    }
    .stat-label { color: #8b949e; }
    .stat-value { color: #e6edf3; font-weight: bold; }
    .badge {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 3px 10px;
      border-radius: 12px;
      font-size: 12px;
      font-weight: bold;
    }
    .badge-cal { background: #1c3a5e; color: #58a6ff; border: 1px solid #1f6feb; }
    .badge-det { background: #1a3a1a; color: #3fb950; border: 1px solid #238636; }
    .progress-track {
      background: #21262d;
      border-radius: 3px;
      height: 4px;
      margin-top: 6px;
      overflow: hidden;
    }
    .progress-fill {
      height: 100%;
      background: #1f6feb;
      border-radius: 3px;
      transition: width 0.5s ease;
    }
    .cluster-card {
      background: #21262d;
      border-radius: 6px;
      padding: 8px 10px;
      font-size: 12px;
      border-left: 3px solid #444;
    }
    .cluster-card.human { border-color: #ff4444; }
    .cluster-title { font-weight: bold; margin-bottom: 3px; }
    .cluster-detail { color: #8b949e; line-height: 1.5; }
    #footer {
      font-size: 10px;
      color: #484f58;
      border-top: 1px solid #21262d;
      padding-top: 8px;
      margin-top: auto;
    }
  </style>
</head>
<body>
  <div id="main">
    <canvas id="canvas" width="580" height="580"></canvas>
    <div id="legend">
      <span><span class="dot" style="background:#2d333b"></span>Background</span>
      <span><span class="dot" style="background:#ff4444"></span>Motion</span>
      <span><span class="dot" style="background:#0055ff"></span>Human cluster</span>
      <span><span class="dot" style="background:#ffa500"></span>Origin</span>
    </div>
  </div>

  <div id="panel">
    <div>
      <h3>Status</h3>
      <div id="phase-badge" class="badge badge-cal">
        <span id="phase-text">Waiting...</span>
      </div>
      <div class="progress-track" id="progress-track">
        <div class="progress-fill" id="progress-fill" style="width:0%"></div>
      </div>
    </div>

    <div>
      <h3>Scan Info</h3>
      <div class="stat-row">
        <span class="stat-label">Scan #</span>
        <span class="stat-value" id="scan-num">—</span>
      </div>
      <div class="stat-row">
        <span class="stat-label">Total pts</span>
        <span class="stat-value" id="total-pts">—</span>
      </div>
      <div class="stat-row">
        <span class="stat-label">Motion pts</span>
        <span class="stat-value" id="motion-pts">—</span>
      </div>
      <div class="stat-row">
        <span class="stat-label">Clusters</span>
        <span class="stat-value" id="cluster-count">—</span>
      </div>
    </div>

    <div>
      <h3>Human Detection</h3>
      <div class="stat-row">
        <span class="stat-label">Humans detected</span>
        <span class="stat-value" id="humans" style="color:#3fb950">0</span>
      </div>
      <div id="cluster-list" style="display:flex;flex-direction:column;gap:6px;margin-top:8px"></div>
    </div>

    <div id="footer">
      <div id="update-time">Not connected</div>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('canvas');
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const CX = W / 2, CY = H / 2;
    const MAX_R = 4.0;
    const SCALE = (Math.min(CX, CY) - 20) / MAX_R;

    // Mirror the 90-degree CW rotation used in main.rs plot():
    //   x_rot = y_lidar,  px = CX + x_rot * SCALE
    //   y_rot = -x_lidar, py = CY - y_rot * SCALE  =>  CY + x_lidar * SCALE
    function toScreen(x, y) {
      return [CX + y * SCALE, CY + x * SCALE];
    }

    function blueRed(t) {
      t = Math.max(0, Math.min(1, t));
      const r = Math.round(t * 255);
      const g = Math.round(30 + (1 - t) * 20);
      const b = Math.round((1 - t) * 255);
      return 'rgb(' + r + ',' + g + ',' + b + ')';
    }

    function drawGrid() {
      ctx.strokeStyle = '#1c2128';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      for (let r = 1; r <= MAX_R; r++) {
        ctx.beginPath();
        ctx.arc(CX, CY, r * SCALE, 0, Math.PI * 2);
        ctx.stroke();
        ctx.fillStyle = '#484f58';
        ctx.font = '10px monospace';
        ctx.fillText(r + 'm', CX + r * SCALE + 3, CY + 12);
      }
      ctx.setLineDash([]);
      ctx.strokeStyle = '#1c2128';
      ctx.beginPath(); ctx.moveTo(0, CY); ctx.lineTo(W, CY); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(CX, 0); ctx.lineTo(CX, H); ctx.stroke();
    }

    function draw(data) {
      ctx.fillStyle = '#0d1117';
      ctx.fillRect(0, 0, W, H);
      drawGrid();

      if (data.phase === 'calibrating') {
        ctx.fillStyle = 'rgba(31,111,235,0.3)';
        for (const pt of data.points) {
          const [px, py] = toScreen(pt[0], pt[1]);
          ctx.fillRect(px - 1, py - 1, 3, 3);
        }
      } else {
        // Background bins
        ctx.fillStyle = '#2d333b';
        for (let bin = 0; bin < data.background.length; bin++) {
          const d = data.background[bin];
          if (d == null) continue;
          const rad = bin * Math.PI / 180;
          const [px, py] = toScreen(Math.cos(rad) * d, Math.sin(rad) * d);
          ctx.fillRect(px - 1, py - 1, 2, 2);
        }

        // Motion points
        ctx.fillStyle = '#ff4444';
        for (const pt of data.points) {
          if (pt[2] < 0.5) continue;
          const [px, py] = toScreen(pt[0], pt[1]);
          ctx.fillRect(px - 2, py - 2, 4, 4);
        }

        // Clusters
        for (const c of data.clusters) {
          const [scx, scy] = toScreen(c.centroid[0], c.centroid[1]);
          if (c.is_human) {
            const maxD = (c.features.width / 2) || 0.1;
            for (const pt of c.points) {
              const dx = pt[0] - c.centroid[0];
              const dy = pt[1] - c.centroid[1];
              const t = Math.min(Math.sqrt(dx * dx + dy * dy) / maxD, 1);
              ctx.fillStyle = blueRed(t);
              const [spx, spy] = toScreen(pt[0], pt[1]);
              ctx.fillRect(spx - 2, spy - 2, 5, 5);
            }
            // Bounding ring
            const radius = (c.features.width / 2) * SCALE + 10;
            ctx.strokeStyle = 'rgba(255,68,68,0.5)';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(scx, scy, radius, 0, Math.PI * 2);
            ctx.stroke();
            // Distance label
            const dist = Math.sqrt(c.centroid[0] ** 2 + c.centroid[1] ** 2);
            ctx.fillStyle = '#ffaaaa';
            ctx.font = 'bold 11px monospace';
            ctx.fillText(dist.toFixed(2) + 'm', scx + radius * 0.7 + 4, scy - 4);
          } else {
            ctx.fillStyle = '#555';
            for (const pt of c.points) {
              const [spx, spy] = toScreen(pt[0], pt[1]);
              ctx.fillRect(spx - 1, spy - 1, 3, 3);
            }
          }
        }
      }

      // Origin
      ctx.fillStyle = '#ffa500';
      ctx.beginPath();
      ctx.arc(CX, CY, 5, 0, Math.PI * 2);
      ctx.fill();
    }

    function updatePanel(data) {
      const badge = document.getElementById('phase-badge');
      const phaseText = document.getElementById('phase-text');
      const progressFill = document.getElementById('progress-fill');
      const progressTrack = document.getElementById('progress-track');

      if (data.phase === 'calibrating') {
        badge.className = 'badge badge-cal';
        const pct = Math.round(data.calibration_progress * 100);
        phaseText.textContent = 'Calibrating ' + pct + '%';
        progressTrack.style.display = 'block';
        progressFill.style.width = pct + '%';
      } else {
        badge.className = 'badge badge-det';
        phaseText.textContent = 'Detecting';
        progressTrack.style.display = 'none';
      }

      document.getElementById('scan-num').textContent = data.scan_number;
      document.getElementById('total-pts').textContent = data.points.length;
      document.getElementById('motion-pts').textContent = data.points.filter(function(p) { return p[2] > 0.5; }).length;
      document.getElementById('cluster-count').textContent = data.clusters.length;

      var humansEl = document.getElementById('humans');
      humansEl.textContent = data.humans_detected;
      humansEl.style.color = data.humans_detected > 0 ? '#ff4444' : '#3fb950';

      var list = document.getElementById('cluster-list');
      list.innerHTML = '';
      for (var i = 0; i < data.clusters.length; i++) {
        var c = data.clusters[i];
        var dist = Math.sqrt(c.centroid[0] * c.centroid[0] + c.centroid[1] * c.centroid[1]);
        var angleDeg = Math.atan2(c.centroid[1], c.centroid[0]) * 180 / Math.PI;
        var card = document.createElement('div');
        card.className = 'cluster-card' + (c.is_human ? ' human' : '');
        card.innerHTML =
          '<div class="cluster-title">' + (c.is_human ? '&#x1F464; Human' : '&#x1F4E6; Object') + '</div>' +
          '<div class="cluster-detail">' +
            dist.toFixed(2) + 'm &nbsp;@&nbsp; ' + angleDeg.toFixed(1) + '&deg;<br>' +
            'width ' + c.features.width.toFixed(2) + 'm &nbsp; pts ' + Math.round(c.features.point_count) +
          '</div>';
        list.appendChild(card);
      }

      var ts = new Date(Number(data.timestamp_ms));
      document.getElementById('update-time').textContent = 'Updated ' + ts.toLocaleTimeString();
    }

    var lastScan = -1;

    function poll() {
      fetch('/api/scan')
        .then(function(res) {
          if (!res.ok) throw new Error('HTTP ' + res.status);
          return res.json();
        })
        .then(function(data) {
          if (data.scan_number !== lastScan) {
            lastScan = data.scan_number;
            draw(data);
            updatePanel(data);
          }
        })
        .catch(function(e) {
          document.getElementById('update-time').textContent = 'Error: ' + e.message;
          ctx.fillStyle = 'rgba(255,0,0,0.1)';
          ctx.fillRect(0, 0, W, H);
          ctx.fillStyle = '#ff4444';
          ctx.font = '14px monospace';
          ctx.textAlign = 'center';
          ctx.fillText('Connection lost — ' + e.message, CX, CY);
          ctx.textAlign = 'left';
        })
        .finally(function() { setTimeout(poll, 100); });
    }

    ctx.fillStyle = '#0d1117';
    ctx.fillRect(0, 0, W, H);
    ctx.fillStyle = '#484f58';
    ctx.font = '14px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('Connecting to LIDAR server...', CX, CY);
    ctx.textAlign = 'left';

    poll();
  </script>
</body>
</html>"#;
