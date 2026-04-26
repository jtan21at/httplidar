use lidar_ld19::detect::{analyze, log_features_csv, Classifier, Cluster};
use lidar_ld19::{LD19, DIR_ROUND};
use minifb::{Key, Window, WindowOptions};
use std::time::{Duration, Instant};
use std::process::Command;

const PORT: &str = "/dev/ttyUSB0";
const MAX_RANGE_M: f64 = 4.0;
const WIDTH: usize = 800;
const HEIGHT: usize = 800;

const CALIBRATION: Duration = Duration::from_secs(30);
const ANGLE_BINS: usize = 360;
const MATCH_TOLERANCE_M: f64 = 0.15;
const MIN_BG_SAMPLES: u32 = 5;

const MODEL_PATH: &str = "human_rf.bin";
const TRAINING_CSV: &str = "training_data.csv";
const DOG_BARK_SCRIPT: &str = "/home/rdrp/dog_bark_pi5.py";
const BARK_COOLDOWN: Duration = Duration::from_secs(5);

// FIXME: replace this crude nearest-centroid sticky-tracker with a proper
// multi-object tracker 
// Today a cluster that was ever classified human stayshuman as long as
// any cluster keeps appearing within TRACK_MATCH_M of its last  centroid
// which will happily latch onto a different object that wanders through
//so REPLACE later, right now it is very poorly matching humans 
const TRACK_MATCH_M: f64 = 0.40;
const TRACK_TTL_SCANS: u32 = 30;

const BG_COLOR: u32 = 0x00_ff_ff_ff;
const GRID: u32 = 0x00_d0_d0_d0;
const STATIC_COLOR: u32 = 0x00_00_00_00;
const MOTION_COLOR: u32 = 0x00_ff_30_30;
const ORIGIN: u32 = 0x00_c0_a0_00;
const CAL_COLOR: u32 = 0x00_00_80_c8;

enum Phase {
    Calibrating { started: Instant, samples: Vec<Vec<f64>> },
    Detecting  { background: Vec<Option<f64>> },
}

struct Scan {
    points: Vec<(f64, f64, bool)>,
    clusters: Vec<Cluster>,
}

struct HumanTrack {
    centroid: (f64, f64),
    missed: u32,
}

fn main() {
    env_logger::init();
    let mut last_bark = Instant::now() - BARK_COOLDOWN; // allow immediate first bark
    let mut lidar = match LD19::open(PORT) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("Failed to open {PORT}: {e}");
            std::process::exit(1);
        }
    };

    let mut window = Window::new(
        "LD19 Lidar — Motion Detector  (ESC to quit)",
        WIDTH,
        HEIGHT,
        WindowOptions::default(),
    )
    .unwrap();
    window.set_target_fps(20);

    let mut buf = vec![BG_COLOR; WIDTH * HEIGHT];
    let mut scan = Scan {
        points: Vec::with_capacity(500),
        clusters: Vec::new(),
    };
    let mut last_dir: u16 = u16::MAX;
    let classifier = Classifier::load_or_fallback(MODEL_PATH);
    let mut tracks: Vec<HumanTrack> = Vec::new();

    let mut phase = Phase::Calibrating {
        started: Instant::now(),
        samples: vec![Vec::new(); ANGLE_BINS],
    };

    println!("Calibrating for {}s — keep the scene static...", CALIBRATION.as_secs());

    while window.is_open() && !window.is_key_down(Key::Escape) {
        for raw in lidar.poll() {
            let wrapped = last_dir != u16::MAX
                && raw.dir < last_dir
                && last_dir > DIR_ROUND / 2;
            last_dir = raw.dir;

            if wrapped {
                phase = maybe_finish_calibration(phase);
                if matches!(phase, Phase::Detecting { .. }) {
                    let foreground: Vec<(f64, f64)> = scan
                        .points
                        .iter()
                        .filter(|p| p.2)
                        .map(|p| (p.0, p.1))
                        .collect();
                    scan.clusters = analyze(&foreground, &classifier);
                    log_features_csv(TRAINING_CSV, &scan.clusters);
                    update_tracks(&mut tracks, &mut scan.clusters);
                    if let Some(angle) = closest_human_angle(&scan.clusters) {
                        println!("human @ {:.1}°", angle);
                        if last_bark.elapsed() >= BARK_COOLDOWN {
                            last_bark = Instant::now();
                            println!("🐕 Barking! Human detected at {:.1}°", angle);
                            std::thread::spawn(|| {
                                match Command::new("python3")
                                    .arg(DOG_BARK_SCRIPT)
                                    .spawn()
                                {
                                    Ok(mut child) => { let _ = child.wait(); }
                                    Err(e) => eprintln!("Failed to run dog bark: {}", e),
                                }
                            });
                        }
                    }
                } else {
                    scan.clusters.clear();
                    tracks.clear();
                }
                redraw(&scan, &phase, &mut buf);
                window.update_with_buffer(&buf, WIDTH, HEIGHT).unwrap();
                scan.points.clear();
            }

            if raw.len == 0 { continue; }
            let meters = raw.len as f64 / 1000.0;
            if meters > MAX_RANGE_M { continue; }
            let deg = raw.dir as f64 / 100.0;
            let bin = (deg.floor() as usize) % ANGLE_BINS;
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
                    Some(bg_m) => (meters - bg_m).abs() > MATCH_TOLERANCE_M,
                    None => true,
                },
            };
            scan.points.push((x, y, is_motion));
        }
    }
}

fn closest_human_angle(clusters: &[Cluster]) -> Option<f64> {
    clusters
        .iter()
        .filter(|c| c.is_human && c.centroid.1 >= 0.0)
        .min_by(|a, b| {
            let da = a.centroid.0.hypot(a.centroid.1);
            let db = b.centroid.0.hypot(b.centroid.1);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|c| {
            let (x, y) = c.centroid;
            y.atan2(x).to_degrees().clamp(0.0, 180.0)
        })
}

// FIXME: crude "once human, always human" sticky tracker — see comment at the
// top of this file. Swap for a real tracker when we care about correctness.
fn update_tracks(tracks: &mut Vec<HumanTrack>, clusters: &mut [Cluster]) {
    let mut matched_track = vec![false; tracks.len()];

    for c in clusters.iter_mut() {
        let mut best: Option<(usize, f64)> = None;
        for (ti, t) in tracks.iter().enumerate() {
            if matched_track[ti] {
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
            matched_track[ti] = true;
            tracks[ti].centroid = c.centroid;
            tracks[ti].missed = 0;
            c.is_human = true;
        } else if c.is_human {
            tracks.push(HumanTrack { centroid: c.centroid, missed: 0 });
            matched_track.push(true);
        }
    }

    let mut i = 0;
    while i < tracks.len() {
        if !matched_track[i] {
            tracks[i].missed += 1;
        }
        if tracks[i].missed > TRACK_TTL_SCANS {
            tracks.swap_remove(i);
            matched_track.swap_remove(i);
        } else {
            i += 1;
        }
    }
}

fn maybe_finish_calibration(phase: Phase) -> Phase {
    match phase {
        Phase::Calibrating { started, samples } if started.elapsed() >= CALIBRATION => {
            let background: Vec<Option<f64>> = samples.iter().map(|s| {
                if (s.len() as u32) < MIN_BG_SAMPLES { return None; }
                let mut v = s.clone();
                v.sort_by(|a, b| a.partial_cmp(b).unwrap());
                Some(v[v.len() / 2])
            }).collect();
            let locked = background.iter().filter(|b| b.is_some()).count();
            println!("Calibration done. Locked {} / {} angle bins. Motion detection active.", locked, ANGLE_BINS);
            Phase::Detecting { background }
        }
        other => other,
    }
}

fn redraw(scan: &Scan, phase: &Phase, buf: &mut Vec<u32>) {
    buf.fill(BG_COLOR);

    let cx = WIDTH as f64 / 2.0;
    let cy = HEIGHT as f64 / 2.0;
    let scale = cx.min(cy) / MAX_RANGE_M;

    for r_m in 1..=(MAX_RANGE_M as usize) {
        let r_px = (r_m as f64 * scale) as isize;
        draw_circle(buf, cx as isize, cy as isize, r_px, GRID);
    }
    draw_hline(buf, cy as usize, GRID);
    draw_vline(buf, cx as usize, GRID);

    match phase {
        Phase::Calibrating { started, .. } => {
            for &(x, y, _) in &scan.points {
                plot(buf, cx, cy, scale, x, y, 2, CAL_COLOR);
            }
            let frac = (started.elapsed().as_secs_f64() / CALIBRATION.as_secs_f64()).min(1.0);
            draw_progress_arc(buf, cx as isize, cy as isize, (cx.min(cy) - 10.0) as isize, frac, CAL_COLOR);
        }
        Phase::Detecting { background } => {
            for (bin, dist) in background.iter().enumerate() {
                if let Some(meters) = dist {
                    let rad = (bin as f64).to_radians();
                    let x = rad.cos() * meters;
                    let y = rad.sin() * meters;
                    plot(buf, cx, cy, scale, x, y, 2, STATIC_COLOR);
                }
            }
            for &(x, y, motion) in &scan.points {
                if motion {
                    plot(buf, cx, cy, scale, x, y, 3, MOTION_COLOR);
                }
            }
            for cluster in &scan.clusters {
                if !cluster.is_human {
                    continue;
                }
                let (hx, hy) = cluster.centroid;
                let max_d = cluster
                    .points
                    .iter()
                    .map(|&(x, y)| ((x - hx).powi(2) + (y - hy).powi(2)).sqrt())
                    .fold(0.0_f64, f64::max)
                    .max(1e-6);
                for &(x, y) in &cluster.points {
                    let d = ((x - hx).powi(2) + (y - hy).powi(2)).sqrt();
                    let t = (d / max_d).clamp(0.0, 1.0);
                    plot(buf, cx, cy, scale, x, y, 3, blue_red_gradient(t));
                }
                let px = (cx + hx * scale).round() as isize;
                let py = (cy - hy * scale).round() as isize;
                draw_smile(buf, px, py - 18, blue_red_gradient(0.0));
            }
        }
    }

    let off = (cx.min(cy) - 40.0) as isize;
    let ox = cx as isize;
    let oy = cy as isize;
    draw_label(buf, ox + off,      oy - off, "Q1", STATIC_COLOR);
    draw_label(buf, ox - off - 20, oy - off, "Q2", STATIC_COLOR);
    draw_label(buf, ox - off - 20, oy + off, "Q3", STATIC_COLOR);
    draw_label(buf, ox + off,      oy + off, "Q4", STATIC_COLOR);

    fill_dot(buf, cx as isize, cy as isize, 4, ORIGIN);
}

fn draw_label(buf: &mut Vec<u32>, x: isize, y: isize, s: &str, color: u32) {
    let mut cx = x;
    for ch in s.chars() {
        draw_glyph(buf, cx, y, ch, color);
        cx += 8;
    }
}

fn draw_glyph(buf: &mut Vec<u32>, x: isize, y: isize, ch: char, color: u32) {
    let rows: [u8; 7] = match ch {
        'Q' => [0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101],
        '1' => [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        '2' => [0b01110, 0b10001, 0b00001, 0b00110, 0b01000, 0b10000, 0b11111],
        '3' => [0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001, 0b11110],
        '4' => [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
        _   => return,
    };
    for (ry, row) in rows.iter().enumerate() {
        for rx in 0..5 {
            if row & (1 << (4 - rx)) != 0 {
                let px = x + rx as isize;
                let py = y + ry as isize;
                set_px(buf, px,     py,     color);
                set_px(buf, px + 1, py,     color);
                set_px(buf, px,     py + 1, color);
                set_px(buf, px + 1, py + 1, color);
            }
        }
    }
}

fn blue_red_gradient(t: f64) -> u32 {
    let t = t.clamp(0.0, 1.0);
    let r = (t * 255.0).round() as u32;
    let b = ((1.0 - t) * 255.0).round() as u32;
    (r << 16) | b
}

fn draw_smile(buf: &mut Vec<u32>, cx: isize, cy: isize, color: u32) {
    draw_circle(buf, cx, cy, 10, color);
    fill_dot(buf, cx - 4, cy - 3, 1, color);
    fill_dot(buf, cx + 4, cy - 3, 1, color);
    for i in -4..=4 {
        let a = (i as f64) * 0.22 + std::f64::consts::FRAC_PI_2;
        let x = cx + (a.cos() * 5.0).round() as isize;
        let y = cy + (a.sin() * 5.0).round() as isize;
        set_px(buf, x, y, color);
        set_px(buf, x + 1, y, color);
    }
}

fn plot(buf: &mut Vec<u32>, cx: f64, cy: f64, scale: f64, x: f64, y: f64, r: isize, color: u32) {
    // 90-degree Clockwise rotation
    let x_rot = y;
    let y_rot = -x;

    let px = (cx + x_rot/2.0 * scale).round() as isize;
    let py = (cy - y_rot/2.0 * scale).round() as isize;
    fill_dot(buf, px, py, r, color);
}

fn set_px(buf: &mut Vec<u32>, x: isize, y: isize, color: u32) {
    if x >= 0 && x < WIDTH as isize && y >= 0 && y < HEIGHT as isize {
        buf[y as usize * WIDTH + x as usize] = color;
    }
}

fn fill_dot(buf: &mut Vec<u32>, cx: isize, cy: isize, r: isize, color: u32) {
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                set_px(buf, cx + dx, cy + dy, color);
            }
        }
    }
}

fn draw_hline(buf: &mut Vec<u32>, y: usize, color: u32) {
    if y < HEIGHT {
        for x in 0..WIDTH { buf[y * WIDTH + x] = color; }
    }
}

fn draw_vline(buf: &mut Vec<u32>, x: usize, color: u32) {
    if x < WIDTH {
        for y in 0..HEIGHT { buf[y * WIDTH + x] = color; }
    }
}

fn draw_circle(buf: &mut Vec<u32>, cx: isize, cy: isize, r: isize, color: u32) {
    let (mut x, mut y, mut d) = (0isize, r, 1 - r);
    while x <= y {
        for &(px, py) in &[
            (cx+x,cy+y),(cx-x,cy+y),(cx+x,cy-y),(cx-x,cy-y),
            (cx+y,cy+x),(cx-y,cy+x),(cx+y,cy-x),(cx-y,cy-x),
        ] {
            set_px(buf, px, py, color);
        }
        x += 1;
        if d < 0 { d += 2*x + 1; } else { y -= 1; d += 2*(x-y) + 1; }
    }
}


fn draw_progress_arc(buf: &mut Vec<u32>, cx: isize, cy: isize, r: isize, frac: f64, color: u32) {
    let steps = 360;
    let end = (frac * steps as f64) as i32;
    for i in 0..end {
        let a = (i as f64 / steps as f64) * std::f64::consts::TAU - std::f64::consts::FRAC_PI_2;
        let px = cx + (a.cos() * r as f64).round() as isize;
        let py = cy + (a.sin() * r as f64).round() as isize;
        for dx in -1..=1 {
            for dy in -1..=1 {
                set_px(buf, px + dx, py + dy, color);
            }
        }
    }
}
