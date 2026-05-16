/* compute.js — CSV parsing, derived columns, lap detection, insights.
   Globals: TelemetryCompute (window.TelemetryCompute)
*/
(function(){
  'use strict';

  // ── Column normalization ─────────────────────────────────────────────
  const COL_MAP = {
    't_us':'t_us','t_ms':'t_ms',
    'ax':'ax','ay':'ay','az':'az',
    'gx':'gx','gy':'gy','gz':'gz',
    'butter_ax':'ax','butter_ay':'ay','butter_az':'az',
    'butter_gx':'gx','butter_gy':'gy','butter_gz':'gz',
    'temp_c':'temp_c','lap':'lap',
    'gps_lat':'gps_lat','gps_lon':'gps_lon',
    'gps_sog_kmh':'gps_sog_kmh','gps_speed_kmh':'gps_sog_kmh',
    'gps_alt_m':'gps_alt_m','gps_sats':'gps_sats','gps_hdop':'gps_hdop',
    'kf_x':'kf_x','kf_y':'kf_y','kf_vel':'kf_vel','kf_heading':'kf_heading',
    'raw_ax':'raw_ax','raw_ay':'raw_ay','raw_az':'raw_az',
    'raw_gx':'raw_gx','raw_gy':'raw_gy','raw_gz':'raw_gz',
    'pipe_lin_ax':'pipe_lin_ax','pipe_lin_ay':'pipe_lin_ay','pipe_lin_az':'pipe_lin_az',
    'pipe_body_gx':'pipe_body_gx','pipe_body_gy':'pipe_body_gy','pipe_body_gz':'pipe_body_gz',
    'bmi_acc_x_g':'bmi_acc_x_g','bmi_acc_y_g':'bmi_acc_y_g','bmi_acc_z_g':'bmi_acc_z_g',
    'bmi_gyr_x_dps':'bmi_gyr_x_dps','bmi_gyr_y_dps':'bmi_gyr_y_dps','bmi_gyr_z_dps':'bmi_gyr_z_dps',
    'kf6_x':'kf6_x','kf6_y':'kf6_y','kf6_vel':'kf6_vel','kf6_heading':'kf6_heading','kf6_bgz':'kf6_bgz',
    'sensor_ax':'sensor_ax','sensor_ay':'sensor_ay','sensor_az':'sensor_az',
    'sensor_gx':'sensor_gx','sensor_gy':'sensor_gy','sensor_gz':'sensor_gz',
  };
  function stripUnit(h){ return (h||'').split(' (')[0].trim().toLowerCase(); }
  function normaliseHeaders(fields){
    const map = {};
    for(const f of fields){
      const base = stripUnit(f);
      const canon = COL_MAP[base];
      if(canon) map[f] = canon;
    }
    return map;
  }

  // ── Main parse + compute ─────────────────────────────────────────────
  async function parseCSV(file, onProgress){
    return new Promise((resolve, reject)=>{
      const chunks = [];
      let fieldMap = null;
      let headers = null;
      let totalRows = 0;
      Papa.parse(file, {
        header: true, dynamicTyping: true, skipEmptyLines: true,
        comments: '#', worker: false, chunkSize: 1024*1024,
        chunk: (res) => {
          if(!fieldMap){
            fieldMap = normaliseHeaders(res.meta.fields);
            headers = res.meta.fields;
          }
          chunks.push(res.data);
          totalRows += res.data.length;
          if(onProgress) onProgress(totalRows);
        },
        complete: () => {
          try {
            const rows = [].concat(...chunks);
            const data = buildArrays(rows, fieldMap);
            resolve(data);
          } catch(e){ reject(e); }
        },
        error: reject,
      });
    });
  }

  function buildArrays(rows, fm){
    const N = rows.length;
    if(N === 0) throw new Error('CSV is empty.');

    // Build canonical-name columns by scanning original headers
    const colAliasToOrig = {};
    for(const [orig, canon] of Object.entries(fm)) colAliasToOrig[canon] = orig;

    const getCol = (canon) => {
      const orig = colAliasToOrig[canon];
      if(!orig) return null;
      const arr = new Float64Array(N);
      for(let i=0;i<N;i++){ const v = rows[i][orig]; arr[i] = (v==null||v==='')?NaN:+v; }
      return arr;
    };

    // Timestamps
    let t_us = getCol('t_us');
    let t_ms = getCol('t_ms');
    if(!t_us && !t_ms) throw new Error('Missing timestamp column (t_us or t_ms).');
    if(!t_ms && t_us){ t_ms = new Float64Array(N); for(let i=0;i<N;i++) t_ms[i] = t_us[i]/1000; }

    // Required
    const required = ['kf_vel','kf_x','kf_y','gps_lat','gps_lon','ax','ay','az','gz','gx'];
    const missing = required.filter(c => !colAliasToOrig[c]);
    if(missing.length) throw new Error('Missing required columns: '+missing.join(', '));

    const o = {};
    const cols = ['ax','ay','az','gx','gy','gz','temp_c','lap',
      'gps_lat','gps_lon','gps_sog_kmh','gps_alt_m','gps_sats','gps_hdop',
      'kf_x','kf_y','kf_vel','kf_heading',
      'raw_ax','raw_ay','raw_az','raw_gx','raw_gy','raw_gz',
      'pipe_lin_ax','pipe_lin_ay','pipe_lin_az','pipe_body_gx','pipe_body_gy','pipe_body_gz',
      'bmi_acc_x_g','bmi_acc_y_g','bmi_acc_z_g','bmi_gyr_x_dps','bmi_gyr_y_dps','bmi_gyr_z_dps'];
    for(const c of cols){ const arr = getCol(c); if(arr) o[c] = arr; }

    o._rawSource = 'none';
    if(o.raw_ax && o.raw_ay && o.raw_gx && o.raw_gz){
      o._rawSource = 'legacy_raw';
    } else if(o.pipe_lin_ax && o.pipe_lin_ay && o.pipe_body_gx && o.pipe_body_gz){
      o.raw_ax = o.pipe_lin_ax; o.raw_ay = o.pipe_lin_ay; o.raw_az = o.pipe_lin_az;
      o.raw_gx = o.pipe_body_gx; o.raw_gy = o.pipe_body_gy; o.raw_gz = o.pipe_body_gz;
      o._rawSource = 'pipe_pre_presentation';
    } else if(o.bmi_acc_x_g && o.bmi_acc_y_g && o.bmi_gyr_x_dps && o.bmi_gyr_z_dps){
      o.raw_ax = o.bmi_acc_x_g; o.raw_ay = o.bmi_acc_y_g; o.raw_az = o.bmi_acc_z_g;
      o.raw_gx = o.bmi_gyr_x_dps; o.raw_gy = o.bmi_gyr_y_dps; o.raw_gz = o.bmi_gyr_z_dps;
      o._rawSource = 'bmi_physical';
    }
    o._hasRaw = o._rawSource !== 'none';

    // t_s (elapsed seconds, origin-zero)
    const t_s = new Float64Array(N);
    const t0 = t_us ? t_us[0] : t_ms[0];
    const scale = t_us ? 1e-6 : 1e-3;
    for(let i=0;i<N;i++){ t_s[i] = ((t_us?t_us[i]:t_ms[i]) - t0) * scale; }
    o.t_s = t_s;

    // distance_m (abs(v) integrated)
    const dist = new Float64Array(N);
    const v = o.kf_vel;
    let d = 0;
    for(let i=1;i<N;i++){
      const dt = t_s[i] - t_s[i-1];
      d += Math.abs(v[i]) * Math.max(0, dt);
      dist[i] = d;
    }
    o.distance_m = dist;

    // vel_kmh
    const vkmh = new Float64Array(N);
    for(let i=0;i<N;i++) vkmh[i] = v[i]*3.6;
    o.vel_kmh = vkmh;

    // acc_mag (2D) filtered + raw
    const accMag = new Float64Array(N);
    for(let i=0;i<N;i++) accMag[i] = Math.hypot(o.ax[i]||0, o.ay[i]||0);
    o.acc_mag = accMag;
    if(o.raw_ax && o.raw_ay){
      const rm = new Float64Array(N);
      for(let i=0;i<N;i++) rm[i] = Math.hypot(o.raw_ax[i]||0, o.raw_ay[i]||0);
      o.raw_acc_mag = rm;
    }

    // ENU → WGS84 for kf_x/kf_y
    const {kf_lat, kf_lon} = enuToLatLon(o.kf_x, o.kf_y, o.gps_lat, o.gps_lon);
    o.kf_lat = kf_lat; o.kf_lon = kf_lon;

    // Metadata
    o._N = N;
    o._duration_s = t_s[N-1] - t_s[0];
    return o;
  }

  function enuToLatLon(kx, ky, glat, glon){
    const N = kx.length;
    const kf_lat = new Float64Array(N);
    const kf_lon = new Float64Array(N);
    // Find first valid GPS fix to recover ENU origin
    let i0 = -1;
    for(let i=0;i<N;i++){
      if(glat[i] && glon[i] && glat[i]!==0 && glon[i]!==0){ i0 = i; break; }
    }
    if(i0 === -1) return {kf_lat, kf_lon};
    const R = 6371000;
    const gLat0 = glat[i0], gLon0 = glon[i0];
    const cosLat0 = Math.cos(gLat0 * Math.PI/180);
    // origin = observed gps - offset used in kf
    const lat0 = gLat0 - (ky[i0]/R) * 180/Math.PI;
    const lon0 = gLon0 - (kx[i0]/(R*cosLat0)) * 180/Math.PI;
    const cosLat0b = Math.cos(lat0 * Math.PI/180);
    for(let i=0;i<N;i++){
      kf_lat[i] = lat0 + (ky[i]/R) * 180/Math.PI;
      kf_lon[i] = lon0 + (kx[i]/(R*cosLat0b)) * 180/Math.PI;
    }
    return {kf_lat, kf_lon};
  }

  // ── Lap detection ────────────────────────────────────────────────────
  // Uses the 'lap' column if present & non-constant; otherwise heuristic:
  // find the first point moving faster than 30 km/h, treat that position as
  // start/finish and segment each time trajectory returns within 25 m.
  function detectLaps(d){
    const N = d._N;
    const laps = [];
    if(d.lap){
      let curLap = d.lap[0];
      let startIdx = 0;
      for(let i=1;i<N;i++){
        if(d.lap[i] !== curLap){
          laps.push({lap: curLap|0, i0: startIdx, i1: i-1,
                     t0: d.t_s[startIdx], t1: d.t_s[i-1],
                     d0: d.distance_m[startIdx], d1: d.distance_m[i-1]});
          startIdx = i;
          curLap = d.lap[i];
        }
      }
      laps.push({lap: curLap|0, i0: startIdx, i1: N-1,
                 t0: d.t_s[startIdx], t1: d.t_s[N-1],
                 d0: d.distance_m[startIdx], d1: d.distance_m[N-1]});
    }
    // Filter out degenerate laps (<10s)
    const good = laps.filter(l => (l.t1 - l.t0) > 10);
    if(good.length < 2) return [{lap:1, i0:0, i1:N-1, t0:d.t_s[0], t1:d.t_s[N-1], d0:d.distance_m[0], d1:d.distance_m[N-1]}];
    // Compute duration and deltas
    let best = Infinity, bestIdx = -1;
    good.forEach((l,i)=>{ const dur = l.t1-l.t0; if(dur<best){best=dur; bestIdx=i;}});
    good.forEach(l => { l.duration = l.t1 - l.t0; l.delta = l.duration - best; l.best = (good.indexOf(l)===bestIdx); });
    return good;
  }

  // ── Session insights ─────────────────────────────────────────────────
  function computeInsights(d, laps){
    const N = d._N;
    const out = {};

    // Top speed + location
    let vmax = -Infinity, iv = 0;
    for(let i=0;i<N;i++){ if(d.vel_kmh[i]>vmax){vmax=d.vel_kmh[i]; iv=i;} }
    out.topSpeed = { v: vmax, t: d.t_s[iv], dist: d.distance_m[iv],
                     lat: d.gps_lat[iv]||d.kf_lat[iv], lon: d.gps_lon[iv]||d.kf_lon[iv],
                     lap: lapOfIdx(iv, laps), idx: iv };

    // Hardest brake (most negative ax)
    let axmin = Infinity, ib = 0;
    for(let i=0;i<N;i++){ if(d.ax[i]<axmin){axmin=d.ax[i]; ib=i;}}
    out.hardestBrake = { g: Math.abs(axmin), t: d.t_s[ib], dist: d.distance_m[ib],
                         speed: d.vel_kmh[ib], lat: d.gps_lat[ib]||d.kf_lat[ib], lon: d.gps_lon[ib]||d.kf_lon[ib],
                         lap: lapOfIdx(ib, laps), idx: ib };

    // Max lateral G (absolute ay) with speed
    let aymax = -Infinity, il = 0;
    for(let i=0;i<N;i++){ const a = Math.abs(d.ay[i]); if(a>aymax){aymax=a; il=i;}}
    out.maxLateral = { g: aymax, side: d.ay[il]>0?'L':'R',
                       t: d.t_s[il], dist: d.distance_m[il], speed: d.vel_kmh[il],
                       lat: d.gps_lat[il]||d.kf_lat[il], lon: d.gps_lon[il]||d.kf_lon[il],
                       lap: lapOfIdx(il, laps), idx: il };

    // Time in brake / accel / coast
    const BRAKE = -0.05, ACC = 0.05;
    let tb=0, ta=0, tc=0;
    for(let i=1;i<N;i++){
      const dt = d.t_s[i]-d.t_s[i-1];
      if(dt<=0||dt>0.5) continue;
      if(d.ax[i] < BRAKE) tb += dt;
      else if(d.ax[i] > ACC) ta += dt;
      else tc += dt;
    }
    const total = tb+ta+tc || 1;
    out.timeDist = { brake: tb, accel: ta, coast: tc,
                     brakePct: tb/total*100, accelPct: ta/total*100, coastPct: tc/total*100 };

    // G-G coverage (% of unit disk covered when scaled by actual max G)
    out.ggCoverage = computeGGCoverage(d);

    // Yaw stability (std dev)
    let sum=0,n=0;
    for(let i=0;i<N;i++){ if(isFinite(d.gz[i])){ sum+=d.gz[i]; n++; } }
    const mean = sum/Math.max(n,1);
    let varSum = 0;
    for(let i=0;i<N;i++){ if(isFinite(d.gz[i])){ const x = d.gz[i]-mean; varSum += x*x; } }
    out.yawStab = { std: Math.sqrt(varSum/Math.max(n,1)), mean };

    // Oversteer/understeer hint: when gz exceeds "expected" yaw from lateral accel
    //   ω_expected = ay*g / v, compare |gz_actual| to |ω_expected|.
    //   ratio > 1.2 → oversteer, < 0.8 → understeer.
    out.balance = computeBalance(d);

    // Distance + duration
    out.distance_m = d.distance_m[N-1];
    out.duration_s = d.t_s[N-1];

    return out;
  }

  function lapOfIdx(i, laps){
    for(const l of laps){ if(i>=l.i0 && i<=l.i1) return l.lap; }
    return laps[0]?.lap ?? 1;
  }

  function computeGGCoverage(d){
    // Bin ax/ay into a 20x20 grid within [-Gmax, +Gmax], return %cells with ≥3 hits
    const N = d._N;
    let Gmax = 0;
    for(let i=0;i<N;i++){ const a = Math.max(Math.abs(d.ax[i]||0), Math.abs(d.ay[i]||0)); if(a>Gmax) Gmax=a; }
    if(Gmax<=0) return { pct:0, Gmax:0 };
    const B = 20; const grid = new Int32Array(B*B); const step = (2*Gmax)/B;
    for(let i=0;i<N;i++){
      const ax = d.ax[i], ay = d.ay[i];
      if(!isFinite(ax)||!isFinite(ay)) continue;
      // Only count points inside the friction disc (radius Gmax)
      if(Math.hypot(ax,ay) > Gmax) continue;
      const ix = Math.min(B-1, Math.max(0, Math.floor((ay+Gmax)/step)));
      const iy = Math.min(B-1, Math.max(0, Math.floor((ax+Gmax)/step)));
      grid[iy*B+ix]++;
    }
    // Count cells inside the disc that are hit (≥3 samples)
    let used=0, inside=0;
    for(let iy=0; iy<B; iy++){
      for(let ix=0; ix<B; ix++){
        const cx = (ix+0.5)*step - Gmax;
        const cy = (iy+0.5)*step - Gmax;
        if(Math.hypot(cx,cy) <= Gmax*0.97){
          inside++;
          if(grid[iy*B+ix] >= 3) used++;
        }
      }
    }
    return { pct: used/Math.max(1,inside)*100, Gmax };
  }

  function computeBalance(d){
    // Count samples with high lateral G and compare actual yaw rate to expected
    const N = d._N;
    let over=0, under=0, neut=0;
    for(let i=0;i<N;i++){
      const ay = d.ay[i], gz = d.gz[i], vms = d.kf_vel[i];
      if(!isFinite(ay)||!isFinite(gz)||!isFinite(vms)||vms<5) continue;
      if(Math.abs(ay) < 0.3) continue;               // only cornering
      const wExp = (ay * 9.81) / vms * (180/Math.PI); // °/s expected
      const ratio = Math.abs(gz) / Math.max(0.1, Math.abs(wExp));
      if(ratio > 1.2) over++;
      else if(ratio < 0.8) under++;
      else neut++;
    }
    const tot = over+under+neut || 1;
    let verdict = 'Neutral';
    if(over > under*1.3 && over > neut*0.6) verdict = 'Oversteer';
    else if(under > over*1.3 && under > neut*0.6) verdict = 'Understeer';
    return { over, under, neut, verdict, tot };
  }

  // Format helpers
  function fmtTime(s){
    if(!isFinite(s)) return '–';
    const m = Math.floor(s/60), r = s - m*60;
    return m.toString().padStart(2,'0')+':'+r.toFixed(3).padStart(6,'0');
  }
  function fmtDur(s){
    if(!isFinite(s)||s<0) return '–';
    if(s<60) return s.toFixed(1)+'s';
    const m=Math.floor(s/60), sec=Math.floor(s-m*60);
    return m+'m '+sec.toString().padStart(2,'0')+'s';
  }
  function fmtDelta(s){
    if(!isFinite(s)) return '';
    const sign = s>0 ? '+' : (s<0?'−':'');
    const abs = Math.abs(s);
    return sign + abs.toFixed(3);
  }

  window.TelemetryCompute = {
    parseCSV, detectLaps, computeInsights, fmtTime, fmtDur, fmtDelta,
  };
})();
