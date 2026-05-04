/* charts.jsx - Plotly chart builders & helpers.
   Globals: window.TelemetryCharts
*/
(function(){
  'use strict';

  function palette(){
    const style = getComputedStyle(document.body);
    const css = (k) => style.getPropertyValue(k).trim();
    return {
      bg:        css('--color-bg-2') || '#131519',
      fg:        css('--color-fg-1') || '#E7E9EC',
      fg2:       css('--color-fg-2') || '#9FA4AC',
      fg3:       css('--color-fg-3') || '#6C7078',
      grid:      css('--color-grid') || 'rgba(255,255,255,0.04)',
      border:    css('--color-border-2') || 'rgba(255,255,255,0.12)',
      signal:    css('--color-signal') || '#00E676',
      accent:    css('--color-accent') || '#39C7FF',
      warn:      css('--color-warn')   || '#FFB300',
      alert:     css('--color-alert')  || '#FF3B30',
      magenta:   css('--color-magenta')|| css('--color-warn') || '#FFB300',
      mono:      css('--font-mono')    || 'JetBrains Mono, monospace',
    };
  }

  const baseLayout = (p, overrides = {}) => ({
    paper_bgcolor: p.bg,
    plot_bgcolor: p.bg,
    margin: {l: 48, r: 18, t: 8, b: 28},
    font: { family: p.mono, size: 10, color: p.fg2 },
    xaxis: {
      gridcolor: p.grid, linecolor: p.border, zerolinecolor: p.grid,
      tickfont: {color: p.fg3, size: 9}, ticklen: 3, tickcolor: p.border,
      showspikes: true, spikemode:'across', spikethickness:1, spikecolor: p.fg3, spikedash:'dot',
    },
    yaxis: {
      gridcolor: p.grid, linecolor: p.border, zerolinecolor: p.grid,
      tickfont: {color: p.fg3, size: 9}, ticklen: 3, tickcolor: p.border,
    },
    hovermode: 'x unified',
    hoverlabel: {
      bgcolor: p.bg, bordercolor: p.border, font: {family: p.mono, size: 10, color: p.fg}
    },
    showlegend: false,
    ...overrides,
  });

  const baseConfig = {
    displaylogo: false,
    responsive: true,
    modeBarButtonsToRemove: ['autoScale2d','lasso2d','select2d','toggleSpikelines'],
    toImageButtonOptions: { format:'png', filename:'telemetry', height: 700, width: 1600, scale: 2 }
  };

  function decimate(arrs, maxPoints){
    const N = arrs[0].length;
    if(N <= maxPoints) return arrs;
    const step = Math.ceil(N / maxPoints);
    return arrs.map(a => {
      const out = new Float32Array(Math.ceil(N/step));
      let j = 0;
      for(let i=0;i<N;i+=step){ out[j++] = a[i]; }
      return out;
    });
  }

  function getSeries(d, channels, xAxis, maxPoints){
    if(d.getSeries){
      return d.getSeries({channels, xAxis, maxPoints});
    }
    const x = xAxis === 'dist' ? d.distance_m : d.t_s;
    const dec = decimate([x].concat(channels.map(name => d[name])), maxPoints);
    const out = {x: dec[0], idx: null};
    channels.forEach((name, i) => { out[name] = dec[i + 1]; });
    return out;
  }

  function hasRaw(d){
    if(d._hasRaw) return true;
    if(!d.raw_ax) return false;
    for(let i=0; i<d.raw_ax.length; i++){
      if(Number.isFinite(d.raw_ax[i])) return true;
    }
    return false;
  }

  function xTitle(xAxis){
    return xAxis === 'dist' ? 'distance (m)' : 'time (s)';
  }

  function renderSpeed(el, d, xAxis, showRaw){
    const p = palette();
    const s = getSeries(d, ['vel_kmh'], xAxis, 2500);
    const traces = [{
      x: s.x, y: s.vel_kmh, customdata: s.idx, type:'scattergl', mode:'lines',
      line: {color: p.signal, width: 1.5}, name:'v',
      hovertemplate:'%{y:.1f} km/h<extra></extra>',
    }];
    const layout = baseLayout(p, {
      xaxis: { ...baseLayout(p).xaxis, title: {text: xTitle(xAxis), font:{size:9, color:p.fg3}}, showgrid:true },
      yaxis: { ...baseLayout(p).yaxis, title: {text:'SPEED (km/h)', font:{size:9, color:p.fg3}, standoff:2}, rangemode:'tozero'},
    });
    Plotly.newPlot(el, traces, layout, baseConfig);
  }

  function renderAccel(el, d, xAxis, showRaw){
    const p = palette();
    const raw = showRaw && hasRaw(d);
    const channels = raw ? ['ax','ay','raw_ax','raw_ay'] : ['ax','ay'];
    const s = getSeries(d, channels, xAxis, 2500);
    const traces = [
      { x: s.x, y: s.ax, customdata: s.idx, type:'scattergl', mode:'lines',
        line:{color: p.alert, width: 1.3}, name:'ax', hovertemplate:'ax %{y:.2f}g<extra></extra>'},
      { x: s.x, y: s.ay, customdata: s.idx, type:'scattergl', mode:'lines',
        line:{color: p.accent, width: 1.3}, name:'ay', hovertemplate:'ay %{y:.2f}g<extra></extra>'},
    ];
    if(raw){
      traces.push({x:s.x, y:s.raw_ax, customdata:s.idx, type:'scattergl', mode:'lines',
        line:{color: p.alert, width: .8, dash:'dot'}, opacity:.5, name:'raw ax', hoverinfo:'skip'});
      traces.push({x:s.x, y:s.raw_ay, customdata:s.idx, type:'scattergl', mode:'lines',
        line:{color: p.accent, width: .8, dash:'dot'}, opacity:.5, name:'raw ay', hoverinfo:'skip'});
    }
    const layout = baseLayout(p, {
      xaxis: { ...baseLayout(p).xaxis, title: {text: xTitle(xAxis), font:{size:9, color:p.fg3}}, showgrid:true },
      yaxis: { ...baseLayout(p).yaxis, title: {text:'ACCEL (g)', font:{size:9, color:p.fg3}, standoff:2}, zeroline:true, zerolinecolor: p.border},
    });
    Plotly.newPlot(el, traces, layout, baseConfig);
  }

  function renderYaw(el, d, xAxis){
    const p = palette();
    const s = getSeries(d, ['gz'], xAxis, 2500);
    const traces = [{
      x: s.x, y: s.gz, customdata: s.idx, type:'scattergl', mode:'lines',
      line:{color: p.magenta, width: 1.3},
      hovertemplate:'%{y:.1f} deg/s<extra></extra>',
    }];
    const layout = baseLayout(p, {
      xaxis: { ...baseLayout(p).xaxis, title: {text: xTitle(xAxis), font:{size:9, color:p.fg3}}, showgrid:true },
      yaxis: { ...baseLayout(p).yaxis, title: {text:'YAW (deg/s)', font:{size:9, color:p.fg3}, standoff:2}, zeroline:true, zerolinecolor: p.border},
    });
    Plotly.newPlot(el, traces, layout, baseConfig);
  }

  function renderGG(el, d, mode3D){
    const p = palette();
    const s = getSeries(d, ['ax','ay','vel_kmh'], 'time', 4000);
    const ax = s.ax, ay = s.ay, v = s.vel_kmh;

    if(mode3D){
      const traces = [{
        type:'scatter3d', mode:'markers',
        x: Array.from(ay), y: Array.from(ax), z: Array.from(v),
        marker: { size: 2, color: Array.from(v), colorscale:[[0,p.signal],[0.5,p.warn],[1,p.alert]],
          showscale:false, opacity:0.7 },
        hovertemplate:'ay %{x:.2f} / ax %{y:.2f} / %{z:.0f} km/h<extra></extra>',
      }];
      const layout = baseLayout(p, {
        margin:{l:0,r:0,t:0,b:0},
        scene: {
          bgcolor: p.bg,
          xaxis: {title:'ay (g)', gridcolor: p.grid, color: p.fg3, backgroundcolor: p.bg, showbackground:true},
          yaxis: {title:'ax (g)', gridcolor: p.grid, color: p.fg3, backgroundcolor: p.bg, showbackground:true},
          zaxis: {title:'v (km/h)', gridcolor: p.grid, color: p.fg3, backgroundcolor: p.bg, showbackground:true},
          camera: {eye: {x:1.4, y:-1.4, z:1}},
        },
      });
      Plotly.newPlot(el, traces, layout, baseConfig);
      return;
    }

    const traces = [{
      x: Array.from(ay), y: Array.from(ax), type:'scattergl', mode:'markers',
      marker: {
        size: 3, color: Array.from(v),
        colorscale:[[0,p.signal],[0.5,p.warn],[1,p.alert]],
        showscale:true,
        colorbar:{ title:{text:'km/h', font:{size:9, color:p.fg3}}, tickfont:{size:9, color:p.fg3},
          outlinewidth:0, thickness:8, len:0.7, x:1.02, xpad:0, tickcolor:p.border},
        opacity: 0.55,
      },
      hovertemplate:'ay %{x:.2f}g / ax %{y:.2f}g<extra></extra>',
    }];

    const layoutShapes = [];
    for(const r of [0.5, 1.0, 1.5]){
      layoutShapes.push({type:'circle', xref:'x', yref:'y',
        x0:-r, y0:-r, x1:r, y1:r,
        line:{color: p.border, width:1, dash: r===1.0 ? 'solid':'dot'}});
    }
    layoutShapes.push({type:'line', x0:0, y0:-2, x1:0, y1:2, line:{color:p.border, width:1}});
    layoutShapes.push({type:'line', x0:-2, y0:0, x1:2, y1:0, line:{color:p.border, width:1}});

    const layout = baseLayout(p, {
      margin:{l:44, r:60, t:16, b:40},
      xaxis: { ...baseLayout(p).xaxis, title:{text:'ay lateral (g)', font:{size:9, color:p.fg3}}, range:[-1.8,1.8], zeroline:false, scaleanchor:'y'},
      yaxis: { ...baseLayout(p).yaxis, title:{text:'ax long. (g)', font:{size:9, color:p.fg3}}, range:[-1.8,1.8], zeroline:false},
      shapes: layoutShapes,
      hovermode:'closest',
      annotations: [
        {x:0,y:1.55,text:'ACCEL',font:{color:p.fg3, size:9}, showarrow:false},
        {x:0,y:-1.55,text:'BRAKE',font:{color:p.fg3, size:9}, showarrow:false},
        {x:1.55,y:0,text:'LEFT',font:{color:p.fg3, size:9}, showarrow:false, textangle:-90},
        {x:-1.55,y:0,text:'RIGHT',font:{color:p.fg3, size:9}, showarrow:false, textangle:90},
      ]
    });
    Plotly.newPlot(el, traces, layout, baseConfig);
  }

  function renderLapHistogram(el, laps){
    const p = palette();
    const x = laps.map(l => 'L'+l.lap);
    const y = laps.map(l => l.duration);
    const colors = laps.map(l => l.best ? p.signal : p.fg2);
    const traces = [{
      x, y, type:'bar', marker:{color: colors, line:{color: p.border, width: 0}},
      hovertemplate:'%{x}: %{y:.3f}s<extra></extra>',
      text: laps.map(l => TelemetryCompute.fmtTime(l.duration)),
      textposition:'outside',
      textfont:{family:p.mono, color:p.fg2, size:10},
    }];
    const layout = baseLayout(p, {
      margin:{l:48, r:18, t:24, b:32},
      xaxis: {...baseLayout(p).xaxis, showgrid:false},
      yaxis: {...baseLayout(p).yaxis, title:{text:'LAP TIME (s)', font:{size:9, color:p.fg3}, standoff:2}},
      bargap:0.35,
    });
    Plotly.newPlot(el, traces, layout, baseConfig);
  }

  function renderTimeline(el, d, xAxis){
    const p = palette();
    const s = getSeries(d, ['vel_kmh','ax','ay','gz'], xAxis, 4000);
    const traces = [
      { x: s.x, y: s.vel_kmh, customdata:s.idx, type:'scattergl', mode:'lines', yaxis:'y',
        line:{color:p.signal, width:1.5}, name:'v (km/h)',
        hovertemplate:'%{y:.1f} km/h<extra></extra>' },
      { x: s.x, y: s.ax, customdata:s.idx, type:'scattergl', mode:'lines', yaxis:'y2',
        line:{color:p.alert, width:1.2}, name:'ax (g)',
        hovertemplate:'ax %{y:.2f}g<extra></extra>' },
      { x: s.x, y: s.ay, customdata:s.idx, type:'scattergl', mode:'lines', yaxis:'y2',
        line:{color:p.accent, width:1.2}, name:'ay (g)',
        hovertemplate:'ay %{y:.2f}g<extra></extra>' },
      { x: s.x, y: s.gz, customdata:s.idx, type:'scattergl', mode:'lines', yaxis:'y3',
        line:{color:p.magenta, width:1.2}, name:'yaw (deg/s)',
        hovertemplate:'%{y:.1f} deg/s<extra></extra>' },
    ];
    const layout = baseLayout(p, {
      margin:{l:48, r:48, t:10, b:32},
      xaxis: { ...baseLayout(p).xaxis, domain:[0,1],
        title:{text: xTitle(xAxis), font:{size:9, color:p.fg3}}, showgrid:true},
      yaxis:  { ...baseLayout(p).yaxis, domain:[0.55, 1.0],
        title:{text:'SPEED (km/h)', font:{size:9, color:p.fg3}, standoff:2}, rangemode:'tozero'},
      yaxis2: { ...baseLayout(p).yaxis, domain:[0.28, 0.5],
        title:{text:'ACCEL (g)', font:{size:9, color:p.fg3}, standoff:2}, zeroline:true, zerolinecolor:p.border},
      yaxis3: { ...baseLayout(p).yaxis, domain:[0.0, 0.22],
        title:{text:'YAW (deg/s)', font:{size:9, color:p.fg3}, standoff:2}, zeroline:true, zerolinecolor:p.border},
      hovermode:'x unified',
      showlegend: true,
      legend: { orientation:'h', y: 1.08, x:0,
        font:{family:p.mono, size:10, color:p.fg2}, bgcolor:'rgba(0,0,0,0)'}
    });
    Plotly.newPlot(el, traces, layout, baseConfig);
  }

  function eventRawIndex(ev, d, xAxis){
    if(ev && ev.points && ev.points.length){
      const cd = ev.points[0].customdata;
      if(cd !== undefined && cd !== null && Number.isFinite(+cd)) return +cd | 0;
      return findSampleAtX(d, ev.points[0].x, xAxis);
    }
    return null;
  }

  function bindCursorSync(els, d, xAxis){
    const handler = (ev) => {
      if(!ev || !ev.points || !ev.points.length) return;
      const xVal = ev.points[0].x;
      if(window.__onXSync) window.__onXSync(xVal, xAxis);
    };
    const unbinders = [];
    els.forEach(el => {
      if(!el) return;
      el.on && el.on('plotly_hover', handler);
      unbinders.push(()=>{ el.removeAllListeners && el.removeAllListeners('plotly_hover'); });
    });
    return () => unbinders.forEach(fn => fn());
  }

  function findSampleAtX(d, xVal, xAxis){
    const arr = xAxis==='dist' ? d.distance_m : d.t_s;
    let lo=0, hi=arr.length-1;
    while(lo<hi){
      const mid = (lo+hi)>>1;
      if(arr[mid] < xVal) lo = mid+1; else hi = mid;
    }
    return lo;
  }

  function xValueForIndex(d, idx, xAxis){
    if(idx == null) return null;
    const arr = xAxis === 'dist' ? d.distance_m : d.t_s;
    const xVal = arr && arr[idx];
    return Number.isFinite(xVal) ? xVal : null;
  }

  function setCursorLines(els, xVal, skipEl=null){
    if(!Number.isFinite(xVal)) return;
    (els || []).forEach(el => {
      if(!el || el === skipEl || !window.Plotly) return;
      Plotly.relayout(el, {
        shapes: [{
          type:'line', yref:'paper', y0:0, y1:1, x0:xVal, x1:xVal,
          line:{color:'rgba(255,255,255,0.35)', width:1, dash:'dot'}
        }]
      });
    });
  }

  function clearCursorLines(els){
    (els || []).forEach(el => {
      if(el && window.Plotly) Plotly.relayout(el, {shapes: []});
    });
  }

  function mapSeries(d, maxPoints){
    if(d.getSeries){
      return d.getSeries({channels:['gps_lat','gps_lon','kf_lat','kf_lon','vel_kmh'], xAxis:'time', maxPoints});
    }
    return getSeries(d, ['gps_lat','gps_lon','kf_lat','kf_lon','vel_kmh'], 'time', maxPoints);
  }

  function lastFinite(arr){
    if(!arr) return NaN;
    for(let i=arr.length-1; i>=0; i--){
      if(Number.isFinite(arr[i])) return arr[i];
    }
    return NaN;
  }

  function clamp(n, lo, hi){
    if(hi < lo) return hi;
    return Math.max(lo, Math.min(hi, n));
  }

  function mapDrawMaxPoints(d){
    const n = d._N || (d.t_s && d.t_s.length) || 2500;
    const distance = lastFinite(d.distance_m);
    const byDistance = Number.isFinite(distance) && distance > 0 ? Math.ceil(distance / 20) : 0;
    return clamp(Math.max(2500, byDistance), 2500, Math.min(8000, n));
  }

  function mapHoverMaxPoints(d){
    const n = d._N || (d.t_s && d.t_s.length) || 8000;
    const duration = d._duration_s || lastFinite(d.t_s);
    const distance = lastFinite(d.distance_m);
    const byTime = Number.isFinite(duration) && duration > 0 ? Math.ceil(duration * 8) : 0;
    const byDistance = Number.isFinite(distance) && distance > 0 ? Math.ceil(distance / 3) : 0;
    return clamp(Math.max(8000, byTime, byDistance), 2500, Math.min(60000, n));
  }

  function validCoordCount(lat, lon){
    if(!lat || !lon) return 0;
    let count = 0;
    for(let i=0; i<lat.length; i++){
      if(Number.isFinite(lat[i]) && Number.isFinite(lon[i]) && lat[i] !== 0 && lon[i] !== 0) count++;
    }
    return count;
  }

  function chooseMapCoords(series, prefer='kf'){
    const kfValid = validCoordCount(series.kf_lat, series.kf_lon);
    const gpsValid = validCoordCount(series.gps_lat, series.gps_lon);
    if(prefer === 'gps' && gpsValid > 1) return {lat:series.gps_lat, lon:series.gps_lon, source:'gps'};
    if(kfValid > 1) return {lat:series.kf_lat, lon:series.kf_lon, source:'kf'};
    if(gpsValid > 1) return {lat:series.gps_lat, lon:series.gps_lon, source:'gps'};
    return {lat:series.kf_lat || series.gps_lat, lon:series.kf_lon || series.gps_lon, source:'none'};
  }

  function mapPoints(d, maxPoints, prefer='kf'){
    const s = mapSeries(d, maxPoints);
    const pos = chooseMapCoords(s, prefer);
    const pts = [];
    const lat = pos.lat || [];
    const lon = pos.lon || [];
    for(let i=0; i<lat.length; i++){
      if(Number.isFinite(lat[i]) && Number.isFinite(lon[i]) && lat[i] !== 0 && lon[i] !== 0){
        pts.push({lat:lat[i], lon:lon[i], v:s.vel_kmh[i], idx:s.idx ? s.idx[i] : i});
      }
    }
    pts.source = pos.source;
    return pts;
  }

  function mapPointHtml(d, idx){
    const t = d.t_s && d.t_s[idx];
    const v = d.vel_kmh && d.vel_kmh[idx];
    const dist = d.distance_m && d.distance_m[idx];
    const time = Number.isFinite(t) && window.TelemetryCompute ? TelemetryCompute.fmtTime(t) : '--:--.---';
    const speed = Number.isFinite(v) ? v.toFixed(1) : '--';
    const distance = Number.isFinite(dist) ? (dist/1000).toFixed(3) : '--';
    return `<div class="map-hover-card">
      <div><span>t</span><b>${time}</b></div>
      <div><span>v</span><b>${speed} km/h</b></div>
      <div><span>d</span><b>${distance} km</b></div>
    </div>`;
  }

  function nearestMapPoint(map, pts, latlng, maxPx=Infinity){
    if(!map || !pts || !pts.length || !latlng) return null;
    const target = map.latLngToLayerPoint(latlng);
    const bounds = map.getPixelBounds();
    const cacheKey = [
      map.getZoom(),
      bounds.min.x, bounds.min.y,
      bounds.max.x, bounds.max.y,
      pts.length,
    ].join('|');
    if(pts._pxKey !== cacheKey){
      const projected = new Array(pts.length);
      for(let i=0; i<pts.length; i++){
        const lp = map.latLngToLayerPoint([pts[i].lat, pts[i].lon]);
        projected[i] = {x:lp.x, y:lp.y};
      }
      pts._px = projected;
      pts._pxKey = cacheKey;
    }
    const projected = pts._px || [];
    let best = null;
    let bestD2 = Number.isFinite(maxPx) ? maxPx * maxPx : Infinity;
    for(let i=0; i<pts.length; i++){
      const lp = projected[i];
      const dx = lp.x - target.x;
      const dy = lp.y - target.y;
      const d2 = dx*dx + dy*dy;
      if(d2 <= bestD2){
        bestD2 = d2;
        best = pts[i];
      }
    }
    return best;
  }

  function bindMapHover(map, ptsRef, d, onHover, onLeave){
    const container = map && map.getContainer ? map.getContainer() : null;
    if(!container) return () => {};
    let tip = null;
    let raf = null;
    let lastEvent = null;
    let lastIdx = null;

    const getPts = () => {
      if(typeof ptsRef === 'function') return ptsRef() || [];
      return ptsRef && ptsRef.current ? ptsRef.current : (ptsRef || []);
    };
    const closeTip = () => {
      if(tip){
        map.removeLayer(tip);
        tip = null;
      }
    };
    const leave = () => {
      if(raf) cancelAnimationFrame(raf);
      raf = null;
      lastEvent = null;
      lastIdx = null;
      closeTip();
      if(onLeave) onLeave();
    };
    const render = () => {
      raf = null;
      const ev = lastEvent;
      if(!ev) return;
      const latlng = ev.latlng || map.mouseEventToLatLng(ev);
      const p = nearestMapPoint(map, getPts(), latlng);
      if(!p){
        leave();
        return;
      }
      if(!tip){
        tip = L.tooltip({
          className:'telemetry-map-tooltip',
          direction:'top',
          offset:[0,-10],
          opacity:1,
          interactive:false,
        }).addTo(map);
      }
      tip.setLatLng([p.lat, p.lon]).setContent(mapPointHtml(d, p.idx));
      if(p.idx !== lastIdx){
        lastIdx = p.idx;
        if(onHover) onHover(p.idx, p);
      }
    };
    const move = (ev) => {
      if(ev.target && ev.target.closest && ev.target.closest('.leaflet-control')){
        leave();
        return;
      }
      lastEvent = ev;
      if(!raf) raf = requestAnimationFrame(render);
    };

    container.addEventListener('mousemove', move, {capture:true, passive:true});
    container.addEventListener('pointermove', move, {capture:true, passive:true});
    container.addEventListener('mouseleave', leave, true);
    container.addEventListener('pointerleave', leave, true);
    map.on('zoomstart movestart', leave);
    return () => {
      container.removeEventListener('mousemove', move, true);
      container.removeEventListener('pointermove', move, true);
      container.removeEventListener('mouseleave', leave, true);
      container.removeEventListener('pointerleave', leave, true);
      map.off('zoomstart movestart', leave);
      leave();
    };
  }

  window.TelemetryCharts = {
    palette,
    renderSpeed, renderAccel, renderYaw,
    renderGG, renderLapHistogram, renderTimeline,
    bindCursorSync, findSampleAtX, eventRawIndex, mapSeries,
    mapDrawMaxPoints, mapHoverMaxPoints, mapPoints,
    xValueForIndex, setCursorLines, clearCursorLines,
    mapPointHtml, nearestMapPoint, bindMapHover,
  };
})();
