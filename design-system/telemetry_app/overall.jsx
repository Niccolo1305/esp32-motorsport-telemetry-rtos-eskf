/* overall.jsx — Overall summary view.
   Exports: window.OverallView
*/
const { useEffect, useRef, useState, useMemo } = React;

function Icon({name, className='ic'}){
  const ref = useRef(null);
  useEffect(()=>{
    if(ref.current && window.lucide){ window.lucide.createIcons({ attrs:{class:className}, nameAttr:'data-lucide', icons: window.lucide?.icons });
      // Force re-render of this node
      const el = ref.current;
      el.innerHTML = '';
      const i = document.createElement('i'); i.setAttribute('data-lucide', name); el.appendChild(i);
      window.lucide.createIcons({ attrs:{class:className}});
    }
  }, [name, className]);
  return <span ref={ref} style={{display:'inline-flex'}}></span>;
}

function KPI({label, value, unit, sub, live}){
  return (
    <div className="kpi">
      <div className="l">{live && <span className="dot"/>}{label}</div>
      <div className="v">{value}{unit && <span className="u">{unit}</span>}</div>
      {sub && <div className="sub">{sub}</div>}
    </div>
  );
}

function Insight({tone='info', icon, label, value, meta}){
  return (
    <div className={'insight '+tone}>
      <Icon name={icon} className="ic" />
      <div className="txt">
        <div className="k">{label}</div>
        <div className="v">{value}</div>
      </div>
      {meta && <div className="m">{meta}</div>}
    </div>
  );
}

function HeroMap({d, laps, activeIdx, onHoverPoint, onHoverEnd}){
  const mapRef = useRef(null);
  const instRef = useRef(null);
  const segmentsRef = useRef([]);
  const markerRef = useRef(null);
  const annotMarkersRef = useRef([]);
  const ptsRef = useRef([]);
  const hoverPtsRef = useRef([]);
  const onHoverRef = useRef(onHoverPoint);
  const onHoverEndRef = useRef(onHoverEnd);
  const [hoverInfo, setHoverInfo] = useState(null);
  const [mapSource, setMapSource] = useState('kf');
  const insights = useMemo(()=> TelemetryCompute.computeInsights(d, laps), [d, laps]);
  const mapPrefer = mapSource === 'gps' ? 'gps_only' : 'kf';

  useEffect(()=>{
    onHoverRef.current = onHoverPoint;
    onHoverEndRef.current = onHoverEnd;
  }, [onHoverPoint, onHoverEnd]);

  // Init map once
  useEffect(()=>{
    if(!mapRef.current || instRef.current) return;
    const map = L.map(mapRef.current, {
      zoomControl: true, attributionControl: true,
      preferCanvas: true, fadeAnimation: false,
    });
    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}{r}.png',{
      subdomains:'abcd', maxZoom: 20,
      attribution:'© OpenStreetMap · CARTO',
    }).addTo(map);
    instRef.current = map;
  }, []);

  // Draw the racing-line colored by speed, fit bounds
  useEffect(()=>{
    const map = instRef.current; if(!map) return;
    // Clean previous
    segmentsRef.current.forEach(s => map.removeLayer(s)); segmentsRef.current = [];
    annotMarkersRef.current.forEach(s => map.removeLayer(s)); annotMarkersRef.current = [];

    const pts = TelemetryCharts.mapPoints(d, TelemetryCharts.mapDrawMaxPoints(d), mapPrefer);
    const hoverPts = TelemetryCharts.mapPoints(d, TelemetryCharts.mapHoverMaxPoints(d), mapPrefer);
    ptsRef.current = pts;
    hoverPtsRef.current = hoverPts.length ? hoverPts : pts;
    if(!pts.length){
      if(markerRef.current){ map.removeLayer(markerRef.current); markerRef.current = null; }
      return;
    }

    // Compute v range for colors
    let vmin = Infinity, vmax = -Infinity;
    for(const p of pts){ if(p.v<vmin)vmin=p.v; if(p.v>vmax)vmax=p.v; }
    const span = Math.max(1, vmax - vmin);
    const color = (v) => {
      const t = (v - vmin) / span; // 0..1
      // green → yellow → red
      if(t < 0.5){
        const u = t/0.5;
        const r = Math.round(0 + (255-0)*u);
        const g = Math.round(230 + (179-230)*u);
        const b = Math.round(118 + (0-118)*u);
        return `rgb(${r},${g},${b})`;
      } else {
        const u = (t-0.5)/0.5;
        const r = Math.round(255 + (255-255)*u);
        const g = Math.round(179 + (59-179)*u);
        const b = Math.round(0 + (48-0)*u);
        return `rgb(${r},${g},${b})`;
      }
    };

    // Draw as polyline segments so we can vary color
    for(let i=1;i<pts.length;i++){
      const seg = L.polyline([[pts[i-1].lat, pts[i-1].lon],[pts[i].lat, pts[i].lon]],
        { color: color((pts[i-1].v + pts[i].v)/2), weight: 3, opacity: 0.9 });
      seg.addTo(map);
      segmentsRef.current.push(seg);
    }

    // Start/finish flag
    const start = L.circleMarker([pts[0].lat, pts[0].lon],
      {radius: 5, color: '#fff', weight:2, fillColor:'#000', fillOpacity:1});
    start.addTo(map); annotMarkersRef.current.push(start);

    // Annotate key events
    const mark = (lat, lon, label, color) => {
      if(!lat || !lon) return;
      const m = L.circleMarker([lat, lon],
        {radius: 6, color, weight: 2, fillColor:'#000', fillOpacity:0.9});
      m.bindTooltip(label, { permanent:false, direction:'top',
        className:'tele-tip', offset:[0,-4]});
      m.addTo(map); annotMarkersRef.current.push(m);
    };
    const pal = TelemetryCharts.palette();
    mark(insights.topSpeed.lat, insights.topSpeed.lon, `V max · ${insights.topSpeed.v.toFixed(0)} km/h`, pal.signal);
    mark(insights.hardestBrake.lat, insights.hardestBrake.lon, `Brake · ${insights.hardestBrake.g.toFixed(2)}g`, pal.alert);
    mark(insights.maxLateral.lat, insights.maxLateral.lon, `Lat max · ${insights.maxLateral.g.toFixed(2)}g`, pal.accent);

    // Cursor marker
    if(markerRef.current){ map.removeLayer(markerRef.current); }
    const cur = L.circleMarker([pts[0].lat, pts[0].lon],
      {radius: 7, color:'#fff', weight: 2, fillColor:'#00E676', fillOpacity: 1});
    cur.addTo(map);
    markerRef.current = cur;

    // Fit bounds with padding
    const latlngs = pts.map(p => [p.lat, p.lon]);
    map.fitBounds(L.latLngBounds(latlngs), {padding: [24, 24]});
    setTimeout(()=>map.invalidateSize(), 50);
  }, [d, mapPrefer]);

  // Move cursor marker on sync
  useEffect(()=>{
    const map = instRef.current; if(!map || !markerRef.current || activeIdx==null) return;
    const coord = TelemetryCharts.mapCoordForIndex(d, activeIdx, mapPrefer);
    if(coord) markerRef.current.setLatLng([coord.lat, coord.lon]);
  }, [activeIdx, d, mapPrefer]);

  const clearMapHover = () => {
    setHoverInfo(null);
    if(onHoverEndRef.current) onHoverEndRef.current();
  };

  const handleMapHover = (ev) => {
    if(ev.target && ev.target.closest && ev.target.closest('.leaflet-control')){
      clearMapHover();
      return;
    }
    const map = instRef.current;
    const pts = hoverPtsRef.current.length ? hoverPtsRef.current : ptsRef.current;
    if(!map || !pts || !pts.length) return;
    const nativeEvent = ev.nativeEvent || ev;
    const p = TelemetryCharts.nearestMapPoint(map, pts, map.mouseEventToLatLng(nativeEvent));
    if(!p) return;

    if(markerRef.current) markerRef.current.setLatLng([p.lat, p.lon]);
    if(onHoverRef.current) onHoverRef.current(p.idx);

    const rect = ev.currentTarget.getBoundingClientRect();
    const x = Math.max(10, Math.min(rect.width - 140, ev.clientX - rect.left + 14));
    const y = Math.max(10, Math.min(rect.height - 72, ev.clientY - rect.top - 72));
    setHoverInfo(prev => (
      prev && prev.idx === p.idx && Math.abs(prev.x - x) < 2 && Math.abs(prev.y - y) < 2
        ? prev
        : {idx:p.idx, x, y}
    ));
  };

  return (
    <div className="hero-map"
      onMouseMoveCapture={handleMapHover}
      onPointerMoveCapture={handleMapHover}
      onMouseLeave={clearMapHover}
      onPointerLeave={clearMapHover}>
      <div id="leaflet-map" ref={mapRef}></div>
      <div className="map-source-toggle">
        <span>Source</span>
        <div className="seg mini">
          <button className={mapSource==='kf'?'on':''} onClick={()=>setMapSource('kf')}>ESKF</button>
          <button className={mapSource==='gps'?'on':''} onClick={()=>setMapSource('gps')}>GPS</button>
        </div>
      </div>
      {hoverInfo && (
        <div className="map-hover-float"
          style={{left:hoverInfo.x, top:hoverInfo.y}}
          dangerouslySetInnerHTML={{__html: TelemetryCharts.mapPointHtml(d, hoverInfo.idx)}} />
      )}
      <div className="hero-scale">
        <span>Slow</span>
        <div className="grad"></div>
        <span>Fast</span>
      </div>
      <div className="hero-overlay">
        <div className="chip">Racing line · <b>speed heat</b></div>
        <div className="chip">Source · <b>{mapSource === 'gps' ? 'GPS only' : 'ESKF'}</b></div>
        <div className="chip">Samples · <b>{d._N.toLocaleString()}</b></div>
      </div>
    </div>
  );
}

function SmallPlot({title, freq, chartKey, onReady}){
  const ref = useRef(null);
  useEffect(()=>{
    if(!ref.current) return;
    onReady(ref.current, chartKey);
  }, [chartKey]);
  return (
    <div className="card flat">
      <div className="head">
        <div className="t">{title}</div>
        {freq && <div className="freq">● {freq}</div>}
      </div>
      <div className="body"><div ref={ref} className="plot plot-sm"/></div>
    </div>
  );
}

function OverallView({ d, laps, xAxis, setXAxis, showRaw, setShowRaw, mode3D, setMode3D, onHoverIdx }){
  const plotRefs = useRef({ speed:null, accel:null, yaw:null, gg:null, laps:null, tl:null });
  const cursorElsRef = useRef([]);
  const insights = useMemo(()=> TelemetryCompute.computeInsights(d, laps), [d, laps]);
  const [activeIdx, setActiveIdx] = useState(null);

  const syncRawIndex = (idx, skipEl=null) => {
    if(idx == null) return;
    setActiveIdx(idx);
    if(onHoverIdx) onHoverIdx(idx);
    const xVal = TelemetryCharts.xValueForIndex(d, idx, xAxis);
    TelemetryCharts.setCursorLines(cursorElsRef.current, xVal, skipEl);
  };
  const clearCursorSync = () => {
    TelemetryCharts.clearCursorLines(cursorElsRef.current);
  };

  // Chart init
  useEffect(()=>{
    const r = plotRefs.current;
    if(r.speed) TelemetryCharts.renderSpeed(r.speed, d, xAxis, showRaw);
    if(r.accel) TelemetryCharts.renderAccel(r.accel, d, xAxis, showRaw);
    if(r.yaw)   TelemetryCharts.renderYaw(r.yaw, d, xAxis);
    if(r.gg)    TelemetryCharts.renderGG(r.gg, d, mode3D);
    if(r.laps)  TelemetryCharts.renderLapHistogram(r.laps, laps);
    if(r.tl)    TelemetryCharts.renderTimeline(r.tl, d, xAxis);

    // Bind cursor sync on all x-axis charts
    const syncEls = [r.speed, r.accel, r.yaw, r.tl].filter(Boolean);
    cursorElsRef.current = syncEls;
    const handler = (ev) => {
      if(!ev || !ev.points || !ev.points.length) return;
      const idx = TelemetryCharts.eventRawIndex(ev, d, xAxis);
      syncRawIndex(idx, ev.event?.target?.closest('.plot'));
    };
    syncEls.forEach(el => {
      if(el){
        el.on('plotly_hover', handler);
        el.on('plotly_unhover', clearCursorSync);
      }
    });
    return () => {
      syncEls.forEach(el => {
        if(el && el.removeAllListeners){
          el.removeAllListeners('plotly_hover');
          el.removeAllListeners('plotly_unhover');
        }
      });
    };
  }, [d, laps, xAxis, showRaw, mode3D]);

  const attach = (el, key) => { plotRefs.current[key] = el; };
  const fmt = (n, dec=1) => (isFinite(n)? n.toFixed(dec): '–');
  const bestLap = laps.find(l => l.best);

  return (
    <div className="shell">
      {/* Controls row */}
      <div className="controls">
        <div className="ctl">
          <span className="lbl">x-axis</span>
          <div className="seg">
            <button className={xAxis==='time'?'on':''} onClick={()=>setXAxis('time')}>Time</button>
            <button className={xAxis==='dist'?'on':''} onClick={()=>setXAxis('dist')}>Distance</button>
          </div>
        </div>
        <div className="ctl">
          <span className="lbl">g-g</span>
          <div className="seg">
            <button className={!mode3D?'on':''} onClick={()=>setMode3D(false)}>2D</button>
            <button className={mode3D?'on':''} onClick={()=>setMode3D(true)}>3D</button>
          </div>
        </div>
        <div className="ctl">
          <div className={'toggle '+(showRaw?'on':'')} onClick={()=>setShowRaw(!showRaw)}>
            <div className="sw"></div>
            <span>Raw overlay</span>
          </div>
        </div>
        <div className="ctl" style={{marginLeft:'auto'}}>
          <span className="lbl" style={{marginRight:8}}>Laps · {laps.length}</span>
          <span className="lbl">Duration · {TelemetryCompute.fmtDur(d._duration_s)}</span>
        </div>
      </div>

      {/* KPI strip */}
      <div className="kpis">
        <KPI live label="V max" value={insights.topSpeed.v.toFixed(1)} unit="km/h"
             sub={`L${insights.topSpeed.lap} · t+${insights.topSpeed.t.toFixed(1)}s`}/>
        <KPI label="G long max" value={insights.hardestBrake.g.toFixed(2)} unit="g"
             sub={`brake @ ${insights.hardestBrake.speed.toFixed(0)} km/h`}/>
        <KPI label="G lat max" value={insights.maxLateral.g.toFixed(2)} unit="g"
             sub={`${insights.maxLateral.side==='L'?'left':'right'} · ${insights.maxLateral.speed.toFixed(0)} km/h`}/>
        <KPI label="Distance" value={(insights.distance_m/1000).toFixed(2)} unit="km"/>
        <KPI label="Duration" value={TelemetryCompute.fmtDur(insights.duration_s)} unit=""/>
        <KPI label="Best lap" value={bestLap ? TelemetryCompute.fmtTime(bestLap.duration) : '–'} unit=""
             sub={bestLap ? `L${bestLap.lap}` : 'n/a'}/>
      </div>

      {/* Hero map + insights side */}
      <div className="row-map-insights">
        <div className="card flat">
          <div className="head">
            <div className="t">Track · racing line</div>
            <div className="freq">● speed heat · {d._N.toLocaleString()} samples</div>
          </div>
          <HeroMap d={d} laps={laps} activeIdx={activeIdx}
            onHoverPoint={syncRawIndex} onHoverEnd={clearCursorSync}/>
        </div>
        <div className="card">
          <div className="head">
            <div className="t">Insights</div>
            <div className="freq">● auto</div>
          </div>
          <div className="body">
            <div className="insights">
              <Insight tone="ok" icon="gauge" label="Top speed"
                value={`${insights.topSpeed.v.toFixed(1)} km/h`}
                meta={`L${insights.topSpeed.lap}`} />
              <Insight tone="alert" icon="circle-slash-2" label="Hardest brake"
                value={`${insights.hardestBrake.g.toFixed(2)} g @ ${insights.hardestBrake.speed.toFixed(0)} km/h`}
                meta={`L${insights.hardestBrake.lap}`} />
              <Insight tone="info" icon="rotate-3d" label="Max lateral"
                value={`${insights.maxLateral.g.toFixed(2)} g ${insights.maxLateral.side==='L'?'LEFT':'RIGHT'}`}
                meta={`L${insights.maxLateral.lap}`} />
              <Insight tone="info" icon="percent" label="G-G coverage"
                value={`${insights.ggCoverage.pct.toFixed(0)}% used`}
                meta={`peak ${insights.ggCoverage.Gmax.toFixed(2)}g`} />
              <Insight tone={insights.balance.verdict==='Neutral'?'ok':'warn'} icon="scale" label="Chassis balance"
                value={insights.balance.verdict}
                meta={`${insights.balance.over}/${insights.balance.neut}/${insights.balance.under}`} />
              <Insight tone="info" icon="wind" label="Yaw stability"
                value={`σ ${insights.yawStab.std.toFixed(1)}°/s`}
                meta={`μ ${insights.yawStab.mean.toFixed(1)}`} />
              <Insight tone="info" icon="clock" label="Time distribution"
                value={`B ${insights.timeDist.brakePct.toFixed(0)}%  ·  T ${insights.timeDist.accelPct.toFixed(0)}%  ·  C ${insights.timeDist.coastPct.toFixed(0)}%`}
                meta="" />
            </div>
          </div>
        </div>
      </div>

      {/* Mini chart row: Speed / Accel / Yaw */}
      <div className="row-3" style={{marginBottom:12}}>
        <SmallPlot title="Speed" freq="km/h" chartKey="speed" onReady={attach}/>
        <SmallPlot title="Accel ax / ay" freq="g" chartKey="accel" onReady={attach}/>
        <SmallPlot title="Yaw rate gz" freq="°/s" chartKey="yaw" onReady={attach}/>
      </div>

      {/* G-G + Lap hist */}
      <div className="row-2" style={{marginBottom:12}}>
        <div className="card flat">
          <div className="head">
            <div className="t">G-G diagram · {mode3D?'3D (speed on Z)':'2D friction disc'}</div>
            <div className="freq">● scatter · colored by speed</div>
          </div>
          <div className="body"><div ref={el=>attach(el,'gg')} className="plot plot-md"/></div>
        </div>
        <div className="card flat">
          <div className="head">
            <div className="t">Laps · times</div>
            <div className="freq">● best highlighted</div>
          </div>
          <div className="body" style={{display:'grid', gridTemplateColumns:'1fr 260px', gap:12}}>
            <div ref={el=>attach(el,'laps')} className="plot plot-md"/>
            <div style={{borderLeft:'1px solid var(--color-border-1)', paddingLeft:12, overflow:'auto', maxHeight:280}}>
              <table>
                <thead>
                  <tr><th>Lap</th><th>Time</th><th>Δ best</th></tr>
                </thead>
                <tbody>
                  {laps.map(l => (
                    <tr key={l.lap} className={l.best?'best':''}>
                      <td>L{l.lap}</td>
                      <td>{TelemetryCompute.fmtTime(l.duration)}</td>
                      <td>{l.best ? '—' : TelemetryCompute.fmtDelta(l.delta)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>
        </div>
      </div>

      {/* Full-width timeline */}
      <div className="card flat" style={{marginBottom:12}}>
        <div className="head">
          <div className="t">Timeline · scroll & hover syncs map + charts</div>
          <div className="freq">● speed · accel · yaw</div>
        </div>
        <div className="body"><div ref={el=>attach(el,'tl')} className="plot plot-tl"/></div>
      </div>
    </div>
  );
}

window.OverallView = OverallView;
