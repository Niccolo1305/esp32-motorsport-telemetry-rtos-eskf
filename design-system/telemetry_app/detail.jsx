/* detail.jsx — Dense engineering detail view (4-panel, like dashboard.py).
   Exports: window.DetailView
*/
const { useEffect: useEffD, useRef: useRefD, useState: useStateD, useMemo: useMemoD } = React;

function DetailView({d, laps, xAxis, setXAxis, showRaw, setShowRaw, mode3D, setMode3D, onHoverIdx}){
  const plotRefs = useRefD({ map:null, gg:null, yaw:null, tl:null });
  const mapRef = useRefD(null);
  const mapInstRef = useRefD(null);
  const mapLineRef = useRefD([]);
  const mapCurRef = useRefD(null);

  // Init Leaflet
  useEffD(()=>{
    if(!mapRef.current || mapInstRef.current) return;
    const m = L.map(mapRef.current, { preferCanvas:true });
    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}{r}.png',
      { subdomains:'abcd', maxZoom:20, attribution:'© OSM · CARTO' }).addTo(m);
    mapInstRef.current = m;
  }, []);

  // Draw map + charts
  useEffD(()=>{
    const map = mapInstRef.current;
    if(map){
      mapLineRef.current.forEach(s=>map.removeLayer(s)); mapLineRef.current=[];
      const s = TelemetryCharts.mapSeries(d, 2500);
      const lat = d.gps_lat.some(v=>v&&v!==0)?s.gps_lat:s.kf_lat;
      const lon = d.gps_lon.some(v=>v&&v!==0)?s.gps_lon:s.kf_lon;
      const pts = [];
      for(let i=0;i<lat.length;i++){ if(isFinite(lat[i])&&isFinite(lon[i])&&lat[i]!==0) pts.push({lat:lat[i], lon:lon[i], v:s.vel_kmh[i]}); }
      if(pts.length){
        let vmin=Infinity,vmax=-Infinity; for(const p of pts){ if(p.v<vmin)vmin=p.v; if(p.v>vmax)vmax=p.v;}
        const span = Math.max(1,vmax-vmin);
        const col = (v)=>{ const t=(v-vmin)/span;
          const r = t<0.5 ? Math.round((255-0)*(t/0.5)) : 255;
          const g = t<0.5 ? Math.round(230+(179-230)*(t/0.5)) : Math.round(179+(59-179)*((t-0.5)/0.5));
          const b = t<0.5 ? Math.round(118+(0-118)*(t/0.5)) : Math.round((48-0)*((t-0.5)/0.5));
          return `rgb(${r},${g},${b})`; };
        for(let i=1;i<pts.length;i++){
          const seg = L.polyline([[pts[i-1].lat,pts[i-1].lon],[pts[i].lat,pts[i].lon]],
            { color: col((pts[i-1].v+pts[i].v)/2), weight:3, opacity:.9 });
          seg.addTo(map); mapLineRef.current.push(seg);
        }
        if(!mapCurRef.current){
          mapCurRef.current = L.circleMarker([pts[0].lat,pts[0].lon],
            {radius:7,color:'#fff',weight:2,fillColor:'#00E676',fillOpacity:1}).addTo(map);
        }
        map.fitBounds(L.latLngBounds(pts.map(p=>[p.lat,p.lon])), {padding:[16,16]});
        setTimeout(()=>map.invalidateSize(), 50);
      }
    }
    const r = plotRefs.current;
    if(r.gg)  TelemetryCharts.renderGG(r.gg, d, mode3D);
    if(r.yaw) TelemetryCharts.renderYaw(r.yaw, d, xAxis);
    if(r.tl)  TelemetryCharts.renderTimeline(r.tl, d, xAxis);

    const syncEls = [r.yaw, r.tl].filter(Boolean);
    const handler = (ev)=>{
      if(!ev||!ev.points||!ev.points.length) return;
      const xVal = ev.points[0].x;
      const idx = TelemetryCharts.eventRawIndex(ev, d, xAxis);
      if(onHoverIdx) onHoverIdx(idx);
      const lat = d.gps_lat[idx]||d.kf_lat[idx];
      const lon = d.gps_lon[idx]||d.kf_lon[idx];
      if(mapCurRef.current && lat && lon) mapCurRef.current.setLatLng([lat,lon]);
      syncEls.forEach(el => {
        if(el){ Plotly.relayout(el, {shapes:[{type:'line', yref:'paper', y0:0, y1:1, x0:xVal, x1:xVal,
          line:{color:'rgba(255,255,255,0.35)', width:1, dash:'dot'}}]}); }
      });
    };
    syncEls.forEach(el => { if(el){ el.on('plotly_hover', handler);
      el.on('plotly_unhover', ()=>{ syncEls.forEach(e=>Plotly.relayout(e,{shapes:[]}));}); }});
    return ()=>{ syncEls.forEach(el => { if(el) el.removeAllListeners && el.removeAllListeners('plotly_hover'); }); };
  }, [d, laps, xAxis, showRaw, mode3D]);

  const attach = (el, key) => { plotRefs.current[key] = el; };

  return (
    <div className="shell">
      <div className="controls">
        <div className="ctl"><span className="lbl">x-axis</span>
          <div className="seg">
            <button className={xAxis==='time'?'on':''} onClick={()=>setXAxis('time')}>Time</button>
            <button className={xAxis==='dist'?'on':''} onClick={()=>setXAxis('dist')}>Distance</button>
          </div>
        </div>
        <div className="ctl"><span className="lbl">g-g</span>
          <div className="seg">
            <button className={!mode3D?'on':''} onClick={()=>setMode3D(false)}>2D</button>
            <button className={mode3D?'on':''} onClick={()=>setMode3D(true)}>3D</button>
          </div>
        </div>
        <div className="ctl">
          <div className={'toggle '+(showRaw?'on':'')} onClick={()=>setShowRaw(!showRaw)}>
            <div className="sw"></div><span>Raw overlay</span>
          </div>
        </div>
      </div>

      <div className="row-2" style={{marginBottom:12}}>
        <div className="card flat" style={{height:440}}>
          <div className="head"><div className="t">GPS · track (speed heat)</div><div className="freq">● racing line</div></div>
          <div ref={mapRef} style={{height:'calc(100% - 42px)', background:'#000'}}></div>
        </div>
        <div className="card flat" style={{height:440}}>
          <div className="head"><div className="t">G-G {mode3D?'3D':'2D'}</div><div className="freq">● friction disc</div></div>
          <div className="body" style={{height:'calc(100% - 42px)'}}><div ref={el=>attach(el,'gg')} style={{height:'100%'}}/></div>
        </div>
      </div>

      <div className="card flat" style={{marginBottom:12}}>
        <div className="head"><div className="t">Yaw rate (gz)</div><div className="freq">● °/s</div></div>
        <div className="body"><div ref={el=>attach(el,'yaw')} className="plot plot-sm"/></div>
      </div>

      <div className="card flat">
        <div className="head"><div className="t">Timeline</div><div className="freq">● speed · accel · yaw</div></div>
        <div className="body"><div ref={el=>attach(el,'tl')} className="plot plot-tl"/></div>
      </div>
    </div>
  );
}

window.DetailView = DetailView;
