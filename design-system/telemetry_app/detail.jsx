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
  const mapPtsRef = useRefD([]);
  const mapHoverPtsRef = useRefD([]);
  const [mapHover, setMapHover] = useStateD(null);

  const syncEls = () => [plotRefs.current.yaw, plotRefs.current.tl].filter(Boolean);
  const clearCursorSync = () => TelemetryCharts.clearCursorLines(syncEls());
  const applyRawIndex = (idx, p=null, skipEl=null) => {
    if(idx == null) return;
    if(onHoverIdx) onHoverIdx(idx);
    const lat = p ? p.lat : (d.gps_lat[idx]||d.kf_lat[idx]);
    const lon = p ? p.lon : (d.gps_lon[idx]||d.kf_lon[idx]);
    if(mapCurRef.current && lat && lon) mapCurRef.current.setLatLng([lat,lon]);
    const xVal = TelemetryCharts.xValueForIndex(d, idx, xAxis);
    TelemetryCharts.setCursorLines(syncEls(), xVal, skipEl);
  };
  const clearMapHover = () => {
    setMapHover(null);
    clearCursorSync();
  };
  const handleMapHover = (ev) => {
    if(ev.target && ev.target.closest && ev.target.closest('.leaflet-control')){
      clearMapHover();
      return;
    }
    const map = mapInstRef.current;
    const pts = mapHoverPtsRef.current.length ? mapHoverPtsRef.current : mapPtsRef.current;
    if(!map || !pts || !pts.length) return;
    const nativeEvent = ev.nativeEvent || ev;
    const p = TelemetryCharts.nearestMapPoint(map, pts, map.mouseEventToLatLng(nativeEvent));
    if(!p) return;
    applyRawIndex(p.idx, p);

    const rect = ev.currentTarget.getBoundingClientRect();
    const x = Math.max(10, Math.min(rect.width - 140, ev.clientX - rect.left + 14));
    const y = Math.max(10, Math.min(rect.height - 72, ev.clientY - rect.top - 72));
    setMapHover(prev => (
      prev && prev.idx === p.idx && Math.abs(prev.x - x) < 2 && Math.abs(prev.y - y) < 2
        ? prev
        : {idx:p.idx, x, y}
    ));
  };

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
      const pts = TelemetryCharts.mapPoints(d, TelemetryCharts.mapDrawMaxPoints(d), 'kf');
      const hoverPts = TelemetryCharts.mapPoints(d, TelemetryCharts.mapHoverMaxPoints(d), 'kf');
      mapPtsRef.current = pts;
      mapHoverPtsRef.current = hoverPts.length ? hoverPts : pts;
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

    const syncElsNow = syncEls();
    const handler = (ev)=>{
      if(!ev||!ev.points||!ev.points.length) return;
      const idx = TelemetryCharts.eventRawIndex(ev, d, xAxis);
      applyRawIndex(idx, null, ev.event?.target?.closest('.plot'));
    };
    syncElsNow.forEach(el => { if(el){ el.on('plotly_hover', handler);
      el.on('plotly_unhover', clearCursorSync); }});
    return ()=>{
      syncElsNow.forEach(el => {
        if(el && el.removeAllListeners){
          el.removeAllListeners('plotly_hover');
          el.removeAllListeners('plotly_unhover');
        }
      });
    };
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
          <div className="detail-map-host"
            onMouseMoveCapture={handleMapHover}
            onPointerMoveCapture={handleMapHover}
            onMouseLeaveCapture={clearMapHover}
            onPointerLeaveCapture={clearMapHover}>
            <div ref={mapRef} style={{height:'100%', background:'#000'}}></div>
            {mapHover && (
              <div className="map-hover-float"
                style={{left:mapHover.x, top:mapHover.y}}
                dangerouslySetInnerHTML={{__html: TelemetryCharts.mapPointHtml(d, mapHover.idx)}} />
            )}
          </div>
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
