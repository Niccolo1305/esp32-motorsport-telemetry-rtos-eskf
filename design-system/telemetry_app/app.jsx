/* app.jsx - main shell, landing, view switcher.
   No exports - renders to #root.
*/
const { useState: useS, useEffect: useE, useRef: useR, useCallback: useC } = React;

function Chrome({view, setView, fileName, onReset, onExport, theme, setTheme, hasData}){
  return (
    <div className="chrome">
      <div className="brand">
        <div className="mark">T</div>
        <div>
          <span className="wm">Telemetria</span>
          <span className="v">/ SESSION ANALYSER</span>
        </div>
      </div>
      <div className="nav">
        <button className={view==='overall'?'active':''} disabled={!hasData} onClick={()=>setView('overall')}>Overall</button>
        <button className={view==='detail'?'active':''} disabled={!hasData} onClick={()=>setView('detail')}>Detail</button>
      </div>
      <div className="tools">
        {fileName && <>
          <span className="file">[{fileName}]</span>
          <span className="sep">/</span>
        </>}
        <button className="btn ghost" onClick={()=>setTheme(theme==='dark'?'light':'dark')}
          title="Toggle theme">{theme==='dark'?'dark':'light'}</button>
        {hasData && <button className="btn ghost" onClick={onExport} title="Export PNG">PNG</button>}
        {hasData && <button className="btn" onClick={onReset}>New session</button>}
      </div>
    </div>
  );
}

function Landing({onFile, err, loading, loadingMsg, sessions, onLoadSession, serverMode}){
  const [drag, setDrag] = useS(false);
  const inputRef = useR(null);
  const onDrop = (e) => {
    e.preventDefault(); setDrag(false);
    const f = e.dataTransfer.files?.[0]; if(f) onFile(f);
  };
  return (
    <div className="landing">
      <div className={'dropzone '+(drag?'drag':'')}
        onDragOver={e=>{e.preventDefault(); setDrag(true);}}
        onDragLeave={()=>setDrag(false)}
        onDrop={onDrop}>
        <div className="stack">
          {loading ? (
            <div className="loading">
              <div className="msg">{loadingMsg || 'Parsing...'}</div>
              <div className="bar"></div>
            </div>
          ) : (
            <>
              <h1>Drop a session log</h1>
              <div className="sub">Telemetria / session analyser</div>
              <p>Drag a <b>.csv</b> or <b>.bin</b> file anywhere on this panel, or click below to pick one.
                 {serverMode ? ' The local server imports it once and reuses the binary cache.' : ' CSV files parse in-browser when the local server is not running.'}</p>
              <button className="btn primary" onClick={()=>inputRef.current.click()}>
                Select log file
              </button>
              <input ref={inputRef} type="file" accept=".csv,.bin,text/csv,application/octet-stream"
                onChange={e=>{ const f = e.target.files?.[0]; if(f) onFile(f); e.target.value=''; }}/>
              <div className="specs">
                <span>Expects <b>t_us</b>, <b>ax/ay/az</b>, <b>gx/gy/gz</b>, <b>gps_lat/lon</b>, <b>kf_vel</b>, <b>kf_x/y</b></span>
              </div>
            </>
          )}
        </div>
      </div>
      {serverMode && sessions && sessions.length > 0 && (
        <div className="sessions">
          {sessions.map(s => (
            <div className="session-row" key={s.id}>
              <div>
                <b>{s.source_name}</b>
                <div className="meta">
                  {Number(s.sample_count || 0).toLocaleString()} samples / {TelemetryCompute.fmtDur(s.duration_s || 0)} / {s.raw_source || 'raw n/a'}
                </div>
              </div>
              <button className="btn ghost" onClick={()=>onLoadSession(s)}>Load</button>
            </div>
          ))}
        </div>
      )}
      {err && <div className="err">{err}</div>}
    </div>
  );
}

function App(){
  const [data, setData] = useS(null);
  const [laps, setLaps] = useS(null);
  const [fileName, setFileName] = useS(null);
  const [view, setView] = useS('overall');
  const [err, setErr] = useS(null);
  const [loading, setLoading] = useS(false);
  const [loadingMsg, setLoadingMsg] = useS('');
  const [xAxis, setXAxis] = useS('time');
  const [showRaw, setShowRaw] = useS(false);
  const [mode3D, setMode3D] = useS(false);
  const [theme, setTheme] = useS('dark');
  const [hoverIdx, setHoverIdx] = useS(null);
  const [sessions, setSessions] = useS([]);
  const [serverMode, setServerMode] = useS(false);

  useE(()=>{
    document.body.classList.toggle('light', theme==='light');
    if(data) setData({...data});
  }, [theme]);

  const refreshSessions = useC(async () => {
    if(!window.TelemetryBinary) return;
    try {
      const list = await TelemetryBinary.listSessions();
      setSessions(list);
      setServerMode(true);
    } catch(_e) {
      setServerMode(false);
    }
  }, []);

  useE(()=>{ refreshSessions(); }, []);

  const loadBinarySession = useC(async (session) => {
    setErr(null); setLoading(true); setLoadingMsg('Loading cached session...');
    try {
      const parsed = await TelemetryBinary.loadSession(session.id, setLoadingMsg);
      setLoadingMsg('Detecting laps...');
      await new Promise(r=>setTimeout(r,10));
      const l = TelemetryCompute.detectLaps(parsed);
      setFileName(parsed.meta?.source_name || session.source_name || session.id);
      setData(parsed);
      setLaps(l);
      setView('overall');
    } catch(e) {
      console.error(e);
      setErr(e.message || String(e));
    } finally {
      setLoading(false); setLoadingMsg('');
    }
  }, []);

  const onFile = useC(async (file) => {
    setErr(null); setLoading(true); setLoadingMsg('Reading '+file.name+'...');
    try {
      let parsed;
      if(serverMode && window.TelemetryBinary){
        const session = await TelemetryBinary.importFile(file, setLoadingMsg);
        await refreshSessions();
        parsed = await TelemetryBinary.loadSession(session.id, setLoadingMsg);
      } else {
        if(!file.name.toLowerCase().endsWith('.csv')){
          throw new Error('Binary import requires the local server. Run: python -m Tool.telemetry_server');
        }
        parsed = await TelemetryCompute.parseCSV(file, (n)=>{
          setLoadingMsg('Parsing... '+n.toLocaleString()+' rows');
        });
      }
      setLoadingMsg('Detecting laps...');
      await new Promise(r=>setTimeout(r,10));
      const l = TelemetryCompute.detectLaps(parsed);
      setFileName(parsed.meta?.source_name || file.name);
      setData(parsed);
      setLaps(l);
      setView('overall');
    } catch(e){
      console.error(e);
      setErr(e.message || String(e));
    } finally {
      setLoading(false); setLoadingMsg('');
    }
  }, [serverMode, refreshSessions]);

  const onReset = () => { setData(null); setLaps(null); setFileName(null); setErr(null); refreshSessions(); };

  const onExport = () => {
    const target = document.querySelector('.plot-tl') || document.querySelector('.plot-md') || document.querySelector('.plot-sm');
    if(target && window.Plotly){
      Plotly.downloadImage(target, {format:'png', filename:'telemetria-export', width:1600, height:900, scale:2});
    }
  };

  const Ov = window.OverallView;
  const Dv = window.DetailView;

  return (
    <>
      <Chrome view={view} setView={setView} fileName={fileName}
        onReset={onReset} onExport={onExport}
        theme={theme} setTheme={setTheme}
        hasData={!!data}/>
      {!data ? (
        <Landing onFile={onFile} err={err} loading={loading} loadingMsg={loadingMsg}
          sessions={sessions} onLoadSession={loadBinarySession} serverMode={serverMode}/>
      ) : view==='overall' ? (
        <Ov d={data} laps={laps}
          xAxis={xAxis} setXAxis={setXAxis}
          showRaw={showRaw} setShowRaw={setShowRaw}
          mode3D={mode3D} setMode3D={setMode3D}
          onHoverIdx={setHoverIdx}/>
      ) : (
        <Dv d={data} laps={laps}
          xAxis={xAxis} setXAxis={setXAxis}
          showRaw={showRaw} setShowRaw={setShowRaw}
          mode3D={mode3D} setMode3D={setMode3D}
          onHoverIdx={setHoverIdx}/>
      )}
      <div className="foot">
        <span>Telemetria / local cache / browser UI</span>
        <span>dark-light / export png / keyboard [T]heme [N]ew</span>
      </div>
    </>
  );
}

function mount(){
  if(!window.Plotly || !window.Papa || !window.L || !window.OverallView || !window.DetailView){
    return setTimeout(mount, 30);
  }
  const root = ReactDOM.createRoot(document.getElementById('root'));
  root.render(<App/>);
}
mount();

window.addEventListener('keydown', (e)=>{
  if(e.target.tagName==='INPUT' || e.target.tagName==='TEXTAREA') return;
  if(e.key==='t' || e.key==='T'){ document.body.classList.toggle('light'); }
});
