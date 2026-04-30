/* session_binary.js - local-server Float32 session loader.
   Exports: window.TelemetryBinary
*/
(function(){
  'use strict';

  const API = '/api';

  async function listSessions(){
    const res = await fetch(`${API}/sessions`, {cache:'no-store'});
    if(!res.ok) throw new Error('Telemetry server is not available.');
    const payload = await res.json();
    return payload.sessions || [];
  }

  async function importFile(file, onProgress){
    if(onProgress) onProgress('Uploading '+file.name+'...');
    const form = new FormData();
    form.append('file', file, file.name);
    const res = await fetch(`${API}/sessions/import`, {method:'POST', body:form});
    if(!res.ok){
      let detail = await res.text();
      try { detail = (JSON.parse(detail).detail || detail); } catch(_e) {}
      throw new Error(detail || 'Import failed.');
    }
    const payload = await res.json();
    return payload.session;
  }

  async function fetchJson(url){
    const res = await fetch(url, {cache:'no-store'});
    if(!res.ok) throw new Error('Failed to load '+url);
    return await res.json();
  }

  async function fetchF32(url){
    const res = await fetch(url, {cache:'no-store'});
    if(!res.ok) throw new Error('Failed to load '+url);
    return new Float32Array(await res.arrayBuffer());
  }

  function splitChannels(flat, channels, sampleCount){
    const out = {};
    channels.forEach((name, i) => {
      const start = i * sampleCount;
      out[name] = flat.subarray(start, start + sampleCount);
    });
    return out;
  }

  function lowerBound(arr, value){
    let lo = 0, hi = arr.length;
    while(lo < hi){
      const mid = (lo + hi) >> 1;
      if(arr[mid] < value) lo = mid + 1;
      else hi = mid;
    }
    return lo;
  }

  function windowedArrays(pool, channels, xName, start, end){
    const x = pool[xName];
    let i0 = 0, i1 = x.length;
    if(Number.isFinite(start)) i0 = lowerBound(x, start);
    if(Number.isFinite(end)) i1 = Math.min(x.length, lowerBound(x, end) + 1);
    if(i1 < i0) i1 = i0;

    const out = {idx: pool.idx.subarray(i0, i1), x: x.subarray(i0, i1)};
    channels.forEach(name => {
      out[name] = (pool[name] || pool.x || x).subarray(i0, i1);
    });
    return out;
  }

  function makeSession(meta, rawFlat, levelFlats){
    const channels = meta.channels || [];
    const raw = splitChannels(rawFlat, channels, meta.sample_count);
    const levels = {};
    const levelList = (meta.levels || []).map(info => {
      const flat = levelFlats[info.level];
      const arrays = splitChannels(flat, channels, info.sample_count);
      levels[info.level] = {info, arrays, flat};
      return levels[info.level];
    }).sort((a,b) => b.info.sample_count - a.info.sample_count);

    const data = {
      meta,
      raw,
      levels,
      _binary: true,
      _buffers: {rawFlat, levelFlats},
      _N: meta.sample_count,
      _duration_s: meta.duration_s,
      _hasRaw: meta.raw_source && meta.raw_source !== 'none',
      getSeries({channels: requested, xAxis='time', start=null, end=null, maxPoints=4000}){
        const xName = xAxis === 'dist' ? 'distance_m' : 't_s';
        let chosen = {info:{sample_count: meta.sample_count, level:'raw'}, arrays: raw};
        for(const level of levelList){
          if(level.info.sample_count <= maxPoints * 1.25){
            chosen = level;
            break;
          }
        }
        if(chosen.info.sample_count > maxPoints * 1.25 && levelList.length){
          chosen = levelList[levelList.length - 1];
        }
        return windowedArrays(chosen.arrays, requested, xName, start, end);
      },
      getRawWindow({xAxis='time', start=null, end=null} = {}){
        const xName = xAxis === 'dist' ? 'distance_m' : 't_s';
        return windowedArrays(raw, channels, xName, start, end);
      },
    };

    channels.forEach(name => { data[name] = raw[name]; });
    if(!data.kf_vel && data.vel_kmh){
      const kfVel = new Float32Array(data.vel_kmh.length);
      for(let i=0; i<kfVel.length; i++) kfVel[i] = data.vel_kmh[i] / 3.6;
      data.kf_vel = kfVel;
    }
    return data;
  }

  async function loadSession(id, onProgress){
    if(onProgress) onProgress('Loading metadata...');
    const meta = await fetchJson(`${API}/sessions/${id}/meta`);
    if(onProgress) onProgress('Loading raw cache...');
    const rawFlat = await fetchF32(`${API}/sessions/${id}/raw.f32`);
    const levelFlats = {};
    for(const info of meta.levels || []){
      if(onProgress) onProgress('Loading '+info.level+'...');
      levelFlats[info.level] = await fetchF32(`${API}/sessions/${id}/levels/${info.level}.f32`);
    }
    return makeSession(meta, rawFlat, levelFlats);
  }

  window.TelemetryBinary = {
    listSessions,
    importFile,
    loadSession,
  };
})();
