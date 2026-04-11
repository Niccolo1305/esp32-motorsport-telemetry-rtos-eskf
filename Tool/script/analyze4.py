import pandas as pd
import numpy as np
from scipy.stats import trim_mean

df = pd.read_csv('tel_gsen4.csv', encoding='latin-1')
rename = {
    't_ms (ms)':'t_ms','raw_ax (G)':'raw_ax','raw_ay (G)':'raw_ay','raw_az (G)':'raw_az',
    'raw_gx (deg/s)':'raw_gx','raw_gy (deg/s)':'raw_gy','raw_gz (deg/s)':'raw_gz',
    'temp_c (degC)':'temp_c','lap':'lap'
}
# rename manuale per evitare problemi encoding
cols = df.columns.tolist()
new_cols = []
for c in cols:
    if 't_ms' in c: new_cols.append('t_ms')
    elif 'raw_ax' in c: new_cols.append('raw_ax')
    elif 'raw_ay' in c: new_cols.append('raw_ay')
    elif 'raw_az' in c: new_cols.append('raw_az')
    elif 'raw_gx' in c: new_cols.append('raw_gx')
    elif 'raw_gy' in c: new_cols.append('raw_gy')
    elif 'raw_gz' in c: new_cols.append('raw_gz')
    elif 'temp_c' in c: new_cols.append('temp_c')
    elif c == 'lap': new_cols.append('lap')
    else: new_cols.append(c)
df.columns = new_cols

df['t_s'] = df['t_ms']/1000.0
df['a_norm'] = np.sqrt(df.raw_ax**2+df.raw_ay**2+df.raw_az**2)

# Solo lap=1
df_lap = df[df['lap']==1].copy().reset_index(drop=True)

fs = 50.0
win = int(2*fs)
rs = df_lap['a_norm'].rolling(win, center=True, min_periods=1).std().fillna(0)
is_still = rs < 0.015

segs = []
in_seg = False
start = 0
for i, s in enumerate(is_still):
    if s and not in_seg:
        start = i
        in_seg = True
    elif not s and in_seg:
        segs.append((start, i-1))
        in_seg = False
if in_seg:
    segs.append((start, len(df_lap)-1))

segs.sort(key=lambda x: x[1]-x[0], reverse=True)
top6 = segs[:6]
top6.sort(key=lambda x: x[0])

TIME_TRIM = int(30*fs)
VTRIM = 0.05
labels = ['Z_up','Z_down','X_up','X_down','Y_up','Y_down']

sep = '='*95
print(sep)
print('  G-SENSITIVITY TEST (tel_gsen4) -- lap=1 only')
print(sep)

print('\nGIROSCOPIO')
print('  Faccia         N  |   gx_mean     s_gx |   gy_mean     s_gy |   gz_mean     s_gz')
print('                    |    (deg/s)  (deg/s) |    (deg/s)  (deg/s) |    (deg/s)  (deg/s)')
print('  ' + '-'*88)

results = []
for i, (i0, i1) in enumerate(top6):
    seg = df_lap.iloc[i0:i1+1]
    core = seg.iloc[TIME_TRIM:-TIME_TRIM] if len(seg) > 2*TIME_TRIM+10 else seg
    ax = trim_mean(core.raw_ax.values, VTRIM)
    ay = trim_mean(core.raw_ay.values, VTRIM)
    az = trim_mean(core.raw_az.values, VTRIM)
    gx = trim_mean(core.raw_gx.values, VTRIM)
    gy = trim_mean(core.raw_gy.values, VTRIM)
    gz = trim_mean(core.raw_gz.values, VTRIM)
    sgx = core.raw_gx.std()
    sgy = core.raw_gy.std()
    sgz = core.raw_gz.std()
    tc0 = core.temp_c.iloc[0]
    tc1 = core.temp_c.iloc[-1]
    flag = ' *' if i == 5 else ''
    lbl = labels[i] + flag
    results.append({'lbl':lbl,'ax':ax,'ay':ay,'az':az,
                    'gx':gx,'gy':gy,'gz':gz,
                    'sgx':sgx,'sgy':sgy,'sgz':sgz,
                    'tc0':tc0,'tc1':tc1,'n':len(core),
                    'i0':i0,'i1':i1})
    print(f'  {lbl:<12}  {len(core):>6}  | {gx:>+9.4f}  {sgx:>7.4f} | {gy:>+9.4f}  {sgy:>7.4f} | {gz:>+9.4f}  {sgz:>7.4f}')

print('\nACCELEROMETRO (conferma orientazione)')
print('  Faccia         |   ax_mean     s_ax |   ay_mean     s_ay |   az_mean     s_az')
print('  ' + '-'*75)
for i, r in enumerate(results):
    seg = df_lap.iloc[top6[i][0]:top6[i][1]+1]
    core = seg.iloc[TIME_TRIM:-TIME_TRIM] if len(seg) > 2*TIME_TRIM+10 else seg
    sax = core.raw_ax.std()
    say = core.raw_ay.std()
    saz = core.raw_az.std()
    print(f'  {r["lbl"]:<12}  | {r["ax"]:>+9.4f}  {sax:>7.4f} | {r["ay"]:>+9.4f}  {say:>7.4f} | {r["az"]:>+9.4f}  {saz:>7.4f}')

print('\nTEMPERATURA')
print('  Faccia         |  T_ini  T_fin     dT   Nota')
print('  ' + '-'*50)
for r in results:
    dt = r['tc1'] - r['tc0']
    if abs(dt) < 0.5: nota = 'ok'
    elif abs(dt) < 1.0: nota = '~ lieve'
    else: nota = 'DRIFT'
    print(f'  {r["lbl"]:<12}  | {r["tc0"]:>6.2f} {r["tc1"]:>6.2f}  {dt:>+5.2f}   {nota}')

print()
print('  * = ultima faccia (cavo USB/Grove)')
print(sep)
