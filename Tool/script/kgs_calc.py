import numpy as np

# Dati per faccia: trimmed mean, chip frame
# tel_gsen4: Z e X (dT <= 0.4 C, temperatura ~43-44 C)
# tel_82:    Y (WiFi off, dT <= 0.7 C, temperatura ~38.6 C)
faces = {
    'Z_up':   dict(ax=+0.0312, ay=-0.0284, az=+1.0861, gx=+0.1510, gy=+0.8179, gz=-0.3643),
    'Z_down': dict(ax=-0.0234, ay=+0.0071, az=-0.9156, gx=-1.0612, gy=+0.6754, gz=-0.0547),
    'X_up':   dict(ax=+1.0119, ay=+0.0041, az=+0.0521, gx=-0.5126, gy=+0.0642, gz=-0.2239),
    'X_down': dict(ax=-0.9888, ay=-0.0022, az=+0.1358, gx=-2.1070, gy=-0.6438, gz=-0.1494),
    'Y_pos':  dict(ax=-0.0082, ay=+1.0310, az=+0.0090, gx=-0.4664, gy=+0.1678, gz=-0.0764),
    'Y_neg':  dict(ax=+0.0300, ay=-0.9710, az=+0.0310, gx=-0.1970, gy=+0.3496, gz=-0.0758),
}

pairs = [
    ('Z_up',  'Z_down', 'az', 'tel_gsen4'),
    ('X_up',  'X_down', 'ax', 'tel_gsen4'),
    ('Y_pos', 'Y_neg',  'ay', 'tel_82 WiFi-off'),
]
axes_gyro = ['gx', 'gy', 'gz']

Kgs = np.zeros((3, 3))  # Kgs[gyro_i][accel_j]  j: 0=az, 1=ax, 2=ay

SEP = '=' * 68
print(SEP)
print('  MATRICE K_gs [(deg/s)/g] -- G-Sensitivity MPU-6886 chip frame')
print(SEP)

for j, (pos_k, neg_k, accel_ax, src) in enumerate(pairs):
    p = faces[pos_k]
    n = faces[neg_k]
    dG = p[accel_ax] - n[accel_ax]
    print()
    print('  [%s] %s(%+.3fg) vs %s(%+.3fg)  dG=%.3f  src=%s' % (
        accel_ax, pos_k, p[accel_ax], neg_k, n[accel_ax], dG, src))
    print('  %-4s %9s %9s %10s' % ('gyro', 'val_+G', 'val_-G', 'K_gs'))
    for i, g in enumerate(axes_gyro):
        k = (p[g] - n[g]) / dG
        Kgs[i][j] = k
        flag = '  <-- ESKF heading' if g == 'gz' else ''
        print('  %-4s %+9.4f %+9.4f %+10.4f%s' % (g, p[g], n[g], k, flag))

print()
print(SEP)
print('  Matrice K_gs completa [gyro x accel]:')
print()
print('           %10s %10s %10s' % ('az(j=0)', 'ax(j=1)', 'ay(j=2)'))
for i, g in enumerate(axes_gyro):
    flag = '  <-- ESKF' if g == 'gz' else ''
    print('  %s(i=%d):  %+10.4f %+10.4f %+10.4f%s' % (
        g, i, Kgs[i][0], Kgs[i][1], Kgs[i][2], flag))

print()
print('  Effetto a regime (az=1g, ax~0, ay~0):')
print('    errore gz fisso = %+.4f deg/s  (K_gz_az x 1g)' % Kgs[2][0])
print('    errore gx fisso = %+.4f deg/s  (K_gx_az x 1g)' % Kgs[0][0])
print('    errore gy fisso = %+.4f deg/s  (K_gy_az x 1g)' % Kgs[1][0])
print()
print('  Correzione firmware (chip frame, prima di rotate_3d):')
print('    gx_clean -= (%+.4f)*az_cal + (%+.4f)*ax_cal + (%+.4f)*ay_cal' % (
    Kgs[0][0], Kgs[0][1], Kgs[0][2]))
print('    gy_clean -= (%+.4f)*az_cal + (%+.4f)*ax_cal + (%+.4f)*ay_cal' % (
    Kgs[1][0], Kgs[1][1], Kgs[1][2]))
print('    gz_clean -= (%+.4f)*az_cal + (%+.4f)*ax_cal + (%+.4f)*ay_cal' % (
    Kgs[2][0], Kgs[2][1], Kgs[2][2]))
print(SEP)
