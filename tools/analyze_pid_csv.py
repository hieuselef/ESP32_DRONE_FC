#!/usr/bin/env python3
"""Phân tích log CSV từ Serial Monitor bản v2, chỉ là công cụ DIAGNOSTIC."""
import csv
import math
import statistics
import sys
from pathlib import Path

EXPECTED = ('ms', 'roll_rate_target', 'roll_rate_filtered', 'pitch_rate_target',
            'pitch_rate_filtered', 'yaw_rate_target', 'yaw_rate_filtered',
            'clipped', 'dt_ms', 'loop_faults', 'pwm')

def parse(path):
    lines = Path(path).read_text(encoding='utf-8-sig', errors='replace').splitlines()
    # Serial Monitor in them banner va dong debug; chi lay cac dong sau header CSV.
    header = next((ln for ln in lines if ln.startswith('ms,rc,arm,bench,pwm,')), None)
    if not header:
        raise ValueError('Khong thay CSV header. Hay bat ENABLE_PID_CSV_LOG=true.')
    fields = next(csv.reader([header]))
    missing = set(EXPECTED).difference(fields)
    if missing:
        raise ValueError('CSV thieu truong: ' + ', '.join(sorted(missing)))
    rows = []
    for ln in lines:
        if not ln or not ln[0].isdigit():
            continue
        vals = next(csv.reader([ln]))
        if len(vals) != len(fields):
            continue
        try:
            rows.append({k:float(v) for k,v in zip(fields,vals)})
        except ValueError:
            continue
    if not rows:
        raise ValueError('Khong co dong du lieu hop le.')
    return rows

def pct95(values):
    v = sorted(values)
    return v[min(len(v)-1, math.ceil(0.95*len(v))-1)]

def summarize(rows):
    print('So dong log:',len(rows))
    print('PWM ra ESC:', 'ON' if any(r['pwm'] >= 1 for r in rows) else 'OFF (BENCH)')
    print('Thoi gian dt_ms trung binh / P95:',
          round(statistics.mean(r['dt_ms'] for r in rows),3), '/',
          round(pct95([r['dt_ms'] for r in rows]),3))
    print('So su kien loi dt (thay doi dem):',
          max(0,int(rows[-1]['loop_faults']-rows[0]['loop_faults'])))
    print('Ty le clipped: %.1f%%' % (100*sum(r['clipped'] for r in rows)/len(rows)))
    for axis in ('roll','pitch','yaw'):
        a=[abs(r[f'{axis}_rate_target']-r[f'{axis}_rate_filtered']) for r in rows]
        print('%s: |setpoint - gyro_filtered| mean=%.2f deg/s max=%.2f deg/s' %
              (axis, statistics.mean(a), max(a)))
    if not any(r['pwm'] >= 1 for r in rows):
        print('CANH BAO: day la so lieu BENCH gia lap, KHONG phai sai so bay thuc te.')

if __name__=='__main__':
    if len(sys.argv)!=2:
        sys.exit('Dung: python tools/analyze_pid_csv.py serial_log.txt')
    try:
        summarize(parse(sys.argv[1]))
    except (ValueError,OSError) as e:
        sys.exit('Loi: '+str(e))
