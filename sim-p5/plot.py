#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""plot.py -- figures for the P5 delay/stability report (matplotlib only)."""
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict

# ---------------------------------------------------------------- load
res = [r for r in csv.reader(open('results.csv', newline='')) if r and r[0] == 'CSV']
aux = [r for r in csv.reader(open('aux.csv', newline='')) if r and r[0] == 'AUX']
rows = []
for r, a in zip(res, aux):
    rows.append(dict(mode=int(r[1]), depth=int(r[2]), gain=float(r[3]),
                     conv=int(r[5]), ts=int(r[6]), einf=float(r[7]),
                     pp=float(r[9]), per=float(r[10]), tag=int(a[13]),
                     sper=float(a[14]) if len(a) > 15 else 0.0))

tr = defaultdict(lambda: {'f': [], 'e': [], 'dr': [], 'd': []})
for r in csv.reader(open('trace.csv', newline='')):
    if r and r[0] == 'TRC':
        t = int(r[1])
        tr[t]['f'].append(int(r[2])); tr[t]['e'].append(float(r[3]))
        tr[t]['dr'].append(int(r[5])); tr[t]['d'].append(int(r[6]))

# average over repeats for the matrices
def cell(mode, gain, depth):
    v = [r for r in rows if r['mode'] == mode and abs(r['gain'] - gain) < 1e-9
         and r['depth'] == depth and not (r['tag'] >= 90000)]
    if not v:
        return None
    return dict(n=len(v), pp=sum(x['pp'] for x in v) / len(v),
                conv=sum(x['conv'] for x in v) / len(v),
                per=sum(x['per'] for x in v) / len(v))

# ---------------------------------------------------------------- fig 1: waveforms
fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
lab = {90000: 'tau=0 (fixed)', 90001: 'tau=2 (fixed)', 90002: 'tau=4 (fixed)',
       90003: 'tau=8 (fixed) -> limit cycle'}
for t, l in lab.items():
    ax[0].plot(tr[t]['f'], tr[t]['e'], lw=1.4, label=l)
ax[0].axhline(6, color='k', ls=':', lw=1)
ax[0].text(5, 8, 'deadzone 6 px', fontsize=8)
ax[0].set_xlabel('frame'); ax[0].set_ylabel('|e|  [px]')
ax[0].set_title('A: fixed delay'); ax[0].set_xlim(0, 200); ax[0].legend(fontsize=8)
lab2 = {90003: 'A tau=8 fixed  (pp=19 px, T=17)',
        90005: 'B jitter cap 8  (mean 4)', 90006: 'B jitter cap 14 (mean 7)'}
for t, l in lab2.items():
    ax[1].plot(tr[t]['f'], tr[t]['e'], lw=1.4, label=l)
ax[1].axhline(6, color='k', ls=':', lw=1)
ax[1].set_xlabel('frame'); ax[1].set_title('A vs B (same plant, same servo)')
ax[1].set_xlim(0, 200); ax[1].legend(fontsize=8)
fig.tight_layout(); fig.savefig('fig_wave.png', dpi=140); plt.close(fig)

# ---------------------------------------------------------------- fig 2: matrix
gains = [0.4, 0.8, 1.6, 3.2, 6.4]
depths = [0, 1, 2, 4, 8]
M = [[cell(1, g, d)['pp'] for d in depths] for g in gains]
fig, ax = plt.subplots(figsize=(6.2, 4.4))
im = ax.imshow(M, cmap='YlOrRd', aspect='auto', origin='lower')
ax.set_xticks(range(len(depths))); ax.set_xticklabels(depths)
ax.set_yticks(range(len(gains))); ax.set_yticklabels(['%.1f' % g for g in gains])
ax.set_xlabel('fixed delay tau  [frames]'); ax.set_ylabel('plant_gain  g')
ax.set_title('pp of |e| over the last third  [px]')
for i, g in enumerate(gains):
    for j, d in enumerate(depths):
        c = cell(1, g, d)
        txt = 'C' if (c['conv'] == 1 and c['pp'] <= 3) else ('O%.0f' % c['per'] if c['per'] > 0 else 'M')
        ax.text(j, i, '%.0f\n%s' % (c['pp'], txt), ha='center', va='center', fontsize=8)
fig.colorbar(im, ax=ax, label='pp [px]')
fig.tight_layout(); fig.savefig('fig_matrix.png', dpi=140); plt.close(fig)

# ---------------------------------------------------------------- fig 3: theory
osc = [r for r in rows if r['mode'] == 1 and r['pp'] > 6 and r['per'] > 0 and r['tag'] < 90000]
fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
bys = defaultdict(list)
for r in osc:
    bys[r['depth']].append(r)
xs = sorted(bys)
ax[0].plot(xs, [sum(x['sper'] for x in bys[d]) / len(bys[d]) for d in xs], 'o-',
           label='measured T (signed error)')
ax[0].plot(xs, [sum(x['per'] for x in bys[d]) / len(bys[d]) for d in xs], 's-',
           label='measured T (|e|, = half)')
ax[0].plot(xs, [4 * d + 2 for d in xs], 'k--', label='4*tau + 2')
ax[0].plot(xs, [2 * d + 1 for d in xs], 'k:', label='2*tau + 1')
ax[0].set_xlabel('delay tau  [frames]'); ax[0].set_ylabel('period  [frames]')
ax[0].set_title('T = 4*tau + 2 holds for the signed error'); ax[0].legend(fontsize=8)

K = 32.0 * 2 * 350 / 65536.0
uniq = {}
for r in rows:
    if r['tag'] >= 90000:
        continue
    uniq.setdefault((r['gain'], r['depth'], r['mode']), []).append(r)
for (g, d, m), v in sorted(uniq.items()):
    pp = sum(x['pp'] for x in v) / len(v)
    kt = K * g * d
    cv = sum(x['conv'] for x in v) / len(v)
    col = 'tab:green' if (pp <= 3 and cv >= 0.99) else ('tab:red' if pp > 6 else 'tab:orange')
    ax[1].scatter(kt, max(pp, 0.3), color=col, s=36)
ax[1].axvline(3.14159 / 2, color='k', ls='--')
ax[1].text(1.62, 120, 'pi/2', fontsize=9)
ax[1].set_xlabel('K_eff * tau   (K_eff = 0.342*g)')
ax[1].set_ylabel('pp  [px]')
ax[1].set_title('stability boundary (green=settled, red=limit cycle)')
ax[1].set_yscale('log')
fig.tight_layout(); fig.savefig('fig_theory.png', dpi=140); plt.close(fig)
print('wrote fig_wave.png, fig_matrix.png, fig_theory.png')
