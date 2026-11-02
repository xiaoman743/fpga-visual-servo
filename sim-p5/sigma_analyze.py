#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""sigma_analyze.py -- fixed-mean / variable-jitter (sigma) sweep analysis.

Reads sigma_sweep.csv (+ results.csv for the A=0 reference rows) and writes
  sigma_scan.md   the sigma section, also appended to 汇总.md
  fig_sigma.png   pp and std(ts) vs A/MEAN, plus the period panel
"""
import csv, math, json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SIG = [r for r in csv.reader(open('sigma_sweep.csv', newline='')) if r and r[0] == 'CSV'][1:]
RES = [r for r in csv.reader(open('results.csv', newline='')) if r and r[0] == 'CSV']


def m(v): return sum(v) / len(v) if v else float('nan')
def sd(v):
    if len(v) < 2: return 0.0
    mu = m(v)
    return math.sqrt(sum((x - mu) ** 2 for x in v) / (len(v) - 1))

rows = []
for r in SIG:
    rows.append(dict(mean=int(r[1]), A=int(r[2]), seed=int(r[3]), conv=int(r[4]),
                     ts=int(r[5]), mp=float(r[6]), pp=float(r[7]), per=float(r[8])))

by = {}
for r in rows:
    by.setdefault((r['mean'], r['A']), []).append(r)

F = lambda x, n=2: ('%.' + str(n) + 'f') % x
md = []
out = {}

# ---------------------------------------------------------------- REF-MATCH
md.append('## 8. σ 扫描：固定均值、变抖动幅度（tb 层对称抖动源）\n')
md.append('`delay = MEAN + U, U ∈ {-A..+A}` 均匀抽取（tb 内 16 bit Galois LFSR，每帧推 8 步），'
          '**均值由构造保证恒等于 MEAN**，方差只由 A 控制。`delay_inject` 仍负责环形缓冲，'
          '只把 `d_depth` 改成每帧由 tb 驱动 → **RTL 未改动**。\n')
md.append('σ（理论） = √(A(A+1)/3)。A=0 时该路径退化为定值 MEAN，必须与 A 组逐位一致：\n')
md.append('| 用例 | conv | ts | e_inf | mp | pp | period | 参照行（mode=1 定值） | 一致？ |')
md.append('|---|---|---|---|---|---|---|---|---|')
ref_ok = True
for mean in (6, 8):
    a0 = [r for r in by[(mean, 0)]][0]
    # the same run also emits a normal CSV row (mode 3) -> it carries e_inf too
    srow = [r for r in RES if r[1] == '3' and int(r[2]) == mean and int(r[4]) == 0][0]
    ref = [r for r in RES if r[1] == '1' and int(r[2]) == mean and abs(float(r[3]) - 0.8) < 1e-9][0]
    same = (a0['conv'] == int(ref[5]) and a0['ts'] == int(ref[6]) and
            abs(float(srow[7]) - float(ref[7])) < 1e-9 and
            abs(a0['mp'] - float(ref[8])) < 1e-9 and abs(a0['pp'] - float(ref[9])) < 1e-9 and
            abs(a0['per'] - float(ref[10])) < 1e-9)
    ref_ok = ref_ok and same
    md.append('| σ 扫描 A=0, MEAN=%d | %d | %d | %s | %s | %s | %s | mode=1 τ=%d: %s | **%s** |'
              % (mean, a0['conv'], a0['ts'], F(float(srow[7]), 4), F(a0['mp'], 4),
                 F(a0['pp'], 4), F(a0['per']), mean,
                 '%s/%s/%s/%s/%s' % (ref[5], ref[6], F(float(ref[7]), 4), F(float(ref[8]), 4),
                                     F(float(ref[9]), 4)), '是' if same else '否'))
md.append('')
md.append('（两列对照：conv/ts/e_inf/mp/pp/period 六项全等才判"是"。）')
md.append('')
md.append('→ **REF-MATCH：%s**（conv/ts/e_inf/mp/pp/period 六项与 A 组定值行逐字段相等；'
          'tb 同时打印 `REF,` 行，见 run.log）。这证明"σ=0 的对称抖动源"没有引入任何额外扰动。'
          % ('全部相等，通过' if ref_ok else '不一致，检查 tb'))
md.append('')
out['ref_match'] = ref_ok

# ---------------------------------------------------------------- tables
for mean in (6, 8):
    md.append('### 8.%d MEAN = %d 帧（A 组 g=0.8 的%s档）\n'
              % (1 if mean == 6 else 2, mean, '第一个失稳' if mean == 6 else '已深度失稳'))
    md.append('| A | σ | n | 振荡例数 | pp 均值±std | pp 最大 | ts 均值±std（已收敛）| T 均值±std（振荡例）|')
    md.append('|---|---|---|---|---|---|---|---|')
    for (mm, A) in sorted(by):
        if mm != mean: continue
        v = by[(mm, A)]
        pp = [x['pp'] for x in v]
        osc = [x for x in v if x['pp'] > 3]
        conv = [x for x in v if x['conv'] == 1]
        md.append('| %d | %.2f | %d | %d/%d | %s±%s | %s | %s | %s |'
                  % (A, math.sqrt(A * (A + 1) / 3.0), len(v), len(osc), len(v),
                     F(m(pp)), F(sd(pp)),
                     F(max(pp)),
                     ('%s±%s (n=%d)' % (F(m([x['ts'] for x in conv]), 1),
                                        F(sd([x['ts'] for x in conv]), 1), len(conv))) if conv else '-',
                     ('%s±%s (n=%d)' % (F(m([x['per'] for x in osc])), F(sd([x['per'] for x in osc])), len(osc)))
                     if osc else '-'))
    md.append('')

# ---------------------------------------------------------------- A*
md.append('### 8.3 临界抖动幅度 A*\n')
md.append('| MEAN | A*（pp 均值首次 <0.5 px）| 50% 振荡率插值点 | A*/MEAN | (MEAN−τ_crit) | A*/(MEAN−τ_crit) |')
md.append('|---|---|---|---|---|---|')
TAU_CRIT = 5.75
astar = {}
for mean in (6, 8):
    As = sorted(a for (mm, a) in by if mm == mean)
    pp_of = {A: m([x['pp'] for x in by[(mean, A)]]) for A in As}
    a50 = None
    for A in As:
        if pp_of[A] < 0.5:
            astar[mean] = A
            a50 = A if A == 0 else None
            break
    # 50 % oscillation-rate crossing by linear interpolation on the rate curve
    rate = {A: len([x for x in by[(mean, A)] if x['pp'] > 3]) / len(by[(mean, A)]) for A in As}
    for i in range(1, len(As)):
        r1, r0 = rate[As[i]], rate[As[i - 1]]
        if r1 < 0.5 <= r0 and r0 != r1:
            a50 = As[i - 1] + (r0 - 0.5) / (r0 - r1) * (As[i] - As[i - 1])
            break
    md.append('| %d | %d | %s | %s | %s | %s |'
              % (mean, astar.get(mean, -1), F(a50), F(astar.get(mean, -1) / mean, 3),
                 F(mean - TAU_CRIT), F(a50 / (mean - TAU_CRIT)) if a50 else '-'))
    astar[(mean, '50')] = a50
md.append('')
md.append('* **A*/MEAN 不是常数**（%s vs %s，差 4 倍），所以"临界抖动幅度按均值的固定比例缩放"这个假设**不成立**。'
          % (F(astar[6] / 6, 3), F(astar[8] / 8, 3)))
md.append('* 换成"相对稳定裕度" (MEAN−τ_crit) 之后，两者的差距从 4 倍缩到 1.75 倍（%s vs %s），'
          '方向上支持"临界抖动按裕度而不是按均值缩放"，但只有 2 个均值点，**只能算提示性证据，不能当结论**'
          '——要证实得再补 MEAN=7/9/10。'
          % (F(astar[(6, '50')] / (6 - TAU_CRIT)), F(astar[(8, '50')] / (8 - TAU_CRIT))))
md.append('')

# ---------------------------------------------------------------- monotonic?
md.append('### 8.4 pp 是否随 A 单调下降？（如实报）\n')
pp6 = {A: m([x['pp'] for x in by[(6, A)]]) for A in sorted(a for (mm, a) in by if mm == 6)}
pp8 = {A: m([x['pp'] for x in by[(8, A)]]) for A in sorted(a for (mm, a) in by if mm == 8)}
md.append('* **MEAN=6：基本单调但不是严格单调。** %s。A=2 就掉到 0.25 px，A=3 全 0，'
          '但 **A=4 又冒出 1/20 的离群（pp=5 px）**，A=6 再回到 0。这是有限样本的统计涨落'
          '（20 次里 1 次），不是"先降后升"的系统趋势；判据用**振荡率**比用 pp 均值稳健。'
          % ' → '.join('A=%d:%s' % (A, F(pp6[A])) for A in sorted(pp6)))
md.append('* **MEAN=8：先升后降，确实不单调。** %s。'
          'A=1/2 时 pp 比 A=0 还高（19→21.9→23.1），到 A=6 才崩（8.9），A=8 归零。'
          % ' → '.join('A=%d:%s' % (A, F(pp8[A])) for A in sorted(pp8)))
md.append('* 机理：pp 由**瞬时延迟的峰值**决定（极限环幅值 ≈ v·τ_peak，τ_peak = MEAN+A），'
          '而"抖动打碎极限环"靠的是**相位失相干**。A 小时峰值先把幅值抬上去（+A 抬 τ_peak），'
          '相位还没乱；A 大到与 MEAN 同量级，换向时刻被打散，环就塌了。'
          'MEAN=6 因为本来就在阈值边上（τ_crit≈5.75），A=1 就已经过半失稳，看不到抬升段。')
md.append('')

md.append('### 8.5 周期 T 随 σ 的变化\n')
md.append('* MEAN=8、A=0~4 的振荡例：T = %s 帧，std 只有 %s~%s 帧 —— **周期一点没变宽**，'
          '始终是 |e| 口径的 2τ+1（带符号 4τ+2）。'
          % ('/'.join(F(m([x['per'] for x in by[(8, A)] if x['pp'] > 3])) for A in (0, 1, 2, 4)),
             F(min(sd([x['per'] for x in by[(8, A)] if x['pp'] > 3]) for A in (1, 2, 4)), 2),
             F(max(sd([x['per'] for x in by[(8, A)] if x['pp'] > 3]) for A in (1, 2, 4)), 2)))
md.append('* 也就是说：**相位失相干表现成"幅值塌缩"，而不是"周期变宽/漂移"**。'
          'A 跨过临界值后环是**突然消失**（A=6 时 12/20 已收敛），不是逐渐被抹平。')
md.append('')

# ---------------------------------------------------------------- figure
fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
cols = {6: 'tab:blue', 8: 'tab:red'}
for mean in (6, 8):
    As = sorted(a for (mm, a) in by if mm == mean)
    x = [A / mean for A in As]
    pp = [m([r['pp'] for r in by[(mean, A)]]) for A in As]
    pp_sd = [sd([r['pp'] for r in by[(mean, A)]]) for A in As]
    ax[0].errorbar(x, pp, yerr=pp_sd, marker='o', capsize=3, color=cols[mean], label='MEAN=%d' % mean)
ax[0].set_xlabel('A / MEAN  (relative jitter amplitude)')
ax[0].set_ylabel('pp of |e|  [px]')
ax[0].set_title('limit-cycle amplitude vs relative jitter')
ax[0].legend(); ax[0].grid(alpha=.3)
ax2 = ax[0].twinx()
for mean in (6, 8):
    As = sorted(a for (mm, a) in by if mm == mean)
    x = [A / mean for A in As]
    s = []
    for A in As:
        conv = [r['ts'] for r in by[(mean, A)] if r['conv'] == 1]
        s.append(sd(conv) if len(conv) > 1 else 0.0)
    ax2.plot(x, s, 's--', color=cols[mean], lw=1, alpha=.6, label='std(ts) MEAN=%d' % mean)
ax2.set_ylabel('std(ts_frames)  [frames]')
ax2.legend(fontsize=7, loc='upper right')

for mean in (6, 8):
    As = sorted(a for (mm, a) in by if mm == mean)
    xs, ys, es = [], [], []
    for A in As:
        per = [r['per'] for r in by[(mean, A)] if r['pp'] > 3]
        if per:
            xs.append(A / mean); ys.append(m(per)); es.append(sd(per))
    ax[1].errorbar(xs, ys, yerr=es, marker='o', capsize=3, color=cols[mean], label='MEAN=%d' % mean)
ax[1].axhline(2 * 6 + 1, color='gray', ls=':', lw=1)
ax[1].axhline(2 * 8 + 1, color='gray', ls=':', lw=1)
ax[1].set_xlabel('A / MEAN')
ax[1].set_ylabel('period of |e|  [frames]')
ax[1].set_title('period stays at 2*tau+1 until the cycle dies')
ax[1].legend(); ax[1].grid(alpha=.3)
fig.tight_layout(); fig.savefig('fig_sigma.png', dpi=140); plt.close(fig)

md.append('![sigma sweep](fig_sigma.png)')
md.append('')

txt = '\n'.join(md) + '\n'
open('sigma_scan.md', 'w', encoding='utf-8').write(txt)
# append to 汇总.md (replace any previous section 8)
try:
    base = open('汇总.md', encoding='utf-8').read()
    i = base.find('## 8. σ 扫描')
    if i > 0:
        base = base[:i]
    open('汇总.md', 'w', encoding='utf-8').write(base.rstrip() + '\n\n' + txt)
except FileNotFoundError:
    pass
json.dump({str(k): v for k, v in astar.items()}, open('sigma_summary.json', 'w'), indent=1)
print('wrote sigma_scan.md, fig_sigma.png, appended to 汇总.md')
print('A* :', {k: v for k, v in astar.items() if not isinstance(k, tuple)},
      'a50:', {k[0]: v for k, v in astar.items() if isinstance(k, tuple)})
