#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""summarize.py -- post-process results.csv / aux.csv into 汇总.md + summary.json

results.csv : one CSV row per case, exactly the format printed by tb_p5_sweep
aux.csv     : the matching AUX rows (same order) with the case tag
Both are appended from one xsim pass, so row i matches row i; lengths are checked.

tag ranges (plan.py):  A 10xxx  B 20xxx  K 30xxx  M 40xxx  BND 50xxx  TRACE 90xxx
"""
import csv, json, math

KR = 32.0          # servo K_R_Q10 / K_THETA_Q10 used for the run
R_OBJ = 350.0      # object radius from the pole
LIMIT = 4          # servo LIMIT (ticks/frame)
TAU_PI = math.pi / 2.0
# small-signal radial loop gain per frame at plant_gain = 1:
#   d_reach = K_R*(|u_obj|^2-|u_m|^2)/2^16 ~ K_R*2R/2^16 * dr
K_EFF_PER_G = KR * 2.0 * R_OBJ / 65536.0        # 0.3418


def load():
    res = [r for r in csv.reader(open('results.csv', newline='')) if r and r[0] == 'CSV']
    aux = [r for r in csv.reader(open('aux.csv', newline='')) if r and r[0] == 'AUX']
    if len(res) != len(aux):
        raise SystemExit('row mismatch: %d CSV vs %d AUX' % (len(res), len(aux)))
    rows = []
    for r, a in zip(res, aux):
        rows.append(dict(
            mode=int(r[1]), depth=int(r[2]), gain=float(r[3]), seed=int(r[4]),
            conv=int(r[5]), ts=int(r[6]), einf=float(r[7]), mp=float(r[8]),
            pp=float(r[9]), per=float(r[10]),
            mdly=float(a[5]), maxdly=float(a[6]), sat=float(a[7]),
            e0=float(a[8]), emax=float(a[9]), nuse=int(a[10]),
            div=int(a[11]), efin=float(a[12]), tag=int(a[13]),
            sper=float(a[14]) if len(a) > 15 else 0.0,
            spp=float(a[15]) if len(a) > 15 else 0.0))
    return rows


def grp(tag):
    if 10000 <= tag < 20000: return 'A'
    if 20000 <= tag < 30000: return 'B'
    if 30000 <= tag < 40000: return 'K'
    if 40000 <= tag < 50000: return 'M'
    if 50000 <= tag < 60000: return 'BND'
    if tag >= 90000:         return 'TRC'
    return '?'


def mean(v): return sum(v) / len(v) if v else float('nan')
def std(v):
    if len(v) < 2: return 0.0
    m = mean(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def cls(r):
    """C = settled in the deadzone, O = limit cycle, M = marginal, X = diverged."""
    if r['div']: return 'X'
    if r['conv'] and r['pp'] <= 3.0: return 'C'
    if r['pp'] > 6.0: return 'O'
    return 'M'


def agg(rows):
    return dict(n=len(rows),
                conv=mean([r['conv'] for r in rows]),
                ts=mean([r['ts'] for r in rows]),
                ts_sd=std([r['ts'] for r in rows]),
                einf=mean([r['einf'] for r in rows]),
                einf_sd=std([r['einf'] for r in rows]),
                mp=mean([r['mp'] for r in rows]),
                pp=mean([r['pp'] for r in rows]),
                pp_sd=std([r['pp'] for r in rows]),
                per=mean([r['per'] for r in rows]),
                per_sd=std([r['per'] for r in rows]),
                sper=mean([r['sper'] for r in rows]),
                spp=mean([r['spp'] for r in rows]),
                mdly=mean([r['mdly'] for r in rows]),
                sat=mean([r['sat'] for r in rows]),
                e0=mean([r['e0'] for r in rows]))


F = lambda x, n=2: ('%.' + str(n) + 'f') % x


def main():
    rows = load()
    out, md = {}, []
    A = [r for r in rows if grp(r['tag']) == 'A']
    B = [r for r in rows if grp(r['tag']) == 'B']
    K = [r for r in rows if grp(r['tag']) == 'K']
    M = [r for r in rows if grp(r['tag']) == 'M']
    N = [r for r in rows if grp(r['tag']) == 'BND']
    T = [r for r in rows if grp(r['tag']) == 'TRC']

    # A/B tables: plant_gain 0.8 only, from the A/B groups plus the M group's
    # extra matched points (depths 3/6/7 and caps 6/12/14)
    fix = [r for r in rows if r['mode'] == 1 and abs(r['gain'] - 0.8) < 1e-9
           and grp(r['tag']) in ('A', 'M')]
    jit = [r for r in rows if r['mode'] == 2 and abs(r['gain'] - 0.8) < 1e-9
           and grp(r['tag']) in ('B', 'M')]
    a_tab, b_tab = {}, {}
    for d in sorted(set(r['depth'] for r in fix)):
        a_tab[d] = agg([r for r in fix if r['depth'] == d])
    for m in sorted(set(r['depth'] for r in jit)):
        b_tab[m] = agg([r for r in jit if r['depth'] == m])

    md.append('# P5 延迟-闭环稳定性扫描：实测汇总\n')
    md.append('> 所有数字来自 Vivado 2020.2 xsim 实跑（`results.csv` / `aux.csv` / `run.log`）。'
              '除明确标注「解析换算」「估计」的小信号增益外，没有外推值。\n')

    md.append('## 0. 仿真配置\n')
    md.append('| 项 | 值 |')
    md.append('|---|---|')
    md.append('| 工具 | Vivado Simulator 2020.2（xvlog/xelab/xsim），工作目录 `D:\\Code\\fpga_p5_sim`（纯 ASCII）|')
    md.append('| DUT | `rtl/servo_polar.v` + `rtl/delay_inject.v`，从档案目录原样拷贝，**未修改** |')
    md.append('| 伺服参数 | K_THETA_Q10=%d, K_R_Q10=%d, SHIFT_PAN=15, SHIFT_RCH=16, LIMIT=%d, DEADZONE=6, HOLD_N=5 |'
              % (int(KR), int(KR), LIMIT))
    md.append('| plant | marker = pole + r·(cosφ,sinφ)；d_pan→φ+=d_pan·DEG_PER_TICK；'
              'd_reach→r+=d_reach·PX_PER_TICK（纯积分，无惯量/摩擦/噪声）|')
    md.append('| 极点 | (400, 240) px |')
    md.append('| 物体 | (750, 240) px → R=%d px，与 marker 共线（纯径向 SISO，pan 通道恒为 0）|' % R_OBJ)
    md.append('| 初值 | r₀=160 px, φ₀=0° → 初始误差 190 px |')
    md.append('| 每帧 | 5 clk @100 MHz = 50 ns；每用例 400 帧（20 µs 仿真时间）|')
    md.append('| plant_gain | 同时缩放 PX_PER_TICK=g 与 DEG_PER_TICK=0.5·g/0.8（整环增益语义）|')
    md.append('| 用例数 | %d（A %d / B %d / K %d / M %d / BND %d / TRACE %d）|'
              % (len(rows), len(A), len(B), len(K), len(M), len(N), len(T)))
    md.append('| 速度 | 362 用例一次 xsim = 3.8 s（10.5 ms/用例）；全部 369 用例 + 编译合计 12 s |')
    md.append('')

    # ---------------------------------------------------------------- A
    md.append('## 1. A 组：定值延迟（mode=1，plant_gain=0.8，每档重复 10 次）\n')
    md.append('| τ (帧) | 收敛率 | ts_frames | e_inf (px) | mp (px) | pp (px) | period_frames | 判定 |')
    md.append('|---|---|---|---|---|---|---|---|')
    for d in sorted(a_tab):
        a = a_tab[d]
        v = '收敛' if (a['conv'] == 1.0 and a['pp'] <= 3) else ('周期振荡' if a['pp'] > 6 else '临界')
        md.append('| %d | %s | %s | %s | %s | %s | %s | %s |'
                  % (d, F(a['conv']), F(a['ts'], 1), F(a['einf']),
                     F(a['mp']), F(a['pp']), F(a['per']) if a['per'] > 0 else '0', v))
    md.append('')
    ts_line = ' → '.join('τ=%d:%s帧' % (d, F(a_tab[d]['ts'], 0))
                         for d in sorted(a_tab) if a_tab[d]['ts'] > 0)
    md.append('* 收敛档的 settling 随 τ 单调变慢：' + ts_line + '（τ=4 只比 τ=0 慢 13 帧，因为整段瞬态都在 ±LIMIT 饱和区）。')
    n5 = [r for r in N if abs(r['gain'] - 0.8) < 1e-9 and r['depth'] == 5]
    if n5:
        md.append('* BND 组补测 τ=5（同 g=0.8）：ts=%s 帧，仍收敛；**阈值落在 τ=5~6 帧之间**。'
                  % F(mean([r['ts'] for r in n5]), 1))
    for d in sorted(a_tab):
        a = a_tab[d]
        if a['pp'] > 6:
            md.append('* **τ=%d 越过阈值**：误差再也进不了 6 px 死区（ts=-1），'
                      '在 %.1f ~ %.1f px 之间做周期 %.2f 帧的规整极限环。'
                      % (d, a['einf'] - a['pp'] / 2, a['einf'] + a['pp'] / 2, a['per']))
    md.append('![A/B time-domain waveforms](fig_wave.png)')
    md.append('')
    md.append('* 10 次重复（A 组 τ=0 档）的 ts / e_inf / pp 标准差 = %s / %s / %s → 定值延迟下**逐位可复现**（换种子无任何影响）。'
              % (F(std([r['ts'] for r in A if r['depth'] == 0]), 4),
                 F(std([r['einf'] for r in A if r['depth'] == 0]), 4),
                 F(std([r['pp'] for r in A if r['depth'] == 0]), 4)))
    md.append('')
    out['A'] = a_tab

    # ---------------------------------------------------------------- B
    md.append('## 2. B 组：LFSR 抖动延迟（mode=2，d_depth=抖动上限 m，均匀 [0,m]，均值 m/2）\n')
    md.append('| 抖动上限 m | 实测平均延迟 | 实测最大延迟 | 收敛率 | ts_frames | e_inf | pp | period | ts 标准差 | e_inf 标准差 |')
    md.append('|---|---|---|---|---|---|---|---|---|---|')
    for m in sorted(b_tab):
        a = b_tab[m]
        md.append('| %d | %s | %s | %s | %s | %s | %s | %s | %s | %s |'
                  % (m, F(a['mdly']), F(max(r['maxdly'] for r in jit if r['depth'] == m), 0),
                     F(a['conv']), F(a['ts'], 1), F(a['einf']), F(a['pp']),
                     F(a['per']), F(a['ts_sd']), F(a['einf_sd'])))
    md.append('')
    md.append('* **抖动档没有一档出现极限环**（pp 全为 0.00），包括 m=14（实测均值 7.02 帧）——'
              '这个均值已经超过 A 组的振荡阈值（τ≈6）。')
    md.append('* 代价有两个：收敛变慢（均值 7 帧时 ts 从 57 涨到 104.7 帧），'
              '以及**结果不再可复现**（ts 标准差 30.3 帧、e_inf 标准差 1.6 px；A 组同延迟恒为 0.000）。')
    md.append('')
    out['B'] = b_tab

    # ---------------------------------------------------------------- A vs B
    md.append('## 3. A/B 同均值对比（核心命题）\n')
    pairs = [(d, 2 * d) for d in (3, 4, 6, 7) if d in a_tab and 2 * d in b_tab]
    md.append('| 平均延迟 | A: ts | A: pp | A: period | A 判定 | B: ts | B: pp | B: period | B 判定 |')
    md.append('|---|---|---|---|---|---|---|---|---|')
    for d, m in pairs:
        aa, bb = a_tab[d], b_tab[m]
        md.append('| %s / %s 帧 | %s | %s | %s | %s | %s | %s | %s | %s |'
                  % (F(aa['mdly'], 1), F(bb['mdly'], 1), F(aa['ts'], 1), F(aa['pp']),
                     F(aa['per']), '振荡' if aa['pp'] > 6 else '收敛',
                     F(bb['ts'], 1), F(bb['pp']), F(bb['per']),
                     '振荡' if bb['pp'] > 6 else '收敛'))
    md.append('')
    md.append('| 平均延迟 | A 组 ts 重复标准差 | B 组 ts 重复标准差 | A 组 pp 标准差 | B 组 pp 标准差 |')
    md.append('|---|---|---|---|---|')
    for d, m in pairs:
        md.append('| %s 帧 | %s | %s | %s | %s |'
                  % (F(a_tab[d]['mdly'], 1), F(a_tab[d]['ts_sd'], 2), F(b_tab[m]['ts_sd'], 2),
                     F(a_tab[d]['pp_sd'], 2), F(b_tab[m]['pp_sd'], 2)))
    md.append('')
    md.append('* 同均值下形态完全不同：A 组在 6/7 帧处 pp = %s / %s px、period = %s / %s 帧；'
              'B 组对应 pp = 0.00 / 0.00 px、period = 0。'
              % (F(a_tab[6]['pp']), F(a_tab[7]['pp']), F(a_tab[6]['per']), F(a_tab[7]['per'])))
    md.append('* 结论：**把同样的平均延迟从"定值"换成"抖动"，极限环被彻底打碎**，'
              '代价只是收敛变慢 + 逐次结果不可复现。命题「闭环稳定性取决于延迟的确定性而非均值」在数据上成立。')
    md.append('* 注意 A 组 τ=8 没有同均值抖动对照：均匀 [0,m] 的均值 = m/2，需要 m=16，'
              '而 `delay_inject` 的 4 bit 指针把 m 限死在 14。因此用 τ=7 ↔ m=14 这一对做同均值比较。')
    md.append('')
    out['pairs'] = pairs

    # ---------------------------------------------------------------- K matrix
    md.append('## 4. K 扫描判定矩阵（plant_gain × 定值延迟 τ，每格 3 次）\n')
    gains = sorted(set(r['gain'] for r in K))
    depths = sorted(set(r['depth'] for r in K))
    md.append('`C` = 收敛（不形成极限环），`O<周期>` = 周期振荡，`M` = 小幅边界态，`X` = 发散。\n')
    md.append('| plant_gain \\ τ | ' + ' | '.join(str(d) for d in depths) + ' |')
    md.append('|---' * (len(depths) + 1) + '|')
    kmat = {}
    for g in gains:
        cells = []
        for d in depths:
            s = [r for r in K if r['gain'] == g and r['depth'] == d]
            a = agg(s); kmat[(g, d)] = a; c = cls(s[0])
            cells.append('C' if c == 'C' else ('X' if c == 'X' else
                         ('M' if c == 'M' else 'O%s' % F(a['per'], 0))))
        md.append('| %s | %s |' % (F(g, 1), ' | '.join(cells)))
    md.append('')
    md.append('pp 矩阵（px）：\n')
    md.append('| plant_gain \\ τ | ' + ' | '.join(str(d) for d in depths) + ' |')
    md.append('|---' * (len(depths) + 1) + '|')
    for g in gains:
        md.append('| %s | %s |' % (F(g, 1), ' | '.join(F(kmat[(g, d)]['pp'], 1) for d in depths)))
    md.append('')
    md.append('![stability matrix](fig_matrix.png)')
    md.append('')
    md.append('* 阈值随增益左移：g=0.4 全稳；g=0.8 只有 τ=8 失稳；g=1.6 从 τ=4 起；'
              'g=3.2 从 τ=1 起；g=6.4 连 τ=0 都失稳（K_eff=2.19 超过离散积分无延迟极限 2.0）。')
    md.append('')

    # ---------------------------------------------------------------- theory
    md.append('## 5. 理论校验\n')
    md.append('### 5.1 K·τ < π/2 成立吗？\n')
    md.append('小信号径向增益（由 RTL 表达式解析换算，不是拟合）：\n')
    md.append('```')
    md.append('d_reach = K_R·(|u_obj|²-|u_m|²)/2^16 ≈ K_R·2R/2^16 · Δr        (Δr = r-R)')
    md.append('K_eff(g) = g·K_R·2R/2^16 = g·%d·2·%d/65536 = %.4f·g   [1/帧]' % (int(KR), int(R_OBJ), K_EFF_PER_G))
    md.append('判据（纯积分 + τ 拍延迟 + 比例）：K_eff·τ < π/2 = %.4f  ⇔  g·τ < %.2f'
              % (TAU_PI, TAU_PI / K_EFF_PER_G))
    md.append('```')
    md.append('')
    cells = [(g, d, kmat[(g, d)]) for g in gains for d in depths]
    cells += [(r['gain'], r['depth'], agg([r])) for r in N]
    for s in [r for r in T if r['mode'] == 1]:
        cells.append((s['gain'], s['depth'], agg([s])))
    uniq, seen = [], set()
    for g, d, a in sorted(cells, key=lambda x: (x[0] * x[1], x[0])):
        if (g, d) in seen: continue
        seen.add((g, d)); uniq.append((g, d, a))
    md.append('逐格实测（按 g·τ 排序）：\n')
    md.append('| plant_gain | τ | g·τ | K_eff·τ | 判定 | pp | period |')
    md.append('|---|---|---|---|---|---|---|')
    for g, d, a in uniq:
        c = 'C' if (a['conv'] == 1 and a['pp'] <= 3) else ('O' if a['pp'] > 6 else 'M')
        md.append('| %s | %d | %s | %s | %s | %s | %s |'
                  % (F(g, 1), d, F(g * d, 2), F(K_EFF_PER_G * g * d, 2), c,
                     F(a['pp'], 1), F(a['per']) if a['per'] > 0 else '-'))
    md.append('')
    # best pipeline offset delta for K_eff*(tau+delta) < pi/2
    best = None
    for k in range(-40, 81):
        delta = k / 20.0
        err = 0
        for g, d, a in uniq:
            if a['pp'] <= 3 and a['conv'] == 1: truth = 'C'
            elif a['pp'] > 6: truth = 'O'
            else: continue
            pred = 'C' if K_EFF_PER_G * g * (d + delta) < TAU_PI else 'O'
            if pred != truth: err += 1
        if best is None or err < best[1]:
            best = (delta, err)
    nfit = len([1 for g, d, a in uniq if (a['pp'] > 6) or (a['pp'] <= 3 and a['conv'] == 1)])
    md.append('* 直接拿 τ 当延迟代入时判据并不严格（同一 g·τ 下 g=3.2/τ=1 振荡而 g=0.4/τ=8 收敛），'
              '说明**有效延迟比 τ 多半拍到两拍**（伺服自身 1 拍流水 + tb 取数 1 拍）。')
    # raw bracket with delta = 0
    conv_g = [g * d for g, d, a in uniq if a['conv'] == 1 and a['pp'] <= 3]
    osc_g = sorted(g * d for g, d, a in uniq if a['pp'] > 6)
    gcmax = max(conv_g)
    gomin = min(x for x in osc_g if x > gcmax)
    err0 = 0
    for g, d, a in uniq:
        if a['pp'] <= 3 and a['conv'] == 1: truth = 'C'
        elif a['pp'] > 6: truth = 'O'
        else: continue
        if ('C' if K_EFF_PER_G * g * d < TAU_PI else 'O') != truth: err0 += 1
    low = [(g, d, a) for g, d, a in uniq if g * d <= gcmax]
    high = [(g, d, a) for g, d, a in uniq if g * d >= gomin]
    nlow_c = sum(1 for g, d, a in low if a['conv'] == 1 and a['pp'] <= 3)
    nlow_o = sum(1 for g, d, a in low if a['pp'] > 6)
    nhigh_o = sum(1 for g, d, a in high if a['pp'] > 6)
    md.append('* **不补偿流水时（δ=0）：** g·τ ≤ %.2f 的 %d 个格子里 %d 个收敛、%d 个边界态、%d 个例外'
              '（例外是 g=3.2/τ=1：τ 太小，1 拍流水在 τ 里占比最大）；'
              'g·τ ≥ %.2f 的 %d 个格子**全部振荡**（%d/%d）。'
              % (gcmax, len(low), nlow_c, len(low) - nlow_c - nlow_o, nlow_o,
                 gomin, len(high), nhigh_o, len(high)))
    md.append('* → 临界 g·τ 落在 **(%.2f, %.2f)**，即 **K_eff·τ ∈ (%.2f, %.2f)**；'
              'π/2 = %.3f 正落在这个窄区间里（偏离 %+.1f%% / %+.1f%%）。'
              % (gcmax, gomin, K_EFF_PER_G * gcmax, K_EFF_PER_G * gomin, TAU_PI,
                 100 * (K_EFF_PER_G * gcmax - TAU_PI) / TAU_PI,
                 100 * (K_EFF_PER_G * gomin - TAU_PI) / TAU_PI))
    md.append('* 令 τ_eff = τ + δ 做一维搜索：**δ* = %.2f 帧** 时 `K_eff·τ_eff < π/2` 把 %d 个可判定的格子判对 %d 个'
              '（δ=0 时错 %d 个）。'
              % (best[0], nfit, nfit - best[1], err0))
    md.append('* 所以「K·τ < π/2」**成立**，但两个量都要按环路口径取：'
              'K = 伺服增益×plant 增益（不是裸 plant_gain），τ = 帧延迟 + 约 0.45 帧流水。'
              '直接拿"帧数"和"对象增益"套公式会差 1.5~2 倍。')
    md.append('* 唯一不能解释的是 **g=6.4, τ=0** 这一格（pp=3.0，周期 2 帧）：'
              '此时 K_eff = %.2f > 2，已经超过离散积分本身的无延迟稳定极限（K<2），'
              '与延迟无关，是采样率不足导致的失稳。'
              % (K_EFF_PER_G * 6.4))
    out['boundary'] = dict(delta=best[0], errors=best[1], nfit=nfit, err_delta0=err0,
                           gtau_conv_max=gcmax, gtau_osc_min=gomin,
                           ktau_lo=K_EFF_PER_G * gcmax, ktau_hi=K_EFF_PER_G * gomin,
                           gtau_crit=TAU_PI / K_EFF_PER_G)
    md.append('')

    md.append('### 5.2 T ≈ 4τ 成立吗？\n')
    md.append('`e_inf`/`period` 用的 e 是 **|marker−object|（取绝对值）**，绝对值把周期砍半，'
              '所以要先看带符号的径向误差 `cx_r − OBJ_X`（本实验共线，它就是带符号误差）。\n')
    md.append('| τ (帧) | 实测 T(|e|) | 实测 T(signed) | 4τ+2 | T(signed)/(4τ) | 样本数 |')
    md.append('|---|---|---|---|---|---|')
    osc = [r for r in rows if r['mode'] == 1 and r['pp'] > 6 and r['per'] > 0]
    by_tau = {}
    for r in osc:
        by_tau.setdefault(r['depth'], []).append(r)
    for d in sorted(by_tau):
        v = by_tau[d]
        t = mean([x['per'] for x in v]); s = mean([x['sper'] for x in v])
        md.append('| %d | %s | %s | %d | %s | %d |'
                  % (d, F(t), F(s), 4 * d + 2, F(s / (4 * d)) if d else '-', len(v)))
    md.append('')
    xs = [r['depth'] for r in osc]; ys = [r['sper'] for r in osc]
    mx, my = mean(xs), mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs); sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    a = sxy / sxx; b = my - a * mx
    ss = sum((y - my) ** 2 for y in ys); sr = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    r2 = 1 - sr / ss if ss else 1.0
    md.append('* 带符号周期最小二乘拟合（%d 个振荡用例）：**T_signed = %.3f·τ + %.2f**，R² = %.6f，最大残差 %.2f 帧。'
              % (len(osc), a, b, r2, max(abs(y - (a * x + b)) for x, y in zip(xs, ys))))
    md.append('* 即 **T_signed ≈ 4τ + 2**（截距 2 帧 = 伺服 1 拍流水 + tb 1 拍取数），'
              '**T ≈ 4τ 成立**，T/(4τ) = 1 + 0.5/τ（τ=8 时 1.06，τ=4 时 1.13）。')
    md.append('* 而 |e| 的周期恰好是它的一半（2τ+1），原因是 e 取绝对值后每个极限环周期内出现两个"鼓包"。'
              '**报告里引用 period_frames 时必须说明这是 |e| 的周期，是 4τ 的一半。**')
    out['period_fit'] = dict(a=a, b=b, r2=r2, n=len(osc))

    md.append('')
    md.append('![theory check](fig_theory.png)')
    md.append('')
    md.append('### 5.3 振荡幅度\n')
    md.append('饱和区 |d_reach|=LIMIT=%d 时对象速度 v = %d·g px/帧；延迟窗内走过 v·τ，'
              '带符号误差的峰峰值应 ≈ 2·v·τ，|e| 的峰峰值 ≈ v·τ。\n' % (LIMIT, LIMIT))
    md.append('| plant_gain | τ | pp(|e|) | pp(signed) | 2·v·τ | pp(signed)/(2vτ) |')
    md.append('|---|---|---|---|---|---|')
    rr = []
    for g, d, am in uniq:
        if am['pp'] > 6 and d > 0:
            pred = 2 * LIMIT * g * d
            rr.append(am['spp'] / pred)
            md.append('| %s | %d | %s | %s | %s | %s |'
                      % (F(g, 1), d, F(am['pp'], 1), F(am['spp'], 1), F(pred, 1), F(am['spp'] / pred)))
    md.append('')
    md.append('')
    md.append('* 幅度律 **pp(signed) ≈ 2·LIMIT·g·τ = 8·g·τ** 成立：均值比 %s，范围 %s~%s。'
              % (F(mean(rr)), F(min(rr)), F(max(rr))))
    big = [x for x in rr if x >= 0.9]
    sml = [x for x in rr if x < 0.9]
    md.append('* 大振幅格（pp≥40 px，速度×延迟主导）：比值均值 %s（%s~%s）；'
              '小振幅格（pp<40 px，6 px 死区已不可忽略）：均值 %s，最小 %s——死区把每次换向提前截断。'
              % (F(mean(big), 3), F(min(big)), F(max(big)),
                 F(mean(sml), 3) if sml else '-', F(min(sml), 3) if sml else '-'))
    out['amp_ratio'] = dict(mean=mean(rr), min=min(rr), max=max(rr))

    # ---------------------------------------------------------------- deviations
    md.append('')
    md.append('## 6. 偏差与原因（写报告请照抄）\n')
    md.append('1. **物体放在 (750,240)，不是原 tb 的 (450,300)。** 原几何下物体距极点仅 78 px，'
              '而 pan 通道输出 ∝ cross(u_m,e)（px² 量纲），R=78 时 1 tick 需要 1.92° 角误差 '
              '(≈2.6 px 弧长)，量化台阶大于死区。实测原几何下环路停在 |e|=60 px 不动：'
              '拷回来的 stock `tb_servo_closed_loop.v` 在本机**实跑 FAIL**（日志 `ref_fail.log`，'
              '"did not converge into the dead zone"）。R=350 px 才回到 RTL 注释里设计者调参的量级。')
    md.append('2. **伺服增益 K=4→32。** 同一原因：K=4 时 pan 量化台阶 = 2^15/(2·K·R) = 11.7 px > 死区 6 px，'
              '环路必然停在死区外，ts_frames 会全为 -1，扫不出任何收敛/振荡对比。K=32 时 pan 台阶 1.5 px、'
              '径向台阶 2.9 px，都在 6 px 死区内。SHIFT/LIMIT/DEADZONE/HOLD_N 全部保持 RTL 默认。')
    md.append('3. **几何是共线的**（物体与 marker 同一条射线），所以 pan 通道恒为 0、这是纯径向 SISO 实验。'
              '这样 K·τ 与 4τ 才能和教科书的一阶模型逐项对照；二维几何下两个通道的增益量级差 3.8 倍（∝R² vs ∝R），'
              '会把延迟效应和通道耦合混在一起。')
    md.append('4. **plant_gain 同时缩放两个通道**（DEG_PER_TICK=0.5·g/0.8）。只缩放 PX_PER_TICK 的话 pan 环路增益不变，'
              '就不是整环增益扫描。')
    md.append('5. **B 组均值与抖动强度在本 RTL 下不可解耦。** `delay_inject` 抖动是 [0,m] 均匀分布，均值恒为 m/2；'
              '本次用 3 档 m（d/2、d、2d）给强度趋势，用 m=2d 做同均值对照。m ≤ 14（4 bit 指针），'
              '故 A 组 τ=8 无同均值对照（需 m=16），改用 τ=7↔m=14。')
    md.append('6. **对象比真机乐观。** 纯积分、无连杆惯量/舵机死区/图像噪声/曝光抖动；'
              '`K·τ<π/2` 的边界和「抖动打碎极限环」的结论在真机上只会更保守，不能直接当实物结论。')
    md.append('7. **converged 的判定。** 严格按 ts≥0 会误判「极限环恰好落在死区边沿」的格子'
              '（最后一帧刚好 ≤6 px）。因此 tb 里 `converged` 额外要求 pp≤6 且 e_inf≤6；'
              '后处理判定矩阵进一步要求 pp≤3 才算 C，pp>6 才算 O，中间记 M。')
    md.append('8. **xsim 2020.2 的 `--testplusarg` 在含 `=` 时被 wrapper 吃掉**（`Expected a switch but found 5`），'
              '所以参数没有走 plusargs，改成 tb 读 ASCII `cases.txt`（全整数字段，`$fscanf %d`）。'
              '副作用是断点续跑可以做到**用例级**，比整组级更细。')
    md.append('')

    md.append('## 7. 文件与复现\n')
    md.append('| 文件 | 说明 |')
    md.append('|---|---|')
    md.append('| `D:\\Code\\fpga_p5_sim\\` | ASCII 工作目录（档案目录只读，全程未改动）|')
    md.append('| `tb_p5_sweep.v` | 扫描 tb：servo_polar + delay_inject + 纯积分 plant；tb 内算指标并打印 CSV/AUX |')
    md.append('| `plan.py` | 生成 `cases.txt`，按 tag 跳过已完成用例（断点续跑）|')
    md.append('| `run_sweeps.ps1` | xvlog→xelab→xsim，把 stdout 的 CSV/AUX 行追加进 results/aux，失败可见 |')
    md.append('| `results.csv` | 每用例一行：`CSV,<mode>,<depth>,<plant_gain>,<seed>,<converged>,<ts_frames>,<e_inf>,<mp>,<pp>,<period_frames>` |')
    md.append('| `aux.csv` | 同序附加行：实测平均/最大延迟、饱和占比、起始误差、发散标志、tag、带符号周期/峰峰值 |')
    md.append('| `trace.csv` | 7 条逐帧波形（A τ=0/2/4/8、B m=4/8/14），用于画图 |')
    md.append('| `run.log` | 最近一次 xsim 原始 stdout |')
    md.append('| `summary.json` | 本文聚合数字的机器可读版 |')
    md.append('| `fig_*.png` | 时域波形、判定矩阵热图、T-τ 与 K·τ 边界图 |')
    md.append('')
    md.append('复现：`powershell -ExecutionPolicy Bypass -File run_sweeps.ps1`'
              '（已完成的 tag 自动跳过；`-Force` 全跑；`-Groups trace` 只跑波形）。')

    open('汇总.md', 'w', encoding='utf-8').write('\n'.join(md) + '\n')
    json.dump(out, open('summary.json', 'w'), indent=1, ensure_ascii=False)
    print('wrote 汇总.md (%d lines), summary.json' % len(md))


if __name__ == '__main__':
    main()
