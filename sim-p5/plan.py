#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""plan.py - build cases.txt for tb_p5_sweep (and filter out finished tags).

case line:  mode depth gain_milli seed nframes obj_x obj_y r0_milli phi0_milli trace tag

usage:
  python plan.py --groups a,b,k,m,match,trace --out cases.txt [--skip aux.csv]
                 [--nf 400] [--ox 750] [--oy 240] [--r0 160] [--reps 10]
"""
import argparse, csv, os, sys

# ---- geometry defaults: object at R=350 px from the pole (400,240) ----------
DEFAULTS = dict(nf=400, ox=750, oy=240, r0=160.0, gain=0.8, reps=10, kreps=3)

A_DEPTHS = [0, 1, 2, 4, 8]          # required A ladder
K_GAINS  = [0.4, 0.8, 1.6, 3.2, 6.4]
B_STR    = [1, 2, 4]                # jitter cap = depth * str / 2
CAP_MAX  = 14                       # delay_inject D_MAX
MATCH_A  = [3, 6, 7]                # extra A depths whose mean is reproducible
MATCH_B  = [6, 12, 14]              # matching jitter caps (mean = cap/2)


def mk(mode, depth, gain, seed, tag, cfg, trace=0, r0=None, jitA=0):
    return dict(mode=mode, depth=depth, gain=gain, seed=seed, nf=cfg.nf,
                ox=cfg.ox, oy=cfg.oy,
                r0=(cfg.r0 if r0 is None else r0),
                phi0=0.0, trace=trace, tag=tag, jitA=jitA)


def build(cfg):
    cases = []

    def seq(base):
        n = [0]

        def nxt():
            t = base + n[0]
            n[0] += 1
            return t
        return nxt

    # ---------------- A : fixed delay, gain 0.8, 10 reps ----------------
    nxt = seq(10000)
    for d in A_DEPTHS:
        for rep in range(cfg.reps):
            cases.append(mk(1, d, cfg.gain, rep, nxt(), cfg))

    # ---------------- B : LFSR jitter, 3 strengths, 10 reps ----------------
    nxt = seq(20000)
    for d in A_DEPTHS:
        for si, s in enumerate(B_STR):
            cap = d * s // 2
            if d == 0:
                cap = si                     # reference row: caps 0/1/2
            if cap < 1 and d > 0:
                cap = 1
            cap = min(cap, CAP_MAX)
            for rep in range(cfg.reps):
                cases.append(mk(2, cap, cfg.gain, rep, nxt(), cfg))

    # ---------------- K : plant_gain x depth, 3 reps ----------------
    nxt = seq(30000)
    for g in K_GAINS:
        for d in A_DEPTHS:
            for rep in range(cfg.kreps):
                cases.append(mk(1, d, g, rep, nxt(), cfg))

    # ---------------- M : mean-matched A/B morphology pairs ----------------
    nxt = seq(40000)
    for d in MATCH_A:
        for rep in range(cfg.reps):
            cases.append(mk(1, d, cfg.gain, rep, nxt(), cfg))
    for cap in MATCH_B:
        for rep in range(cfg.reps):
            cases.append(mk(2, cap, cfg.gain, rep, nxt(), cfg))

    # ---------------- BND : bracket the g*tau boundary, 3 reps -------------
    # loop gain K_eff = 0.342*g per frame (radial channel, R=350, KR=32);
    # criterion K_eff*tau = pi/2  ->  g*tau = 4.59
    nxt = seq(50000)
    for g, d in [(0.4, 9), (0.4, 11), (0.4, 12), (0.4, 14),
                 (0.8, 5), (0.8, 6), (0.8, 7),
                 (1.6, 3), (3.2, 1)]:
        for rep in range(cfg.kreps):
            cases.append(mk(1, d, g, rep, nxt(), cfg))

    # ---------------- SIG : fixed mean, symmetric jitter amplitude A ----------------
    # delay = MEAN + U, U uniform on [-A,+A] -> mean fixed by construction.
    # A=0 must reproduce the A-group fixed-tau run bit for bit (REF-MATCH).
    nxt = seq(60000)
    for mean, Alist in ((6, [0, 1, 2, 3, 4, 6]), (8, [0, 1, 2, 4, 6, 8])):
        for A in Alist:
            reps = 1 if A == 0 else 20
            for rep in range(reps):
                cases.append(mk(3, mean, cfg.gain, rep, nxt(), cfg, jitA=A))

    # ---------------- TRACE : one realisation per interesting setting ------
    nxt = seq(90000)
    for d in [0, 2, 4, 8]:
        cases.append(mk(1, d, cfg.gain, 0, nxt(), cfg, trace=1))
    for cap in [4, 8, 14]:
        cases.append(mk(2, cap, cfg.gain, 0, nxt(), cfg, trace=1))
    return cases


GROUPS = dict(a=(10000, 19999), b=(20000, 29999), k=(30000, 39999),
              m=(40000, 49999), bnd=(50000, 59999), sig=(60000, 69999),
              trace=(90000, 99999))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--groups', default='a,b,k,m')
    ap.add_argument('--out', default='cases.txt')
    ap.add_argument('--skip', default='aux.csv')
    ap.add_argument('--nf', type=int, default=DEFAULTS['nf'])
    ap.add_argument('--ox', type=int, default=DEFAULTS['ox'])
    ap.add_argument('--oy', type=int, default=DEFAULTS['oy'])
    ap.add_argument('--r0', type=float, default=DEFAULTS['r0'])
    ap.add_argument('--reps', type=int, default=DEFAULTS['reps'])
    ap.add_argument('--kreps', type=int, default=DEFAULTS['kreps'])
    ap.add_argument('--gain', type=float, default=DEFAULTS['gain'])
    ap.add_argument('--limit', type=int, default=0, help='cap number of cases (debug)')
    cfg = ap.parse_args()

    done = set()
    if cfg.skip and os.path.exists(cfg.skip):
        with open(cfg.skip, newline='') as f:
            rd = csv.reader(f)
            for row in rd:
                if row and row[0] == 'AUX' and len(row) >= 14:
                    try:
                        done.add(int(row[13]))
                    except ValueError:
                        pass

    sel = []
    for g in cfg.groups.split(','):
        g = g.strip().lower()
        if g in GROUPS:
            sel.append(GROUPS[g])
        elif g == 'all':
            sel += list(GROUPS.values())
    ranges = sel or list(GROUPS.values())

    cases = [c for c in build(cfg)
             if any(lo <= c['tag'] <= hi for lo, hi in ranges)]
    todo = [c for c in cases if c['tag'] not in done]

    with open(cfg.out, 'w', newline='\n') as f:
        for c in todo:
            f.write('%d %d %d %d %d %d %d %d %d %d %d %d\n' % (
                c['mode'], c['depth'], int(round(c['gain'] * 1000)), c['seed'],
                c['nf'], c['ox'], c['oy'],
                int(round(c['r0'] * 1000)), int(round(c['phi0'] * 1000)),
                c['trace'], c['tag'], c['jitA']))
    print('plan: %d cases selected, %d already done, %d queued -> %s'
          % (len(cases), len(cases) - len(todo), len(todo), cfg.out))


if __name__ == '__main__':
    main()
