# Generator dataset graf state-space Rubik 2x2x2 (pojok kiri-bawah-belakang tetap, gerakan R U F)
# pakai: python generate_dataset.py [max_depth=7] [folder=dataset] [query_per_depth=6]
import sys, csv, os, random, heapq
from collections import deque
MAXD = int(sys.argv[1]) if len(sys.argv) > 1 else 7
OUT = sys.argv[2] if len(sys.argv) > 2 else 'dataset'
PER = int(sys.argv[3]) if len(sys.argv) > 3 else 6
os.makedirs(OUT, exist_ok=True)
FN = ['px', 'nx', 'py', 'ny', 'pz', 'nz']
FACES = [('px', (1, 0, 0)), ('nx', (-1, 0, 0)), ('py', (0, 1, 0)), ('ny', (0, -1, 0)), ('pz', (0, 0, 1)), ('nz', (0, 0, -1))]
SLOTS, SIDX = [], {}
for k, n in FACES:   # slot dikelompokkan per sisi: kanan, kiri, atas, bawah, depan, belakang (4 stiker per sisi)
    for x in (-.5, .5):
        for y in (-.5, .5):
            for z in (-.5, .5):
                if n[0]*x + n[1]*y + n[2]*z > 0:
                    SIDX[((x, y, z), n)] = len(SLOTS); SLOTS.append(((x, y, z), n, k))
def rot(v, ax, s):  # putar 90 derajat, s=+1/-1 (sama dengan applyAxisAngle di Three.js)
    x, y, z = v
    if ax == 0: return (x, -z*s, y*s)
    if ax == 1: return (z*s, y, -x*s)
    return (-y*s, x*s, z)
def perm(ax, s):
    pm = []
    for i, (p, n, k) in enumerate(SLOTS):
        if p[ax] < 0: pm.append(i); continue
        pm.append(SIDX[(rot(p, ax, s), rot(n, ax, s))])
    return pm
MV = []
for f, ax in (('R', 0), ('U', 1), ('F', 2)):
    q, qi = perm(ax, -1), perm(ax, 1)
    MV += [(f, q, 1), (f + "'", qi, 1), (f + '2', [q[q[i]] for i in range(24)], 2)]
INV = []
for nm, pm, c in MV:
    inv = [0]*24
    for i in range(24): inv[pm[i]] = i
    INV.append(inv)
ap = lambda s, mi: bytes(map(s.__getitem__, INV[mi]))
T0 = bytes(FN.index(k) for _, _, k in SLOTS)
ids = {T0: 0}; states = [T0]; depth = [0]; par = [(-1, -1)]; dq = deque([0])
while dq:
    i = dq.popleft()
    if depth[i] >= MAXD: continue
    for mi in range(9):
        t = ap(states[i], mi)
        if t not in ids:
            ids[t] = len(states); states.append(t); depth.append(depth[i]+1); par.append((i, mi)); dq.append(ids[t])
N = len(states)
nbr = [[ids.get(ap(states[i], mi), -1) for mi in range(9)] for i in range(N)]
# jarak biaya optimal (Dijkstra dari goal di dalam graf dataset)
INF = 10**9; dist = [INF]*N; dist[0] = 0; hp = [(0, 0)]
while hp:
    d, i = heapq.heappop(hp)
    if d > dist[i]: continue
    for mi, j in enumerate(nbr[i]):
        if j >= 0 and d + MV[mi][2] < dist[j]:
            dist[j] = d + MV[mi][2]; heapq.heappush(hp, (dist[j], j))
g = {}
for i, (p, n, k) in enumerate(SLOTS): g.setdefault(p, []).append(i)
CORN = list(g.values())
hg = lambda s: sum(1 for i in range(24) if s[i] != T0[i])
ha = lambda s: -(-sum(1 for c in CORN if any(s[i] != T0[i] for i in c)) // 4)
with open(f'{OUT}/nodes.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['id', 'state', 'depth_opt_moves', 'opt_cost', 'h_gbfs', 'h_astar'])
    for i, s in enumerate(states): w.writerow([i, ''.join(map(str, s)), depth[i], dist[i], hg(s), ha(s)])
with open(f'{OUT}/edges.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['src', 'dst', 'move', 'cost'])
    for i in range(N):
        for mi, j in enumerate(nbr[i]):
            if j >= 0: w.writerow([i, j, MV[mi][0], MV[mi][2]])
random.seed(42)
with open(f'{OUT}/queries.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['query_id', 'kategori', 'start_id', 'depth_opt_moves', 'scramble'])
    q = 0
    for d in range(3, MAXD+1):
        pool = [i for i in range(N) if depth[i] == d]
        for i in random.sample(pool, min(PER, len(pool))):
            path, c = [], i
            while c != 0: p, mi = par[c]; path.append(MV[mi][0]); c = p
            q += 1; w.writerow([q, 'dangkal' if d <= 5 else 'sedang' if d <= 8 else 'dalam', i, d, ' '.join(reversed(path))])
print('nodes', N, 'edges', sum(1 for r in nbr for j in r if j >= 0))
