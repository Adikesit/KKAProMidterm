# Menjalankan BFS, UCS, GBFS, A* di atas dataset graf (nodes.csv, edges.csv, queries.csv)
# pakai: python run_search_dataset.py [folder=dataset]
import sys, csv, time, heapq, tracemalloc
from collections import deque, defaultdict
D = sys.argv[1] if len(sys.argv) > 1 else 'dataset'
MN = ['R', "R'", 'R2', 'U', "U'", 'U2', 'F', "F'", 'F2']; MI = {m: i for i, m in enumerate(MN)}
COST = [1, 1, 2]*3
rows = list(csv.DictReader(open(f'{D}/nodes.csv')))
N = len(rows)
hg = [int(r['h_gbfs']) for r in rows]; ha = [int(r['h_astar']) for r in rows]
dep = [int(r['depth_opt_moves']) for r in rows]; opt = [int(r['opt_cost']) for r in rows]
del rows
NBR = [[-1]*9 for _ in range(N)]
for r in csv.DictReader(open(f'{D}/edges.csv')): NBR[int(r['src'])][MI[r['move']]] = int(r['dst'])
Q = list(csv.DictReader(open(f'{D}/queries.csv')))

def search(algo, s0):
    seen = {s0: 0}; par = {s0: None}; fifo = deque(); hp = []; seq = 0
    exp = 0; gen = 1; peak = 1; goal = None
    bfs = algo == 'bfs'
    def put(n, g):
        nonlocal seq
        if bfs: fifo.append((n, g))
        else:
            p = g if algo == 'ucs' else hg[n] if algo == 'gbfs' else g + ha[n]
            seq += 1; heapq.heappush(hp, (p, seq, n, g))
    put(s0, 0)
    while (fifo if bfs else hp):
        if bfs: n, g = fifo.popleft()
        else: _, _, n, g = heapq.heappop(hp)
        if not bfs and algo != 'gbfs' and seen[n] < g: continue
        exp += 1
        if n == 0: goal = n; break
        for mi, d in enumerate(NBR[n]):
            if d < 0: continue
            g2 = g + COST[mi]; old = seen.get(d)
            if old is not None and (bfs or algo == 'gbfs' or old <= g2): continue
            seen[d] = g2; par[d] = (n, mi); gen += 1; put(d, g2)
        peak = max(peak, len(fifo) if bfs else len(hp))
    sol = []; c = goal
    while c is not None and par[c]: p, mi = par[c]; sol.append(MN[mi]); c = p
    return goal is not None, sol[::-1], seen.get(0, 0), exp, gen, peak, len(seen)

res = []
for q in Q:
    s0 = int(q['start_id'])
    for a, an in (('bfs', 'BFS'), ('ucs', 'UCS'), ('gbfs', 'GBFS'), ('astar', 'A*')):
        t = time.perf_counter(); r = search(a, s0); ms = (time.perf_counter()-t)*1000   # waktu tanpa tracemalloc
        tracemalloc.start(); search(a, s0); mem = tracemalloc.get_traced_memory()[1]/1024; tracemalloc.stop()
        ok, sol, cost, exp, gen, peak, stored = r
        res.append(dict(query_id=q['query_id'], kategori=q['kategori'], depth_opt=q['depth_opt_moves'], algoritma=an, selesai=int(ok),
            waktu_ms=round(ms, 1), node_dieksplorasi=exp, node_dibangkitkan=gen, frontier_maks=peak, node_tersimpan=stored,
            memori_puncak_kb=round(mem), langkah=len(sol), biaya=cost, optimal_langkah=int(len(sol) == dep[s0]), optimal_biaya=int(cost == opt[s0]),
            rumus=' '.join(sol)))
    print('query', q['query_id'], 'selesai', flush=True)
with open('hasil_pencarian.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(res[0])); w.writeheader(); w.writerows(res)
agg = defaultdict(list)
for r in res: agg[(r['kategori'], r['algoritma'])].append(r)
cols = ['waktu_ms', 'node_dieksplorasi', 'node_dibangkitkan', 'frontier_maks', 'node_tersimpan', 'memori_puncak_kb', 'langkah', 'biaya']
with open('ringkasan_per_kategori.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['kategori', 'algoritma', 'jumlah_query'] + ['rata2_' + c for c in cols] + ['persen_optimal_langkah', 'persen_optimal_biaya'])
    for (k, a), L in sorted(agg.items(), key=lambda x: (x[0][0] != 'dangkal', x[0][1] != 'BFS', x[0])):
        avg = lambda c: round(sum(float(r[c]) for r in L)/len(L), 1)
        w.writerow([k, a, len(L)] + [avg(c) for c in cols] + [round(100*sum(r['optimal_langkah'] for r in L)/len(L)), round(100*sum(r['optimal_biaya'] for r in L)/len(L))])
print(open('ringkasan_per_kategori.csv').read())
