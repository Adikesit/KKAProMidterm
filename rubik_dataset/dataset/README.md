# Dataset graf state-space Rubik 2x2x2

Satu node = satu kondisi kubus (24 stiker). Satu edge = satu gerakan. Model gerakan sama dengan solver di Simulator_Rubik_2x2x2.html (pojok kiri-bawah-belakang tetap, gerakan R, U, F beserta prime dan 2).
Biaya: putaran 90 derajat = 1, putaran 180 derajat (R2, U2, F2) = 2.

Isi: semua state dengan jarak optimal <= 7 gerakan dari goal (289.896 node, 1.462.284 edge). Seluruh ruang state aslinya 3.674.160 node, jadi terlalu besar untuk dibagikan sebagai file. Bisa dibangkitkan lagi dengan generate_dataset.py (argumen pertama = kedalaman maksimum).

- nodes.csv: id (0 = goal), state (24 digit warna 0-5, dikelompokkan per sisi: 4 digit kanan, kiri, atas, bawah, depan, belakang; goal = 000011112222333344445555), depth_opt_moves (jumlah gerakan optimal ke goal), opt_cost (biaya optimal ke goal), h_gbfs (jumlah stiker salah), h_astar (ceil pojok salah / 4)
- edges.csv: src, dst, move, cost (menerapkan move pada state src menghasilkan state dst)
- queries.csv: 30 kasus uji (6 per kedalaman 3-7). scramble = rumus dari kubus solved ke state awal, bisa langsung dimasukkan ke kolom rumus simulator setelah Reset.
- kategori: dangkal (kedalaman 3-5), sedang (6-8). Kategori dalam (9-12) belum ada di dataset ini.
