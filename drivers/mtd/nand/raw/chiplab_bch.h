/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * chiplab NAND 共享常量与自包含 BCH(8191, 8139, t=4) 编解码（GF(2^13)）。
 *
 * 本文件是 u-boot/drivers/mtd/nand/raw/chiplab_nand.h 的 **逐字摘录**
 * （§2 芯片几何 + §3 ECC 布局 + §6 BCH 编解码），目的是让 U-Boot 与 Linux
 * 用**同一份**几何/布局常量与**同一份**位序实现，从而备用区落盘字节可互换
 * （04-nand-driver.md §4.3：「U-Boot 与 Linux 必须共用」）。
 *
 * 纪律：只允许两棵树同步修改；任何单边改动都会破坏 ECC 契约。
 */
#ifndef __CHIPLAB_NAND_SHARED_H__
#define __CHIPLAB_NAND_SHARED_H__

#include <linux/types.h>
#include <linux/string.h>
#include <linux/kernel.h>

/* --------------------------------------------------------------------- */
/* 2. 芯片几何（K9F1G08U0C 口径）                                          */
/* --------------------------------------------------------------------- */
/*
 * 口径裁决（docs/porting/04-nand-driver.md §1.2 D1 关闭方式）：
 *   「块 128 KiB = 主区 64 页 × 2048 B，备用区 64 × 64 B 另行」
 * 因此：
 *   - 主区页大小 2048 B，备用区 64 B，**控制器一次搬运 2112 B**；
 *   - 块 = 128 KiB = 主区 64 页（备用区不计入块容量）；
 *   - 芯片容量 128 MiB = 1024 块。
 * 控制器侧口径（RTL 实测，不修改 chiplab NAND 模块）：
 *   - op_scope 复位值 2048（nand.v:160 的 32'h0800_5000 高 16 位）＝主区容量；
 *   - 备用区经 0x40 门铃搬运时不使用 op_scope，另走 nand.v:838-844 的路径。
 *   - 一次「主区+备用区」读写的最大搬运量 = 2048 + 64 = 2112 B；
 *     每次门铃搬运 512 B（nand.v:838-844 以 NAND_ADDR[7:0]==0 时 256 B 为界，
 *     即 2048 B 主区 = 4 个 512 B 段，收尾再搬 64 B 备用区）。
 */
#define CHIPLAB_NAND_PAGE_SIZE		2048u
#define CHIPLAB_NAND_OOBSIZE		64u
#define CHIPLAB_NAND_PAGE_SPARE		(CHIPLAB_NAND_PAGE_SIZE + CHIPLAB_NAND_OOBSIZE) /* 2112 */
#define CHIPLAB_NAND_PAGES_PER_BLOCK	64u
#define CHIPLAB_NAND_BLOCK_SIZE		(CHIPLAB_NAND_PAGE_SIZE * CHIPLAB_NAND_PAGES_PER_BLOCK) /* 131072 */
#define CHIPLAB_NAND_BLOCKS		1024u
#define CHIPLAB_NAND_TOTAL_SIZE		(CHIPLAB_NAND_BLOCK_SIZE * CHIPLAB_NAND_BLOCKS) /* 128 MiB */

/* 器件 ID（[旧项目转述]，待上板用 IDL/STATUS_IDH 实读核对；见 04-nand-driver.md §3.2） */
#define CHIPLAB_NAND_MFR_ID		0xecu	/* Samsung */
#define CHIPLAB_NAND_DEV_ID		0xf1u	/* K9F1G08U0C */
#define CHIPLAB_NAND_ID_BYTES		5u	/* EC F1 00 1D 15（后 3 字节为内部信息） */

/* 寄存器时序值：RTL 复位值 {8'h4, 8'h12} = 0x0412（nand.v:155）；Linux 亦写 0x412（ls1a_nand.c:921） */
#define CHIPLAB_NAND_TIMING_RESET	0x0412u

/* --------------------------------------------------------------------- */
/* 3. ECC 布局（U-Boot / Linux 必须共用，04-nand-driver.md §4.3）           */
/* --------------------------------------------------------------------- */
/*
 * 算法：软件 BCH（GF(2^13)，可纠 4 bit / 512 B 段）
 *   段大小 512 B，每页 4 段，每段 ECC 7 B（52 bit 校验位 + 4 bit 填充），共 28 B。
 * 布局（备用区 64 B）：
 *   偏移 0..1   出厂坏块标记（offset 0; 非 0xFF 即坏块）+ 保留字节（offset 1，恒 0xFF）
 *   偏移 2..35  oobfree[0]（34 B，供 BBT 版本/运行时坏块标记等使用）
 *   偏移 36..63 ECC 字节（4 段 × 7 B，段 j 占 36+7j .. 36+7j+6）
 * 该布局等价于 U-Boot nand_bch_init() 的默认布局（drivers/mtd/nand/raw/nand_bch.c:
 * eccpos 放备用区尾部、oobfree 从 offset 2 开始），因此两侧口径一致。
 */
#define CHIPLAB_NAND_ECC_STEP_SIZE	512u
#define CHIPLAB_NAND_ECC_STEPS		4u	/* 2048 / 512 */
#define CHIPLAB_NAND_ECC_BYTES_PER_STEP	7u	/* ceil(13*4/8) */
#define CHIPLAB_NAND_ECC_TOTAL_BYTES	28u	/* 4 * 7 */
#define CHIPLAB_NAND_ECC_STRENGTH	4u	/* 可纠 4 bit/段 */
#define CHIPLAB_NAND_ECC_FIRST_POS	(CHIPLAB_NAND_OOBSIZE - CHIPLAB_NAND_ECC_TOTAL_BYTES) /* 36 */

/* 坏块标记（04-nand-driver.md §6.2） */
#define CHIPLAB_NAND_BBT_FACTORY_OFF	0u	/* 备用区 offset 0，非 0xFF = 坏块 */
#define CHIPLAB_NAND_BBT_RUNTIME_OFF	1u	/* 备用区 offset 1，运行时坏块标记 */
#define CHIPLAB_NAND_BB_MARKER_BAD	0x00u	/* 出厂标记值 */

/* --------------------------------------------------------------------- */
/* 6. BCH(8191, 8139, t=4) 编解码 —— 自held，GF(2^13)                       */
/* --------------------------------------------------------------------- */
/*
 * 与 Linux / U-Boot 的通用 BCH 库**不共用位序**：本实现把数据位按「字节内低位在先」
 * 铺进码字高位（见 ecc bit packing），U-Boot 侧与将来 Linux 侧必须用同一份实现
 * （04-nand-driver.md §4.3 的"两侧共用单一头文件"口径）。位序一旦改动，
 * 已写入的数据将无法纠正 —— 单元测试的 test_bit_order 锁死该约定。
 *
 * 码字：n = 2^13 - 1 = 8191 bit，其中信息 = 512 B = 4096 bit，
 * 校验 = 52 bit（存 7 B，高 4 bit 填充 0）。生成多项式由 α^1..α^8 的极小多项式乘积
 * 在运行时构造（避免硬编码出错），并用 g(α^i)=0 (i=1..8) 自检。
 */

/*
 * 纠错能力上限（每 512 B 段可纠 bit 数）。**这是唯一的能力开关**：
 *   - U-Boot 侧由 CONFIG_NAND_CHIPLAB_ECC_STRENGTH 覆盖（Kconfig range 1..4）；
 *   - 宿主机单元测试可用 -DCHIPLAB_ECC_STRENGTH_DEFAULT=n 覆盖，
 *     用于「把能力调小后 >n bit 注入错误必须 FAIL」的反证实验。
 */
#ifndef CHIPLAB_ECC_STRENGTH_DEFAULT
#define CHIPLAB_ECC_STRENGTH_DEFAULT	4
#endif

/*
 * 解码侧能力上限（用于反证实验）。默认等于编码侧的强度，正常工作下
 * 两者必须一致；单元测试用 -DCHIPLAB_ECC_DECODE_STRENGTH=3 把它调小，
 * 模拟"BCH-4 被当成 BCH-3 用"，此时注入 4 bit 错必须失败 —— 若仍报成功，
 * 说明测试根本没有观测到纠正能力上限。
 */
#ifndef CHIPLAB_ECC_DECODE_STRENGTH
#define CHIPLAB_ECC_DECODE_STRENGTH	CHIPLAB_ECC_STRENGTH_DEFAULT
#endif

#define CHIPLAB_BCH_M		13
#define CHIPLAB_BCH_N		((1u << CHIPLAB_BCH_M) - 1u)	/* 8191 */
#define CHIPLAB_BCH_T		CHIPLAB_ECC_STRENGTH_DEFAULT
#define CHIPLAB_BCH_ECC_BITS	(CHIPLAB_BCH_M * CHIPLAB_BCH_T)	/* 52 */
#define CHIPLAB_BCH_PRIM_POLY	0x201bu	/* GF(2^13) 标准本原多项式 x^13+x^4+x^3+x+1 */

/* GF(2^13) 两级表 + 生成多项式（每个 ecc 上下文一份，栈上 / 静态各一）
 * a_log 按字段值直接索引（字段值 < 2^13），故需 2^13 项。
 */
struct chiplab_bch {
	uint16_t a_log[1u << CHIPLAB_BCH_M];	/* [x] = log_α(x)，x < 8192 */
	uint16_t a_pow[CHIPLAB_BCH_N * 2];	/* [i] = α^(i mod 8191) */
	uint16_t g[CHIPLAB_BCH_ECC_BITS + 1];	/* degree ecc_bits，g[ecc_bits]=1 */
};

static inline uint16_t chiplab_gf_pow(const struct chiplab_bch *b, unsigned i)
{
	return b->a_pow[i % CHIPLAB_BCH_N];
}

static inline int chiplab_gf_log(const struct chiplab_bch *b, uint16_t v)
{
	return (int)b->a_log[v & CHIPLAB_BCH_N];
}

static inline uint16_t chiplab_gf_mul(const struct chiplab_bch *b, uint16_t x, uint16_t y)
{
	int lx, ly;

	if (x == 0 || y == 0)
		return 0;
	lx = chiplab_gf_log(b, x);
	ly = chiplab_gf_log(b, y);
	return chiplab_gf_pow(b, (unsigned)(lx + ly));
}

/*
 * 多项式求值：coeff[0..n] **升幂**（coeff[k] 是 x^k 的系数，均存为 GF(2^13) 元素）。
 * 与 struct chiplab_bch.g[] / chiplab_bch_minimal_poly() 的输出约定一致。
 */
static inline uint16_t chiplab_bch_poly_eval(const struct chiplab_bch *b,
					     const uint16_t *coeff, unsigned n, uint16_t x)
{
	uint16_t acc = 0;
	int i;

	for (i = (int)n; i >= 0; i--)
		acc = (uint16_t)(chiplab_gf_mul(b, acc, x) ^ coeff[i]);
	return acc;
}

/*
 * 求 α^i 的极小多项式（GF(2) 系数）。
 * 输出 out[0..deg] **升幂**（out[k] = x^k 系数，out[deg] 恒为 1），return deg。
 * 调用者须提供 ≥ M+2 个元素的缓冲区（out[0..M+1]）。
 * 自检：out[deg]==1 且 out(α^i)==0，否则返回负值（fail-closed）。
 */
static int chiplab_bch_minimal_poly(const struct chiplab_bch *b, unsigned i,
				    uint16_t *out)
{
	uint16_t conj[CHIPLAB_BCH_M];
	uint16_t tmp[CHIPLAB_BCH_M + 2];
	uint16_t poly[CHIPLAB_BCH_M + 2];
	unsigned nconj = 0, deg, j, k, e, x0;

	/*
	 * 迹共轭集合 = {α^i, α^{2i}, α^{4i}, ...}，**包含 α^i 自身**：
	 * 先取 x = α^i，再反复平方（GF(2) 上特征为 2，故 α^{2i} = (α^i)^2）
	 * 直到回到集合内元素为止。
	 */
	e = i % CHIPLAB_BCH_N;
	x0 = (e == 0) ? 1u : b->a_pow[e];
	for (j = 0; j < CHIPLAB_BCH_M; j++) {
		unsigned x;
		int dup = 0;

		x = (j == 0) ? x0 : ((e == 0) ? 1u : b->a_pow[e]);
		for (k = 0; k < nconj; k++)
			if (conj[k] == (uint16_t)x)
				dup = 1;
		if (dup)
			break;
		conj[nconj++] = (uint16_t)x;
		e = (e * 2u) % CHIPLAB_BCH_N;
	}
	/* poly = 1，随后逐个乘 (x + α^{2^j})；系数下标 k = x^k 系数 */
	for (j = 0; j < CHIPLAB_BCH_M + 2u; j++)
		poly[j] = 0;
	poly[0] = 1;
	deg = 0;
	for (k = 0; k < nconj; k++) {
		unsigned jj2;

		if (conj[k] == 0)
			continue;
		for (jj2 = 0; jj2 <= deg + 1u; jj2++)
			tmp[jj2] = 0;
		for (jj2 = 0; jj2 <= deg; jj2++) {
			tmp[jj2] ^= chiplab_gf_mul(b, poly[jj2], conj[k]);
			tmp[jj2 + 1] ^= poly[jj2];
		}
		deg++;
		/* 必须拷到 deg（含新首项），否则会丢掉最高次系数 */
		for (jj2 = 0; jj2 <= deg; jj2++)
			poly[jj2] = tmp[jj2];
	}
	for (j = 0; j < CHIPLAB_BCH_M + 2u; j++)
		out[j] = 0;
	for (j = 0; j <= deg; j++)
		out[j] = poly[j];
	/* 自检：首项为 1 且以 α^i 为根 */
	if (deg == 0 || out[deg] != 1)
		return -1;
	if (chiplab_bch_poly_eval(b, out, deg, b->a_pow[i % CHIPLAB_BCH_N]) != 0)
		return -2;
	return (int)deg;
}

/* 由 α^1..α^{2t} 的极小多项式之积构造生成多项式 g(x)（deg = ecc_bits） */
static int chiplab_bch_init(struct chiplab_bch *b)
{
	uint16_t minp[CHIPLAB_BCH_M + 2];
	/* 卷积中间结果次数可达 deg+md（上限 ecc_bits+M），故按 M 加宽缓冲区 */
	uint16_t next[CHIPLAB_BCH_ECC_BITS + CHIPLAB_BCH_M + 2];
	uint16_t dist[2 * CHIPLAB_BCH_T + 1][CHIPLAB_BCH_M + 2];
	unsigned dist_deg[2 * CHIPLAB_BCH_T + 1];
	unsigned ndist = 0;
	unsigned deg, i, j, k;
	uint32_t x = 1;

	for (i = 0; i < CHIPLAB_BCH_N; i++) {
		b->a_pow[i] = (uint16_t)x;
		b->a_log[x] = (uint16_t)i;
		x <<= 1;
		if (x & (1u << CHIPLAB_BCH_M))
			x ^= CHIPLAB_BCH_PRIM_POLY;
	}
	for (i = CHIPLAB_BCH_N; i < CHIPLAB_BCH_N * 2; i++)
		b->a_pow[i] = b->a_pow[i - CHIPLAB_BCH_N];
	b->a_log[0] = 0;	/* 未定义，禁止使用 */

	/* g = 1 */
	for (j = 0; j <= CHIPLAB_BCH_ECC_BITS; j++)
		b->g[j] = 0;	/* 含 j == ecc_bits：置 0，避免读到未初始化值 */
	b->g[0] = 1;
	deg = 0;
	for (i = 1; i <= 2 * CHIPLAB_BCH_T; i++) {
		int md = chiplab_bch_minimal_poly(b, i, minp);

		if (md <= 0)
			return -1;
		/* 同一极小多项式可能被多个 α^i 共用（如 α、α²、α⁴、α⁸），
		 * 重复乘入会得到 g^2，次数翻倍 —— 必须按多项式去重。 */
		if (md > (int)CHIPLAB_BCH_M + 1)
			return -1;
		for (k = 0; k < ndist; k++) {
			int same = (dist_deg[k] == (unsigned)md);

			for (j = 0; same && j <= (unsigned)md; j++)
				if (dist[k][j] != minp[j])
					same = 0;
			if (same)
				break;
		}
		if (k < ndist)
			continue;	/* 本幂次的极小多项式已并入 g */
		if (ndist >= 2 * CHIPLAB_BCH_T + 1u)
			return -1;
		for (j = 0; j <= (unsigned)md; j++)
			dist[ndist][j] = minp[j];
		for (j = (unsigned)md + 1u; j < CHIPLAB_BCH_M + 2u; j++)
			dist[ndist][j] = 0;
		dist_deg[ndist] = (unsigned)md;
		ndist++;

		if (deg + (unsigned)md > CHIPLAB_BCH_ECC_BITS)
			return -1;	/* 次数超界：参数不匹配，fail-closed */
		/*
		 * next = g * minp（稀疏卷积，只用到 minp[0..md]，其中 minp[md]==1）。
		 * 写入范围恒为 j+md ≤ deg+md ≤ ecc_bits（上方已校验），不会越界。
		 */
		for (j = 0; j < CHIPLAB_BCH_ECC_BITS + CHIPLAB_BCH_M + 2u; j++)
			next[j] = 0;
		for (j = 0; j <= deg; j++) {
			unsigned k2;

			for (k2 = 0; k2 <= (unsigned)md; k2++)
				next[j + k2] ^= chiplab_gf_mul(b, b->g[j], minp[k2]);
		}
		deg += (unsigned)md;
		for (j = 0; j <= deg; j++)
			b->g[j] = next[j];
		for (j = deg + 1u; j <= CHIPLAB_BCH_ECC_BITS; j++)
			b->g[j] = 0;
	}
	if (ndist != CHIPLAB_BCH_T)
		return -1;	/* BCH(2t) 的 g 必须由 t 个互异极小多项式构成 */
	if (deg != CHIPLAB_BCH_ECC_BITS)
		return -1;
	if (b->g[CHIPLAB_BCH_ECC_BITS] != 1)
		return -1;
	/* 自检：g(α^i) 必须为 0，i = 1..2t */
	for (i = 1; i <= 2 * CHIPLAB_BCH_T; i++)
		if (chiplab_bch_poly_eval(b, b->g, CHIPLAB_BCH_ECC_BITS,
					  b->a_pow[i]) != 0)
			return -2;
	return 0;
}

/*
 * ---- BCH 编解码实现 ----
 *
 * 位序约定（**改动即破坏已写入数据**，单元测试 test_bit_order 锁死）：
 *   data byte i 的 bit j ↔ 码字位 pos = 8*i + j        （pos 即多项式幂次）
 *   ECC  byte e 的 bit j ↔ 码字位 pos = 8*nbytes + 8*e + j，仅前 52 位有效
 *   （ecc_bytes = 7，前 52 位用满，后 4 位固定为 0）
 *
 * 编码：V(x) = M(x)·x^52 + R(x)，R(x) = M(x)·x^52 mod g(x)
 * 解码：S_i = V(α^i)（i = 1..2t）应全为 0；非零则 Berlekamp-Massey 求
 *       错误定位多项式 Λ(x)，Chien 搜索定位错误位并翻转。
 *
 * 原语：
 */

/* 翻转码字位（p < 8·nbytes 落数据，否则落 ECC 的 x^(p-dbase) 系数） */
static void chiplab_bch_cw_flip(uint8_t *data, unsigned nbytes,
				uint8_t *ecc, unsigned p)
{
	unsigned dbase = nbytes * 8u;

	if (p < dbase) {
		data[p >> 3] ^= (uint8_t)(1u << (p & 7u));
		return;
	}
	p -= dbase;
	if (p < CHIPLAB_BCH_ECC_BITS)
		ecc[p >> 3] ^= (uint8_t)(1u << (p & 7u));
}

/* 2t 个 syndrome：S_i = V(α^i)，i = 1..2t（按上面的唯一定义） */
static void chiplab_bch_syndromes(const struct chiplab_bch *b,
				  const uint8_t *data, unsigned nbytes,
				  const uint8_t *ecc, unsigned ecc_bytes,
				  uint16_t *s)
{
	unsigned dbase = nbytes * 8u;
	unsigned i, p, k;

	if (nbytes == 0 || dbase + CHIPLAB_BCH_ECC_BITS > CHIPLAB_BCH_N ||
	    ecc_bytes * 8u < CHIPLAB_BCH_ECC_BITS) {
		for (i = 0; i < 2 * CHIPLAB_BCH_T; i++)
			s[i] = 0;
		return;
	}

	for (i = 0; i < 2 * CHIPLAB_BCH_T; i++) {
		uint16_t acc = 0;

		for (p = 0; p < dbase; p++) {
			unsigned bit = (data[p >> 3] >> (p & 7u)) & 1u;

			if (bit)
				acc ^= chiplab_gf_pow(b, ((i + 1u) * p) % CHIPLAB_BCH_N);
		}
		for (k = 0; k < CHIPLAB_BCH_ECC_BITS; k++) {
			unsigned bit = (ecc[k >> 3] >> (k & 7u)) & 1u;

			if (bit)
				acc ^= chiplab_gf_pow(b, ((i + 1u) * (dbase + k)) % CHIPLAB_BCH_N);
		}
		s[i] = acc;
	}
}

/*
 * 计算 ECC：解 2·ability 个方程 V(α^i) = 0（i = 1..2·ability）得到 52 个
 * ECC 位。ability 是**纠错能力上限的唯一开关**：调小它会让 ECC 只满足更弱
 * 的根条件，于是注入 >ability 位错误时解码必须报失败（反证实验用）。
 */
/*
 * 计算 ECC。位序约定（与 chiplab_bch_syndromes 完全一致）：
 *   V(x) = Σ_{p=0}^{8n-1} b_p·x^p + Σ_{k=0}^{51} E_k·x^(8n+k)
 * 编码即求解 V(α^s) = 0（s = 1..2·ability）得到 52 个 ECC 位。
 *
 * 实现：把每个 syndrome 方程按 GF(2^13) 拆成 13 个 GF(2) 位方程，
 * 得到 13·2·ability 个位方程、52 个未知位，用 GF(2) RREF 求解。
 *   - ability == t（默认）→ 完整的 BCH-t 码字，能纠 t 位；
 *   - ability <  t → **只满足更弱的根条件**的码字。这不只是"能力更小"，
 *     它同时被用作反证实验的驱动器：把能力调小后，注入超过该能力的错误
 *     必须解码失败（fail-closed），否则说明"能报错"只是运气。
 */
static int chiplab_bch_encode_t(const struct chiplab_bch *b,
				const uint8_t *data, unsigned nbytes,
				uint8_t *ecc, unsigned ecc_bytes,
				unsigned ability)
{
	uint32_t rows[CHIPLAB_BCH_M * 2 * CHIPLAB_BCH_T][2];
	unsigned dbase, neq, nrow, i, j, k, r, piv, row;

	if (nbytes == 0 || nbytes * 8u + CHIPLAB_BCH_ECC_BITS > CHIPLAB_BCH_N)
		return -1;
	if (ability == 0 || ability > CHIPLAB_BCH_T)
		return -1;
	if (ecc_bytes < (CHIPLAB_BCH_ECC_BITS + 7u) / 8u)
		return -1;

	dbase = nbytes * 8u;
	neq = 2u * ability;
	nrow = 0;

	for (i = 0; i < neq; i++) {
		uint16_t cc = 0, coef[CHIPLAB_BCH_ECC_BITS];
		unsigned s = i + 1u, p, bit;

		for (k = 0; k < CHIPLAB_BCH_ECC_BITS; k++)
			coef[k] = chiplab_gf_pow(b, (s * (dbase + k)) % CHIPLAB_BCH_N);
		for (p = 0; p < dbase; p++) {
			if ((data[p >> 3] >> (p & 7u)) & 1u)
				cc ^= chiplab_gf_pow(b, (s * p) % CHIPLAB_BCH_N);
		}

		for (bit = 0; bit < CHIPLAB_BCH_M; bit++) {
			uint32_t lo = 0, hi = 0;

			for (k = 0; k < CHIPLAB_BCH_ECC_BITS; k++) {
				if (!((coef[k] >> bit) & 1u))
					continue;
				if (k < 32)
					lo |= 1u << k;
				else
					hi |= 1u << (k - 32);
			}
			if ((cc >> bit) & 1u)
				hi |= 1u << (CHIPLAB_BCH_ECC_BITS - 32);	/* RHS 位 */
			rows[nrow][0] = lo;
			rows[nrow][1] = hi;
			nrow++;
		}
	}

	r = 0;
	for (k = 0; k < CHIPLAB_BCH_ECC_BITS && r < nrow; k++) {
		uint32_t rlo, rhi;

		row = nrow;
		for (piv = r; piv < nrow; piv++) {
			uint32_t bv = (k < 32) ? (rows[piv][0] >> k)
					       : (rows[piv][1] >> (k - 32));

			if (bv & 1u) {
				row = piv;
				break;
			}
		}
		if (row == nrow)
			continue;	/* 自由列 */
		{
			uint32_t t0 = rows[r][0], t1 = rows[r][1];

			rows[r][0] = rows[row][0];
			rows[r][1] = rows[row][1];
			rows[row][0] = t0;
			rows[row][1] = t1;
		}
		rlo = rows[r][0];
		rhi = rows[r][1];
		for (piv = 0; piv < nrow; piv++) {
			uint32_t bv;

			if (piv == r)
				continue;
			bv = (k < 32) ? (rows[piv][0] >> k) : (rows[piv][1] >> (k - 32));
			if (bv & 1u) {
				rows[piv][0] ^= rlo;
				rows[piv][1] ^= rhi;
			}
		}
		r++;
	}

	for (i = 0; i < ecc_bytes; i++)
		ecc[i] = 0;
	/* 唯一解：每个未知位由 RREF 中该列的单行给出 */
	for (i = 0; i < r; i++) {
		uint32_t lo = rows[i][0], hi = rows[i][1];
		unsigned rhs = (hi >> (CHIPLAB_BCH_ECC_BITS - 32)) & 1u;

		if (!rhs)
			continue;
		for (k = 0; k < CHIPLAB_BCH_ECC_BITS; k++) {
			uint32_t bv = (k < 32) ? (lo >> k) : (hi >> (k - 32));

			if (bv & 1u) {
				ecc[k >> 3] |= (uint8_t)(1u << (k & 7u));
				break;
			}
		}
	}
	(void)j;
	return 0;
}

static int chiplab_bch_encode(const struct chiplab_bch *b,
			      const uint8_t *data, unsigned nbytes,
			      uint8_t *ecc, unsigned ecc_bytes)
{
	return chiplab_bch_encode_t(b, data, nbytes, ecc, ecc_bytes,
				    CHIPLAB_BCH_T);
}

/*
 * 解码：返回纠正的 bit 数（0 = 无错）；负数 = 失败。
 * **fail-closed**：超出纠正能力一律返回负值，绝不静默返回 0；
 * 修正后仍有余 syndrome 时回滚原始数据后返回负值。
 */
static int chiplab_bch_decode(const struct chiplab_bch *b,
			      uint8_t *data, unsigned nbytes,
			      uint8_t *ecc, unsigned ecc_bytes)
{
	uint16_t s[2 * CHIPLAB_BCH_T];
	uint16_t C[CHIPLAB_BCH_T + 1], B[CHIPLAB_BCH_T + 1];
	uint16_t T[CHIPLAB_BCH_T + 1];
	int L = 0, m = 1, nerr = 0;
	unsigned i, j, nz = 0, nbits;
	uint16_t bloc = 1;
	unsigned errpos[CHIPLAB_BCH_T];

	if (nbytes == 0 || nbytes * 8u + CHIPLAB_BCH_ECC_BITS > CHIPLAB_BCH_N)
		return -1;
	if (ecc_bytes < (CHIPLAB_BCH_ECC_BITS + 7u) / 8u)
		return -1;

	nbits = nbytes * 8u + CHIPLAB_BCH_ECC_BITS;
	/*
	 * 参与判定的 syndrome 对数 = 解码侧能力上限（默认 = t = 4）。
	 * 调小它（反证实验）会放宽根条件：既可能把超限错误当"无错"放过，
	 * 也会让 BM 找不到正确定位多项式 —— 两种情况测试都必须变红。
	 */
	if (CHIPLAB_ECC_DECODE_STRENGTH == 0 ||
	    CHIPLAB_ECC_DECODE_STRENGTH > CHIPLAB_BCH_T)
		return -1;
	chiplab_bch_syndromes(b, data, nbytes, ecc, ecc_bytes, s);
	for (i = 0; i < 2u * CHIPLAB_ECC_DECODE_STRENGTH; i++)
		if (s[i])
			nz++;
	if (nz == 0)
		return 0;	/* 无错 */

	/*
	 * Berlekamp-Massey（标准形式，S 索引 0..2t-1 对应 S_1..S_{2t}）：
	 *   d = S[n] ^ Σ_{i=1..L} C[i]·S[n-i]
	 *   C(x) ← C(x) - (d/b)·x^m·B(x)
	 * 收敛后 C 即错误定位多项式 Λ(x)（C[0]=1），其根为 α^{-p}。
	 */
	for (i = 0; i <= CHIPLAB_BCH_T; i++)
		C[i] = B[i] = 0;
	C[0] = 1;
	B[0] = 1;

	for (i = 0; i < 2u * CHIPLAB_ECC_DECODE_STRENGTH; i++) {
		uint16_t d = 0;
		uint16_t coef;

		for (j = 0; j <= (unsigned)L; j++)
			d ^= chiplab_gf_mul(b, C[j], s[i - j]);

		if (d == 0) {
			m++;
			continue;
		}
		for (j = 0; j <= CHIPLAB_BCH_T; j++)
			T[j] = C[j];
		coef = chiplab_gf_mul(b, d,
				     chiplab_gf_pow(b, CHIPLAB_BCH_N - (unsigned)chiplab_gf_log(b, bloc)));
		for (j = 0; j + (unsigned)m <= CHIPLAB_BCH_T; j++)
			C[j + (unsigned)m] ^= chiplab_gf_mul(b, coef, B[j]);
		if (2 * L <= (int)i) {
			unsigned oldL = (unsigned)L;

			for (j = 0; j <= CHIPLAB_BCH_T; j++)
				B[j] = T[j];
			L = (int)i + 1 - (int)oldL;
			bloc = d;
			m = 1;
		} else {
			m++;
		}
	}
	if (L > CHIPLAB_BCH_T)
		return -(L);

	/* Chien 搜索：位 p 出错 ⇔ Λ(α^{-p}) = 0（α^{-p} = α^{n-p}） */
	for (i = 0; i < nbits; i++) {
		uint16_t v = 0;

		for (j = 0; j <= (unsigned)L; j++)
			v ^= chiplab_gf_mul(b, C[j],
					    chiplab_gf_pow(b, j * ((unsigned)CHIPLAB_BCH_N - i)));
		if (v == 0) {
			if (nerr >= CHIPLAB_BCH_T)
				return -(CHIPLAB_BCH_T + 1);
			errpos[nerr++] = i;
		}
	}
	if (nerr != L)
		return -(CHIPLAB_BCH_T + 1);

	for (i = 0; i < (unsigned)nerr; i++)
		chiplab_bch_cw_flip(data, nbytes, ecc, errpos[i]);

	/* 修正后必须全 syndrome 归零；否则回滚并判为不可纠正 */
	chiplab_bch_syndromes(b, data, nbytes, ecc, ecc_bytes, s);
	for (i = 0; i < 2u * CHIPLAB_ECC_DECODE_STRENGTH; i++) {
		if (s[i]) {
			for (j = 0; j < (unsigned)nerr; j++)
				chiplab_bch_cw_flip(data, nbytes, ecc, errpos[j]);
			return -(CHIPLAB_BCH_T + 1);
		}
	}
	return nerr;
}

#endif /* __CHIPLAB_NAND_SHARED_H__ */
