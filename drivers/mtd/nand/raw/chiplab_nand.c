// SPDX-License-Identifier: GPL-2.0-only
/*
 * chiplab (Loongson Artix-7 实验箱) APB NAND 控制器驱动 —— Linux 侧
 *
 * 控制器：chiplab/IP/APB_DEV/NAND/nand.v（NAND_top）+ 平台 DMA（IP/DMA/dma.v）
 * 芯片：K9F1G08U0C（128 MiB，页 2048+64 B，块 128 KiB = 64 页，1024 块）
 *
 * 口径（**与 U-Boot 侧逐位/逐字节同源**）：
 *   u-boot/drivers/mtd/nand/raw/chiplab_nand.c   控制器层（寄存器 / DMA / 命令序列）
 *   u-boot/drivers/mtd/nand/raw/chiplab_nand.h   常量表 + 自包含 BCH-4（本目录 chiplab_bch.h 为其逐字拷贝）
 *   rv32gc-cpu/docs/porting/04-nand-driver.md    寄存器语义 / ECC 布局契约 / 分区口径
 *
 * ECC 契约（04-nand-driver.md §4.3，**两侧必须落盘一致**）：
 *   4 段 × 512 B，每段 7 B BCH-4，放备用区 offset 36..63；
 *   坏块标记 offset 0（出厂）/1（运行时）；oobfree = offset 2..35（34 B）。
 *
 * Linux 7.3 用现代接口：nand_controller_ops.exec_op + 驱动自带 read_page/
 * write_page（避免框架 NAND_ECC_ENGINE_TYPE_SOFT 分支改写 calculate/correct，
 * 见 nand_base.c:nand_set_ecc_soft_ops()——与 U-Boot 侧同一坑，处置方式相同：
 * nand_scan() 之后重新装回自包含 BCH-4 与布局）。
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>

#include "chiplab_bch.h"

/* ------------------------------------------------------------------ */
/* 控制器寄存器（nand.v；与 chiplab_nand.h §1 逐条对齐）                */
/* ------------------------------------------------------------------ */
#define CHIPLAB_NAND_REG_CMD		0x00u
#define CHIPLAB_NAND_REG_ADDRL		0x04u
#define CHIPLAB_NAND_REG_ADDRH		0x08u
#define CHIPLAB_NAND_REG_TIMING		0x0cu
#define CHIPLAB_NAND_REG_IDL		0x10u
#define CHIPLAB_NAND_REG_STATUS_IDH	0x14u
#define CHIPLAB_NAND_REG_PARAM		0x18u
#define CHIPLAB_NAND_REG_OP_NUM		0x1cu
#define CHIPLAB_NAND_REG_CE_MAP0	0x20u
#define CHIPLAB_NAND_REG_RDY_MAP0	0x28u
#define CHIPLAB_NAND_REG_DMA_ACK	0x40u

#define CHIPLAB_CMD_VALID		BIT(0)
#define CHIPLAB_CMD_READ		BIT(1)
#define CHIPLAB_CMD_WRITE		BIT(2)
#define CHIPLAB_CMD_ERASE		BIT(3)
#define CHIPLAB_CMD_READ_ID		BIT(5)
#define CHIPLAB_CMD_RESET		BIT(6)
#define CHIPLAB_CMD_READ_STATUS		BIT(7)
#define CHIPLAB_CMD_OP_MAIN		BIT(8)
#define CHIPLAB_CMD_OP_SPARE		BIT(9)
#define CHIPLAB_CMD_DONE		BIT(10)
#define CHIPLAB_CMD_DMA_REQ		BIT(31)

#define CHIPLAB_NAND_PARAM_OP_SCOPE(x)	(((x) & 0x3fffu) << 16)
#define CHIPLAB_NAND_PARAM_ID_NUM(x)	(((x) & 0x7u) << 12)
#define CHIPLAB_NAND_PARAM_SIZE(x)	(((x) & 0xfu) << 8)
#define CHIPLAB_NAND_SIZE_1GBIT		3u
#define CHIPLAB_NAND_TIMING_RESET	0x0412u
#define CHIPLAB_NAND_ID_BYTES		5u

/* DMA：confreg order 寄存器 + NAND 门铃（chiplab_nand.h §4） */
#define CHIPLAB_DMA_ORDER_OFF		0x1160u
#define CHIPLAB_DMA_ORDER_ASK		BIT(2)
#define CHIPLAB_DMA_ORDER_START		BIT(3)
#define CHIPLAB_DMA_ORDER_ADDR_MASK	0xffffffe0u
#define CHIPLAB_DMA_CMD_RW		BIT(12)
#define CHIPLAB_DMA_NAND_DEV_ADDR	0x1fe78040u
#define CHIPLAB_DMA_DESC_WORDS		8u
#define CHIPLAB_DMA_DESC_BYTES		32u

#define CHIPLAB_NAND_TIMEOUT_US		500000u

struct chiplab_nand {
	struct nand_controller base;
	struct nand_chip chip;
	struct device *dev;
	void __iomem *regs;
	void __iomem *confreg;

	u32 *dma_buf;			/* 32 B 描述符 + 2112 B 数据 */
	dma_addr_t dma_phys;

	u8 frame[CHIPLAB_NAND_PAGE_SPARE];
	u32 frame_len;
	u32 page;			/* SEQIN 锁存的页号 */
	u8 status;
	u64 id_inform;
};

static inline struct chiplab_nand *to_chiplab(struct nand_chip *chip)
{
	return container_of(chip, struct chiplab_nand, chip);
}

static inline u32 cnand_rd(struct chiplab_nand *p, u32 off)
{
	return readl(p->regs + off);
}

static inline void cnand_wr(struct chiplab_nand *p, u32 off, u32 v)
{
	writel(v, p->regs + off);
}

/* 等待控制器 DONE（nand.v:635/952/1143） */
static int cnand_wait_done(struct chiplab_nand *p)
{
	u32 cmd;

	return readl_poll_timeout(p->regs + CHIPLAB_NAND_REG_CMD, cmd,
				  cmd & CHIPLAB_CMD_DONE, 1,
				  CHIPLAB_NAND_TIMEOUT_US) ? -ETIMEDOUT : 0;
}

/*
 * 平台 DMA：CPU 只写 confreg order 寄存器，引擎自行取描述符搬运
 * （chiplab_nand.h §8 的 4×64b 描述符；cmd[12]=dma_r_w 只决定 DDR 侧方向）。
 * QEMU 模型在 order 写命中时同步完成搬运，故轮询立即返回。
 */
static int cnand_dma(struct chiplab_nand *p, u32 len, bool to_device)
{
	u32 *desc = p->dma_buf;
	u32 data_phys = (u32)(p->dma_phys + CHIPLAB_DMA_DESC_BYTES);
	u32 order;
	int i;

	if (!len || (len & 3u))
		return -EINVAL;

	for (i = 0; i < CHIPLAB_DMA_DESC_WORDS; i++)
		desc[i] = 0;
	desc[0] = 0;					/* order_addr（未用） */
	desc[1] = data_phys;				/* mem_addr */
	desc[2] = CHIPLAB_DMA_NAND_DEV_ADDR;		/* dev_addr = 门铃 */
	desc[3] = len / 4u;				/* length（4 B 字数） */
	desc[4] = 0;					/* step_length */
	desc[5] = 1;					/* step_times 必须为 1 */
	desc[6] = to_device ? CHIPLAB_DMA_CMD_RW : 0;
	desc[7] = 0;
	wmb();

	cnand_wr(p, CHIPLAB_NAND_REG_DMA_ACK, 0);	/* 门铃 ack 清零 */
	writel((u32)(p->dma_phys & CHIPLAB_DMA_ORDER_ADDR_MASK) |
	       CHIPLAB_DMA_ORDER_START, p->confreg + CHIPLAB_DMA_ORDER_OFF);

	return readl_poll_timeout(p->confreg + CHIPLAB_DMA_ORDER_OFF, order,
				  !(order & (CHIPLAB_DMA_ORDER_START |
					     CHIPLAB_DMA_ORDER_ASK)),
				  1, CHIPLAB_NAND_TIMEOUT_US) ? -ETIMEDOUT : 0;
}

/* 控制器高层操作：READ / WRITE / ERASE（命令字位域见 nand.v:461-536） */
static int cnand_read_frame(struct chiplab_nand *p, u32 page)
{
	int ret;

	cnand_wr(p, CHIPLAB_NAND_REG_ADDRL, 0);
	cnand_wr(p, CHIPLAB_NAND_REG_ADDRH, page & 0xffffu);
	cnand_wr(p, CHIPLAB_NAND_REG_OP_NUM, CHIPLAB_NAND_PAGE_SPARE);
	cnand_wr(p, CHIPLAB_NAND_REG_CMD, CHIPLAB_CMD_READ | CHIPLAB_CMD_OP_MAIN |
		 CHIPLAB_CMD_OP_SPARE | CHIPLAB_CMD_VALID);
	ret = cnand_dma(p, CHIPLAB_NAND_PAGE_SPARE, false);
	if (ret)
		return ret;
	memset(p->frame, 0xff, sizeof(p->frame));
	memcpy(p->frame, p->dma_buf + CHIPLAB_DMA_DESC_BYTES / 4,
	       CHIPLAB_NAND_PAGE_SPARE);
	return 0;
}

static int cnand_program_frame(struct chiplab_nand *p, u32 page)
{
	memcpy(p->dma_buf + CHIPLAB_DMA_DESC_BYTES / 4, p->frame,
	       CHIPLAB_NAND_PAGE_SPARE);
	cnand_wr(p, CHIPLAB_NAND_REG_ADDRL, 0);
	cnand_wr(p, CHIPLAB_NAND_REG_ADDRH, page & 0xffffu);
	cnand_wr(p, CHIPLAB_NAND_REG_OP_NUM, CHIPLAB_NAND_PAGE_SPARE);
	cnand_wr(p, CHIPLAB_NAND_REG_CMD, CHIPLAB_CMD_WRITE | CHIPLAB_CMD_OP_MAIN |
		 CHIPLAB_CMD_OP_SPARE | CHIPLAB_CMD_VALID);
	return cnand_dma(p, CHIPLAB_NAND_PAGE_SPARE, true);
}

static int cnand_erase_block(struct chiplab_nand *p, u32 page)
{
	cnand_wr(p, CHIPLAB_NAND_REG_ADDRL, 0);
	cnand_wr(p, CHIPLAB_NAND_REG_ADDRH, page & 0xffffu);
	cnand_wr(p, CHIPLAB_NAND_REG_OP_NUM, 1);
	cnand_wr(p, CHIPLAB_NAND_REG_CMD, CHIPLAB_CMD_ERASE | CHIPLAB_CMD_VALID);
	return cnand_wait_done(p);
}

static void cnand_build_id(struct chiplab_nand *p)
{
	cnand_wr(p, CHIPLAB_NAND_REG_CMD, CHIPLAB_CMD_READ_ID | CHIPLAB_CMD_VALID);
	(void)cnand_wait_done(p);
	p->id_inform = (u64)(cnand_rd(p, CHIPLAB_NAND_REG_IDL)) |
		       ((u64)(cnand_rd(p, CHIPLAB_NAND_REG_STATUS_IDH) & 0xffffu) << 32);
}

static void cnand_read_status(struct chiplab_nand *p)
{
	cnand_wr(p, CHIPLAB_NAND_REG_CMD, CHIPLAB_CMD_READ_STATUS | CHIPLAB_CMD_VALID);
	(void)cnand_wait_done(p);
	p->status = (u8)(cnand_rd(p, CHIPLAB_NAND_REG_STATUS_IDH) >> 16);
}

/* 由 SEQIN 锁存页号、PAGEPROG 提交的帧编程（定义在 ECC 一节） */
static int chiplab_nand_commit_frame(struct chiplab_nand *p);

/* ------------------------------------------------------------------ */
/* exec_op：把框架发来的通用指令序列翻译成控制器高层命令                */
/* ------------------------------------------------------------------ */
static int chiplab_nand_exec_op(struct nand_chip *chip,
				const struct nand_operation *op, bool check_only)
{
	struct chiplab_nand *p = to_chiplab(chip);
	u8 opcode = 0;
	u32 col = 0, row = 0;
	bool frame_loaded = false;
	unsigned int i;
	int ret = 0;

	if (check_only)
		return 0;

	for (i = 0; i < op->ninstrs && !ret; i++) {
		const struct nand_op_instr *instr = &op->instrs[i];

		switch (instr->type) {
		case NAND_OP_CMD_INSTR:
			opcode = instr->ctx.cmd.opcode;
			switch (opcode) {
			case NAND_CMD_READ0:
			case NAND_CMD_READOOB:
			case NAND_CMD_READSTART:
				/*
				 * 大页读的框架序列是
				 *   CMD READ0(0x00) + ADDR + CMD READSTART(0x30) + WAITRDY + DATA_IN
				 * 即 DATA_IN 之前**最后**一条 CMD 是 0x30。早期实现只认
				 * READ0/READOOB，于是这种读落到 DATA_IN 的 default 分支，
				 * 被静默填成 0xFF（UBIFS 读索引节点就是这么坏的：数据其实
				 * 已在 NAND 上，读回却是 255 而且没有任何 ECC 报错）。
				 * frame_loaded 在这里一并复位，保证真正读一次器件。
				 */
				frame_loaded = false;
				break;
			case NAND_CMD_SEQIN:
				memset(p->frame, 0xff, sizeof(p->frame));
				p->frame_len = CHIPLAB_NAND_PAGE_SPARE;
				break;
			case NAND_CMD_PAGEPROG:
				/*
				 * 与 u-boot 侧 cnand_cmdfunc() 的 p->page 语义同构：
				 * 框架用 nand_prog_page_begin_op()(SEQIN+ADDR) 起页编程、
				 * nand_prog_page_end_op()(PAGEPROG，**不带地址**) 收尾，
				 * 所以页号必须在 SEQIN 拍锁存（见 ADDR 分支），
				 * PAGEPROG 拍用锁存值提交——早期实现完全忽略 PAGEPROG，
				 * 走到这条路径的写会静默丢弃（数据留在 p->frame 里）。
				 */
				ret = chiplab_nand_commit_frame(p);
				break;
			case NAND_CMD_ERASE2:
				ret = cnand_erase_block(p, row);
				break;
			case NAND_CMD_READID:
				cnand_build_id(p);
				break;
			case NAND_CMD_STATUS:
				cnand_read_status(p);
				break;
			case NAND_CMD_RESET:
				cnand_wr(p, CHIPLAB_NAND_REG_CMD,
					 CHIPLAB_CMD_RESET | CHIPLAB_CMD_VALID);
				ret = cnand_wait_done(p);
				break;
			default:
				break;
			}
			break;

		case NAND_OP_ADDR_INSTR: {
			unsigned int n = instr->ctx.addr.naddrs;
			const u8 *a = instr->ctx.addr.addrs;

			/*
			 * 周期数与字段划分（ONFI 约定，**由实际周期数决定**）：
			 *   5 周期（大页 + 3 行字节）：col[7:0] col[15:8] row[7:0] row[15:8] row[23:16]
			 *   4 周期（大页 + 2 行字节）：col[7:0] col[15:8] row[7:0] row[15:8]
			 *   4 周期（小页）            ：col[7:0]          row[7:0] row[15:8] row[23:16]
			 *   3 周期（擦除）            ：                  row[7:0] row[15:8] row[23:16]
			 * 本芯片 2048 B 页 / 65536 页 ⇒ 行地址 16 bit，框架发的是
			 * **4 周期 = 2 列 + 2 行**；早期实现把 4 周期一律当"1 列 + 3 行"，
			 * 于是 row 被算成 ((col>>8)) | (row_lo<<8) | (row_hi<<16)（例如
			 * 页 27138 变成 6947840，超出器件范围 → 读回全 0xFF），
			 * 而写路径用的是框架直接给的页号，所以"写对、读错"。
			 * 擦除的地址里**没有列地址**（早期实现无条件把 a[0]/a[1] 当列地址，
			 * 3 周期时 row 恒为 0，每次擦块 0——UBIFS 首次挂载即暴露）。
			 */
			if (opcode == NAND_CMD_ERASE1) {
				col = 0;
				row = (n >= 1 ? a[0] : 0) |
				      (n >= 2 ? (a[1] << 8) : 0) |
				      (n >= 3 ? (a[2] << 16) : 0);
			} else if (n >= 5) {
				col = a[0] | (a[1] << 8);
				row = a[2] | (a[3] << 8) | (a[4] << 16);
			} else if (n == 4) {
				if (nand_to_mtd(chip)->writesize > 512) {
					col = a[0] | (a[1] << 8);
					row = a[2] | (a[3] << 8);
				} else {
					col = a[0];
					row = a[1] | (a[2] << 8) | (a[3] << 16);
				}
			} else if (n == 3) {
				col = a[0];
				row = a[1] | (a[2] << 8);
			} else if (n == 2) {
				/* 只换列地址（nand_change_read_column_op / RNDOUT） */
				col = a[0] | (a[1] << 8);
			} else if (n >= 1) {
				col = a[0];
			}
			/* SEQIN 拍锁存页号，PAGEPROG 拍（无地址）用它提交 */
			if (opcode == NAND_CMD_SEQIN)
				p->page = row;
			/* col 是"页内"偏移：>=2048 落在备用区（与 U-Boot 同口径） */
			if (opcode == NAND_CMD_READOOB && col < CHIPLAB_NAND_PAGE_SIZE)
				col += CHIPLAB_NAND_PAGE_SIZE;
			break;
		}

		case NAND_OP_DATA_IN_INSTR: {
			u8 *buf = instr->ctx.data.buf.in;
			unsigned int len = instr->ctx.data.len;

			switch (opcode) {
			case NAND_CMD_READ0:
			case NAND_CMD_READSTART:
			case NAND_CMD_READOOB:
				if (!frame_loaded) {
					ret = cnand_read_frame(p, row);
					if (ret)
						break;
					frame_loaded = true;
					p->page = row;
				}
				if (col + len > CHIPLAB_NAND_PAGE_SPARE)
					len = CHIPLAB_NAND_PAGE_SPARE - col;
				memcpy(buf, p->frame + col, len);
				col += len;
				break;
			case NAND_CMD_RNDOUT:
				/*
				 * 只换列地址重读：帧在 READ0/READSTART 拍已整帧读入，
				 * 这里直接从 frame 取（框架 nand_read_subpage/
				 * nand_change_read_column_op 走这条）。
				 */
				if (!frame_loaded) {
					ret = cnand_read_frame(p, p->page);
					if (ret)
						break;
					frame_loaded = true;
				}
				if (col + len > CHIPLAB_NAND_PAGE_SPARE)
					len = CHIPLAB_NAND_PAGE_SPARE - col;
				memcpy(buf, p->frame + col, len);
				col += len;
				break;
			case NAND_CMD_READID: {
				unsigned int k;

				for (k = 0; k < len; k++) {
					if (k < 4)
						buf[k] = (u8)(p->id_inform >> (8 * k));
					else if (k < 6)
						buf[k] = (u8)(p->id_inform >> (8 * k));
					else
						buf[k] = 0x00;
				}
				break;
			}
			case NAND_CMD_STATUS:
				memset(buf, 0xff, len);
				if (len)
					buf[0] = p->status;
				break;
			default:
				/*
				 * fail-closed：不再"静默填 0xFF"，否则任何没翻译的
				 * 读都会变成"内容全 1 的空页"，上层的坏块/节点判据会
				 * 得到假的成功（UBIFS 索引节点读回 255 就是这么来的）。
				 */
				dev_warn(p->dev, "chiplab-nand: unhandled DATA_IN opcode 0x%02x (len %u)\n",
					 opcode, len);
				ret = -EINVAL;
				break;
			}
			break;
		}

		case NAND_OP_DATA_OUT_INSTR: {
			const u8 *buf = instr->ctx.data.buf.out;
			unsigned int len = instr->ctx.data.len;

			if (opcode == NAND_CMD_SEQIN) {
				if (col + len > CHIPLAB_NAND_PAGE_SPARE)
					len = CHIPLAB_NAND_PAGE_SPARE - col;
				memcpy(p->frame + col, buf, len);
				col += len;
			}
			break;
		}

		case NAND_OP_WAITRDY_INSTR:
			ret = cnand_wait_done(p);
			break;
		}
	}

	return ret;
}

/* ------------------------------------------------------------------ */
/* ECC：自包含 BCH-4 + 04-nand-driver.md §4.3 布局（与 U-Boot 同源）     */
/* ------------------------------------------------------------------ */
static struct chiplab_bch chiplab_bch;

static int chiplab_ecc_calculate(struct nand_chip *chip, const u8 *data, u8 *ecc)
{
	return chiplab_bch_encode(&chiplab_bch, data, CHIPLAB_NAND_ECC_STEP_SIZE,
				  ecc, CHIPLAB_NAND_ECC_BYTES_PER_STEP);
}

/*
 * PAGEPROG 拍提交：把 SEQIN 拍填好的 frame（数据 2048 + 备用区 64）按契约重算
 * ECC 后编程到锁存的页。与 chiplab_write_page() 的区别只是**不再重置备用区**，
 * 保留调用者（例如 nand_write_oob_std）已经写进 frame 的 OOB 字节。
 */
static int chiplab_nand_commit_frame(struct chiplab_nand *p)
{
	unsigned int i;

	for (i = 0; i < CHIPLAB_NAND_ECC_STEPS; i++)
		chiplab_ecc_calculate(&p->chip,
				      p->frame + i * CHIPLAB_NAND_ECC_STEP_SIZE,
				      p->frame + CHIPLAB_NAND_PAGE_SIZE +
				      CHIPLAB_NAND_ECC_FIRST_POS +
				      i * CHIPLAB_NAND_ECC_BYTES_PER_STEP);

	return cnand_program_frame(p, p->page);
}

/* 擦除态页判据：数据段+校验段里 0 的个数 ≤ 阈值即视为空白页 */
static bool chiplab_bch_is_erased(const u8 *data, const u8 *ecc)
{
	unsigned int zeros = 0, i;

	for (i = 0; i < CHIPLAB_NAND_ECC_STEP_SIZE; i++)
		zeros += 8 - hweight8(data[i]);
	for (i = 0; i < CHIPLAB_NAND_ECC_BYTES_PER_STEP; i++)
		zeros += 8 - hweight8(ecc[i]);

	return zeros <= CHIPLAB_NAND_ECC_STRENGTH;
}

static int chiplab_ecc_correct(struct nand_chip *chip, u8 *data, u8 *read_ecc,
			       u8 *calc_ecc)
{
	int ret;

	ret = chiplab_bch_decode(&chiplab_bch, data, CHIPLAB_NAND_ECC_STEP_SIZE,
				 read_ecc, CHIPLAB_NAND_ECC_BYTES_PER_STEP);
	if (ret >= 0)
		return ret;

	if (chiplab_bch_is_erased(data, read_ecc)) {
		memset(data, 0xff, CHIPLAB_NAND_ECC_STEP_SIZE);
		return 0;
	}

	return -EBADMSG;
}

static int chiplab_read_page(struct nand_chip *chip, u8 *buf, int oob_required,
			     int page)
{
	struct chiplab_nand *p = to_chiplab(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	unsigned int i, max_bitflips = 0;
	int ret;

	ret = cnand_read_frame(p, page);
	if (ret)
		return ret;

	memcpy(buf, p->frame, CHIPLAB_NAND_PAGE_SIZE);
	if (oob_required)
		memcpy(chip->oob_poi, p->frame + CHIPLAB_NAND_PAGE_SIZE,
		       CHIPLAB_NAND_OOBSIZE);

	for (i = 0; i < CHIPLAB_NAND_ECC_STEPS; i++) {
		u8 *d = buf + i * CHIPLAB_NAND_ECC_STEP_SIZE;
		u8 *e = p->frame + CHIPLAB_NAND_PAGE_SIZE +
			CHIPLAB_NAND_ECC_FIRST_POS +
			i * CHIPLAB_NAND_ECC_BYTES_PER_STEP;

		ret = chiplab_ecc_correct(chip, d, e, NULL);
		if (ret < 0) {
			mtd->ecc_stats.failed++;
			return ret;
		}
		mtd->ecc_stats.corrected += ret;
		max_bitflips = max(max_bitflips, (unsigned int)ret);
	}

	return max_bitflips;
}

static int chiplab_write_page(struct nand_chip *chip, const u8 *buf,
			      int oob_required, int page)
{
	struct chiplab_nand *p = to_chiplab(chip);
	unsigned int i;

	memcpy(p->frame, buf, CHIPLAB_NAND_PAGE_SIZE);
	/* 备用区：坏块标记 0/1 与 oobfree 2..35 保持 0xFF，ECC 写 36..63 */
	memset(p->frame + CHIPLAB_NAND_PAGE_SIZE, 0xff, CHIPLAB_NAND_OOBSIZE);
	if (oob_required)
		memcpy(p->frame + CHIPLAB_NAND_PAGE_SIZE, chip->oob_poi,
		       CHIPLAB_NAND_OOBSIZE);

	for (i = 0; i < CHIPLAB_NAND_ECC_STEPS; i++) {
		chiplab_ecc_calculate(chip, buf + i * CHIPLAB_NAND_ECC_STEP_SIZE,
				      p->frame + CHIPLAB_NAND_PAGE_SIZE +
				      CHIPLAB_NAND_ECC_FIRST_POS +
				      i * CHIPLAB_NAND_ECC_BYTES_PER_STEP);
	}

	return cnand_program_frame(p, page);
}

static int chiplab_read_page_raw(struct nand_chip *chip, u8 *buf,
				 int oob_required, int page)
{
	struct chiplab_nand *p = to_chiplab(chip);
	int ret = cnand_read_frame(p, page);

	if (ret)
		return ret;
	memcpy(buf, p->frame, CHIPLAB_NAND_PAGE_SIZE);
	if (oob_required)
		memcpy(chip->oob_poi, p->frame + CHIPLAB_NAND_PAGE_SIZE,
		       CHIPLAB_NAND_OOBSIZE);
	return 0;
}

static int chiplab_write_page_raw(struct nand_chip *chip, const u8 *buf,
				  int oob_required, int page)
{
	struct chiplab_nand *p = to_chiplab(chip);

	memcpy(p->frame, buf, CHIPLAB_NAND_PAGE_SIZE);
	if (oob_required)
		memcpy(p->frame + CHIPLAB_NAND_PAGE_SIZE, chip->oob_poi,
		       CHIPLAB_NAND_OOBSIZE);
	else
		memset(p->frame + CHIPLAB_NAND_PAGE_SIZE, 0xff,
		       CHIPLAB_NAND_OOBSIZE);

	return cnand_program_frame(p, page);
}

/* 备用区布局（唯一契约，与 U-Boot / Linux 侧同源） */
static int chiplab_ooblayout_ecc(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *r)
{
	if (section)
		return -ERANGE;
	r->offset = CHIPLAB_NAND_ECC_FIRST_POS;
	r->length = CHIPLAB_NAND_ECC_TOTAL_BYTES;
	return 0;
}

static int chiplab_ooblayout_free(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *r)
{
	if (section)
		return -ERANGE;
	r->offset = CHIPLAB_NAND_BBT_RUNTIME_OFF + 1;
	r->length = CHIPLAB_NAND_ECC_FIRST_POS - r->offset;
	return 0;
}

static const struct mtd_ooblayout_ops chiplab_ooblayout_ops = {
	.ecc = chiplab_ooblayout_ecc,
	.free = chiplab_ooblayout_free,
};

/* ------------------------------------------------------------------ */
/* probe                                                               */
/* ------------------------------------------------------------------ */
static int chiplab_nand_attach_chip(struct nand_chip *chip)
{
	struct chiplab_nand *p = to_chiplab(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int ret;

	cnand_wr(p, CHIPLAB_NAND_REG_TIMING, CHIPLAB_NAND_TIMING_RESET);
	cnand_wr(p, CHIPLAB_NAND_REG_PARAM,
		 CHIPLAB_NAND_PARAM_OP_SCOPE(CHIPLAB_NAND_PAGE_SIZE) |
		 CHIPLAB_NAND_PARAM_ID_NUM(CHIPLAB_NAND_ID_BYTES) |
		 CHIPLAB_NAND_PARAM_SIZE(CHIPLAB_NAND_SIZE_1GBIT));
	cnand_wr(p, CHIPLAB_NAND_REG_CE_MAP0, 0x88442200u);
	cnand_wr(p, CHIPLAB_NAND_REG_RDY_MAP0, 0x88442200u);
	cnand_read_status(p);

	ret = chiplab_bch_init(&chiplab_bch);
	if (ret) {
		dev_err(p->dev, "BCH init failed\n");
		return ret;
	}

	chip->options |= NAND_NO_SUBPAGE_WRITE;

	/*
	 * 必须先声明软 ECC 引擎（否则 nand_scan_tail() 落到 default 分支报
	 * "Invalid NAND_ECC_MODE 0"）；真正的 calculate/correct 与布局在
	 * nand_scan() 返回后用 chiplab_nand_restamp_ecc() 装回。
	 */
	chip->ecc.engine_type = NAND_ECC_ENGINE_TYPE_SOFT;
	chip->ecc.algo = NAND_ECC_ALGO_HAMMING;

	(void)mtd;
	return 0;
}

/*
 * 装回自包含 BCH-4 与布局契约。
 *
 * 框架的软 ECC 分支（nand_base.c:nand_set_ecc_soft_ops()，被 nand_scan_tail()
 * 调用）会**无条件**改写 ecc.calculate/correct 与 ecc.read_page/write_page，并把
 * bytes/strength 置成 Hamming-1 的值；attach_chip 回调又发生在它之前，所以只能
 * 在 nand_scan() 返回之后重新装回（与 U-Boot 侧同一处置）。
 */
static void chiplab_nand_restamp_ecc(struct chiplab_nand *p)
{
	struct nand_chip *chip = &p->chip;
	struct mtd_info *mtd = nand_to_mtd(chip);

	chip->ecc.engine_type = NAND_ECC_ENGINE_TYPE_SOFT;
	chip->ecc.size = CHIPLAB_NAND_ECC_STEP_SIZE;
	chip->ecc.steps = CHIPLAB_NAND_ECC_STEPS;
	chip->ecc.bytes = CHIPLAB_NAND_ECC_BYTES_PER_STEP;
	chip->ecc.total = CHIPLAB_NAND_ECC_TOTAL_BYTES;
	chip->ecc.strength = CHIPLAB_NAND_ECC_STRENGTH;
	chip->ecc.calculate = chiplab_ecc_calculate;
	chip->ecc.correct = chiplab_ecc_correct;
	chip->ecc.read_page = chiplab_read_page;
	chip->ecc.write_page = chiplab_write_page;
	chip->ecc.read_page_raw = chiplab_read_page_raw;
	chip->ecc.write_page_raw = chiplab_write_page_raw;
	chip->ecc.read_oob = nand_read_oob_std;
	chip->ecc.write_oob = nand_write_oob_std;

	/*
	 * **关掉框架的"子页读"**：nand_scan_tail() 见到 SOFT ECC + 大页
	 * (page_shift > 9) 会**自动**置 chip->options |= NAND_SUBPAGE_READ
	 * (nand_base.c:「Large page NAND with SOFT_ECC should support subpage
	 * reads」)，于是 nand_do_read_ops() 把所有"非整页"读都交给框架的
	 * nand_read_subpage()——它按**框架自己的** ECC 参数与 ooblayout 重新
	 * 取列地址读页数据/读备用区 ECC 字节，与本驱动的自包含 BCH-4 布局
	 * (4×512 B，ECC 在备用区 36..63) 不是一回事，短读一律拿回 0xFF：
	 *   · UBI 的 64 B EC 头/VID 头读 → 每个 PEB 都判成"空白" ⇒
	 *     "empty MTD device detected"、卷表读不到（user volume: 0）、
	 *     每次启动都重格式化 ⇒ 什么都存不住；
	 *   · UBIFS 的 48 B 索引节点读 → bad node type (255 but expected 9)。
	 * 整页读(dd bs=2048 / nanddump / nandwrite 回读)不走这条路，所以此前
	 * L5 交叉验证全绿却与 UBI/UBIFS 完全不兼容。
	 * 控制器本来就把 2112 B 整帧搬进 p->frame，短读没有任何收益，
	 * 直接让框架回到 ecc.read_page()（整页读 + 按请求长度拷贝）。
	 * NAND_HAS_SUBPAGE_READ() 只看 options 位，所以必须清位（只清函数
	 * 指针会变成空指针调用）。
	 */
	chip->options &= ~NAND_SUBPAGE_READ;
	chip->ecc.read_subpage = NULL;

	mtd_set_ooblayout(mtd, &chiplab_ooblayout_ops);
	/* 框架按旧（Hamming）布局算过 oobavail，这里按契约重算：64-28-2 = 34 */
	mtd->oobavail = CHIPLAB_NAND_OOBSIZE - CHIPLAB_NAND_ECC_TOTAL_BYTES - 2;
	mtd->ecc_strength = CHIPLAB_NAND_ECC_STRENGTH;
	mtd->ecc_step_size = CHIPLAB_NAND_ECC_STEP_SIZE;
	mtd->bitflip_threshold = DIV_ROUND_UP(CHIPLAB_NAND_ECC_STRENGTH * 3, 4);

	dev_info(p->dev, "chiplab-nand: ECC BCH-%u/%uB, layout %u..%u, bad-block marks %u/%u, oobavail %u\n",
		 chip->ecc.strength, chip->ecc.size,
		 CHIPLAB_NAND_ECC_FIRST_POS,
		 CHIPLAB_NAND_ECC_FIRST_POS + CHIPLAB_NAND_ECC_TOTAL_BYTES - 1,
		 CHIPLAB_NAND_BBT_FACTORY_OFF, CHIPLAB_NAND_BBT_RUNTIME_OFF,
		 mtd->oobavail);
}

static const struct nand_controller_ops chiplab_nand_controller_ops = {
	.attach_chip = chiplab_nand_attach_chip,
	.exec_op = chiplab_nand_exec_op,
};

static int chiplab_nand_probe(struct platform_device *pdev)
{
	struct chiplab_nand *p;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->dev = &pdev->dev;

	p->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->regs))
		return PTR_ERR(p->regs);

	/* confreg 里的平台 DMA order 寄存器（pdev 1 号资源，可选） */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		p->confreg = devm_ioremap_resource(&pdev->dev, res);
	} else {
		struct device_node *cnp;

		cnp = of_parse_phandle(np, "loongson,dma-order-region", 0);
		if (!cnp)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "missing DMA order register region\n");
		p->confreg = of_iomap(cnp, 0);
		of_node_put(cnp);
	}
	if (IS_ERR_OR_NULL(p->confreg))
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "cannot map DMA order register\n");

	p->dma_buf = dmam_alloc_coherent(&pdev->dev, 4096, &p->dma_phys,
					 GFP_KERNEL);
	if (!p->dma_buf)
		return -ENOMEM;

	chip = &p->chip;
	chip->controller = &p->base;
	chip->options = NAND_NO_SUBPAGE_WRITE;
	nand_controller_init(&p->base);
	p->base.ops = &chiplab_nand_controller_ops;

	mtd = nand_to_mtd(chip);
	mtd->dev.parent = &pdev->dev;

	ret = nand_scan(chip, 1);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "nand_scan failed\n");

	chiplab_nand_restamp_ecc(p);

	/*
	 * 让 MTD 层在 DT 里找 partitions 子节点（fixed-partitions，与 U-Boot
	 * mtdparts 同源）。MTD 设备的 fwnode 默认不从 parent 继承，需显式设置。
	 */
	device_set_node(&mtd->dev, of_fwnode_handle(np));

	cnand_build_id(p);
	dev_info(p->dev, "chiplab-nand: ID_INFORM = ec f1 00 1d 15? -> %02x %02x %02x %02x %02x\n",
		 (u8)(p->id_inform >> 0), (u8)(p->id_inform >> 8),
		 (u8)(p->id_inform >> 16), (u8)(p->id_inform >> 24),
		 (u8)(p->id_inform >> 32));

	ret = mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);
	if (ret) {
		nand_cleanup(chip);
		return dev_err_probe(&pdev->dev, ret, "mtd register failed\n");
	}

	dev_info(p->dev, "chiplab-nand: %llu MiB, page %u+%u B, erase %u B, ECC BCH-%u\n",
		 (unsigned long long)mtd->size >> 20, mtd->writesize,
		 mtd->oobsize, mtd->erasesize, chip->ecc.strength);
	return 0;
}

static void chiplab_nand_remove(struct platform_device *pdev)
{
	struct chiplab_nand *p = platform_get_drvdata(pdev);
	struct mtd_info *mtd = nand_to_mtd(&p->chip);

	mtd_device_unregister(mtd);
	nand_cleanup(&p->chip);
}

static const struct of_device_id chiplab_nand_dt_ids[] = {
	{ .compatible = "loongson,chiplab-nand" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, chiplab_nand_dt_ids);

static struct platform_driver chiplab_nand_driver = {
	.driver = {
		.name = "chiplab-nand",
		.of_match_table = chiplab_nand_dt_ids,
	},
	.probe = chiplab_nand_probe,
	.remove = chiplab_nand_remove,
};
module_platform_driver(chiplab_nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RV32-GC project");
MODULE_DESCRIPTION("Loongson chiplab APB NAND controller (K9F1G08U0C) with platform DMA");
