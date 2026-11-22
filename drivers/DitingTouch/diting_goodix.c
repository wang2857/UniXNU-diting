/* Goodix Berlin-D / Diting SM8475. Adapted from the earlier redwood_goodix.c
 * (APSL 2.0). Hardware resources are the Diting live DT, checked against the
 * NothingOSS cape pinctrl and waipio GCC source. No Redwood peripherals. */
#include <pexpert/arm64/board_config.h>
#if defined(DITING)
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/diting_input.h>
#include <kern/sched_prim.h>
#include <pexpert/device_tree.h>
#include <sys/systm.h>



extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern vm_offset_t ml_vtophys(vm_offset_t vaddr);
extern boolean_t ml_set_interrupts_enabled(boolean_t enable);
extern bool diting_smmu_lease_lock(void) __attribute__((weak_import));
extern void diting_smmu_lease_unlock(void) __attribute__((weak_import));

/* Shared RPMh voter from DitingPlatformResources.cpp (CONFIG_XNU_DITING_POWER).
 * Weak so a touch-only defconfig without the power driver still links. */
extern bool diting_rpmh_vote(uint32_t, uint32_t) __attribute__((weak_import));

static vm_offset_t tlmm, gcc, se, rsc, cmd_db;
static uint64_t tlmm_phys, gcc_phys, se_phys, rsc_phys, cmd_db_phys;
static uint32_t pins[6], gpio_offset, max_x, max_y;

static bool
property(DTEntry entry, const char *name, void *out, unsigned size)
{
    void *value = NULL; unsigned length = 0;
    if (SecureDTGetProperty(entry, name, &value, &length) != kSuccess || length != size) return false;
    memcpy(out, value, size); return true;
}

static bool
configuration(void)
{
    DTEntry entry;
    if (SecureDTLookupEntry(NULL, "/arm-io/diting-touch", &entry) != kSuccess) return false;
    return property(entry,"spi-base",&se_phys,8) && property(entry,"tlmm-base",&tlmm_phys,8) &&
        property(entry,"gcc-base",&gcc_phys,8) && property(entry,"rsc-base",&rsc_phys,8) &&
        property(entry,"cmd-db-base",&cmd_db_phys,8) && property(entry,"gpio-offset",&gpio_offset,4) &&
        property(entry,"pins",pins,sizeof(pins)) && property(entry,"max-x",&max_x,4) &&
        property(entry,"max-y",&max_y,4) && se_phys==0x990000 && tlmm_phys==0xf000000 &&
        gcc_phys==0x100000 && rsc_phys==0x17a20000 && cmd_db_phys==0x80860000 &&
        gpio_offset==0x100000 && pins[0]==16 && pins[1]==17 && pins[2]==18 &&
        pins[3]==19 && pins[4]==21 && pins[5]==20 && max_x!=0 && max_y!=0;
    /* max-x/max-y are only a fallback: goodix_probe() replaces them with the chip's
     * real IC_INFO axes (12200x27120 on this panel) before any report is scaled. */
}

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
	__asm__ volatile("dsb sy" ::: "memory");
}

static void
sleep_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static void
sleep_ms(uint32_t ms)
{
	uint64_t deadline;
	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	assert_wait_deadline((event_t)&cmd_db, THREAD_UNINT, deadline);
	thread_block(THREAD_CONTINUE_NULL);
}

/* ---- pins (TLMM) ------------------------------------------------------------------------------- */

#define TLMM_PHYS               tlmm_phys
#define TLMM_SIZE               0x200000UL
#define GPIO_CFG(n)             (tlmm + gpio_offset + 0x1000UL * (n))         /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(n)              (tlmm + gpio_offset + 0x1000UL * (n) + 4)     /* bit 0 in, bit 1 out */
#define PULL_NONE               0
#define PULL_UP                 3

#define PIN_MISO                pins[0]
#define PIN_MOSI                pins[1]
#define PIN_CLK                 pins[2]
#define PIN_CS                  pins[3]
#define PIN_INT                 pins[4]      /* low = the chip has a report */
#define PIN_RESET               pins[5]     /* low = in reset */
#define PIN_AVDD                71     /* goodix,avdd-gpio = <&tlmm 71 0>, active high */
#define PIN_FUNCTION_SPI        1       /* "qup15" */

static void
gpio_configure(unsigned pin, unsigned function, unsigned pull, unsigned ma, bool output)
{
	uint32_t value = pull | function << 2 | (ma / 2 - 1) << 6 | (output ? 1u << 9 : 0);
	wr(GPIO_CFG(pin), (rd(GPIO_CFG(pin)) & ~0x3ffu) | value);
}

static void
gpio_set(unsigned pin, bool high)
{
	wr(GPIO_IO(pin), high ? 2 : 0);
}

static bool
gpio_get(unsigned pin)
{
	return rd(GPIO_IO(pin)) & 1;
}

static void
gpio_output(unsigned pin, bool high, unsigned pull, unsigned ma)
{
	gpio_set(pin, high);            /* the level first, so that the pin never drives the wrong one */
	gpio_configure(pin, 0, pull, ma, true);
}

/* Actual cape TLMM summary SPI208 / GIC240, GPIO21 falling edge.
 * IRQ only acknowledges the pin and wakes this existing SPI owner. */
extern int armcpu_spi_attach(uint32_t,bool,void (*)(void *),void *,uint32_t *,uint64_t *) __attribute__((weak_import));
extern kern_return_t clear_wait(thread_t,wait_result_t);
static volatile uint32_t touch_pending,touch_irqs,touch_spurious;
static volatile uint64_t touch_irq_time;
static uint64_t touch_frame_time,touch_latency_total,touch_latency_max;
static uint32_t touch_latency_count;
static bool touch_irq_armed;
static void touch_irq(void *unused) {
    (void)unused;
    if(!(rd(GPIO_CFG(PIN_INT)+12)&1)){++touch_spurious;return;}
    wr(GPIO_CFG(PIN_INT)+12,0);
    uint64_t empty=0,stamp=mach_absolute_time();
    __atomic_compare_exchange_n(&touch_irq_time,&empty,stamp,false,__ATOMIC_RELEASE,__ATOMIC_RELAXED);
    __atomic_fetch_add(&touch_irqs,1,__ATOMIC_RELAXED);
    __atomic_store_n(&touch_pending,1,__ATOMIC_RELEASE);
    thread_wakeup((event_t)&touch_pending);
}
static bool arm_touch_irq(void) {
    if(!armcpu_spi_attach)return false;
    uint32_t saved=rd(GPIO_CFG(PIN_INT)+8);
    uint32_t cfg=(saved&~((7u<<5)|(3u<<2)|(1u<<1)|1u))|(3u<<5)|(1u<<2)|(1u<<4);
    wr(GPIO_CFG(PIN_INT)+8,cfg);wr(GPIO_CFG(PIN_INT)+12,0);
    /* no armcpu_spi_share on diting yet; SPI240 is unclaimed so exclusive
     * attach is the whole owner set. Port share when a second TLMM summary
     * consumer appears. */
    int rc=armcpu_spi_attach(240,false,touch_irq,NULL,NULL,NULL);
    if(rc){wr(GPIO_CFG(PIN_INT)+8,saved);return false;}
    wr(GPIO_CFG(PIN_INT)+8,cfg|1);return true;
}
static void touch_wait(unsigned ms) {
    if(__atomic_exchange_n(&touch_pending,0,__ATOMIC_ACQ_REL))return;
    uint64_t deadline;clock_interval_to_deadline(ms,NSEC_PER_MSEC,&deadline);
    assert_wait_deadline((event_t)&touch_pending,THREAD_UNINT,deadline);
    if(__atomic_exchange_n(&touch_pending,0,__ATOMIC_ACQ_REL))clear_wait(current_thread(),THREAD_AWAKENED);
    else thread_block(THREAD_CONTINUE_NULL);
}

/* ---- the I/O supply (RPMh) ----------------------------------------------------------------------- */

#define CMD_DB_PHYS             cmd_db_phys
#define CMD_DB_SIZE             0x20000UL
#define RSC_PHYS                rsc_phys    /* the application processor's "DRV 2" */
#define RSC_SIZE                0x10000UL

#define VRM_VOLTAGE             0               /* a regulator's registers, from its address: millivolts */
#define VRM_ENABLE              4
#define VRM_MODE                8
#define VRM_MODE_LDO_HPM        7

struct cmd_db_slave {
	uint16_t id, header_offset, data_offset, count, version, reserved[3];
};
struct cmd_db_header {
	uint32_t version;
	uint8_t magic[4];
	struct cmd_db_slave slave[8];
	uint32_t checksum, reserved;
	uint8_t data[];
};
struct cmd_db_entry {
	char id[8];
	uint32_t priority[2];
	uint32_t address;
	uint16_t length, offset;
};

/* The RPMh address of a resource by its name in the command database the boot loader left; 0 = none. */
static uint32_t
rpmh_address(const char *name)
{
	const struct cmd_db_header *db = (const struct cmd_db_header *)cmd_db;
	char id[8] = { 0 };

	if (db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		printf("goodix: no command database (magic %02x %02x %02x %02x)\n", db->magic[0], db->magic[1], db->magic[2],
		    db->magic[3]);
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &db->slave[s];
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);

		if (slave->id == 0 || slave->header_offset >= CMD_DB_SIZE - 0x1000 || slave->count > 512 ||
            144u + slave->header_offset + slave->count * sizeof(*entry) > CMD_DB_SIZE) {
			continue;
		}
		for (unsigned i = 0; i < slave->count; i++) {
			if (memcmp(entry[i].id, id, sizeof(id)) == 0) {
				return entry[i].address;
			}
		}
	}
	return 0;
}

/* Diting live DT (soc/spi@990000/xiaomi_touch@0):
 *   vdd-supply      = <&rpmh-regulator-ldoc2>        -> pm8350c_l2, 1.8 V (IOVDD)
 *   goodix,avdd-gpio = <&tlmm 0x47 0>                -> TLMM GPIO 71 (AVDD switch)
 * pm8350c_l3/ldoc3 is a 3.3 V always-on rail, not a 2.7 V touch AVDD; do not retune it.
 * Android's goodix_ts_power_on() enables the vdd regulator and then drives avdd-gpio. */
static bool
io_supply_on(void)
{
    uint32_t iovdd=rpmh_address("ldoc2");

    if (!iovdd || ((iovdd>>16)&15)!=4) {
        printf("goodix: IOVDD ldoc2 is not a PMIC-C resource in the command database (%#x)\n",iovdd);
        return false;
    }
    if (!diting_rpmh_vote || !diting_rpmh_vote(iovdd, 1800)) {
        printf("goodix: shared RPMh voter unavailable or IOVDD voltage vote failed (ldoc2=%#x)\n", iovdd);
        return false;
    }
    sleep_ms(3);
    if (!diting_rpmh_vote || !diting_rpmh_vote(iovdd + 4, 1)) {   /* VRM_ENABLE */
        printf("goodix: shared RPMh voter unavailable or IOVDD enable vote failed (ldoc2=%#x)\n", iovdd);
        return false;
    }
    if (!diting_rpmh_vote || !diting_rpmh_vote(iovdd + 8, 4)) {   /* init-mode the DT carries for pm8350c_l2 */
        printf("goodix: shared RPMh voter unavailable or IOVDD mode vote failed (ldoc2=%#x)\n", iovdd);
        return false;
    }
    sleep_ms(3);
    gpio_output(PIN_AVDD,true,PULL_NONE,6);      /* AVDD switch: active high */
    sleep_ms(10);
    printf("goodix: power on: IOVDD ldoc2=%#x at 1800 mV, mode 4; AVDD gpio %u=%u\n",
        iovdd,PIN_AVDD,gpio_get(PIN_AVDD));
    return true;
}

/* ---- clocks (GCC) -------------------------------------------------------------------------------- */

#define GCC_PHYS                gcc_phys
#define GCC_SIZE                0x70000UL
#define GCC_VOTE                (gcc + 0x62008)         /* the application processor's votes */
#define VOTE_QUP0_CORE_2X       (1u << 9)
#define VOTE_QUP0_CORE          (1u << 8)
#define VOTE_QUP0_M_AHB         (1u << 6)
#define VOTE_QUP0_S_AHB         (1u << 7)
#define VOTE_QUP0_SE4           (1u << 14)             /* Diting: wrapper 0 serial engine 4 */
#define GCC_QUP0_M_AHB_CBCR     (gcc + 0x27004)         /* bit 31: the clock is off */
#define GCC_QUP0_S_AHB_CBCR     (gcc + 0x27008)
#define GCC_QUP0_SE4_CBCR       (gcc + 0x274dc)
#define GCC_QUP0_SE4_CMD        (gcc + 0x274e4)         /* bit 0: take the new configuration */
#define GCC_QUP0_SE4_CFG        (gcc + 0x274e8)         /* [4:0] 2*divider-1, [10:8] source, [13:12] mode */
#define GCC_QUP0_SE4_DFSR       (gcc + 0x274f8)         /* bit 0: the engine chooses among preset rates itself */
#define GCC_QUP0_SE4_PERF_CFG(l) (gcc + 0x27500 + 4 * (l))
#define GCC_QUP0_SE4_PERF_M(l)  (gcc + 0x27540 + 4 * (l))
#define GCC_QUP0_SE4_PERF_N(l)  (gcc + 0x27580 + 4 * (l))
#define CLOCK_OFF               (1u << 31)
#define XO_HZ                   19200000

static uint32_t se_source_hz, se_clock_select;

static uint32_t
rcg_rate(uint32_t cfg, uint32_t m, uint32_t not_n_minus_m)
{
	static const uint32_t parent[8] = { XO_HZ, 600000000, 0, 0, 0, 0, 300000000, 0 };
	uint64_t hz = parent[cfg >> 8 & 7];
	uint32_t divider = cfg & 0x1f, mode = cfg >> 12 & 3;

	if (divider) {
		hz = hz * 2 / (divider + 1);
	}
	if (mode) {
		uint32_t n = (~not_n_minus_m & 0xffff) + (m & 0xffff);

		if (n) {
			hz = hz * (m & 0xffff) / n;
		}
	}
	return (uint32_t)hz;
}

/* BRANCH_HALT_VOTED under HWCG keeps CLK_OFF (bit 31) set while idle and sets
 * bit 1 (HW_CTL / NOC handshake) so the branch opens on a bus request. */
static bool
clock_ready(vm_offset_t reg)
{
	uint32_t v = rd(reg);
	return !(v & CLOCK_OFF) || (v & 2u);
}

static bool
clocks_on(uint32_t target_hz)
{
	wr(GCC_VOTE, rd(GCC_VOTE) | VOTE_QUP0_CORE_2X | VOTE_QUP0_CORE | VOTE_QUP0_M_AHB | VOTE_QUP0_S_AHB);
	se_source_hz = 0;
	se_clock_select = 0;
	if (rd(GCC_QUP0_SE4_DFSR) & 1) {
		uint32_t best_hi = 0, best_hi_sel = 0;
		for (uint32_t level = 0; level < 8; level++) {
			uint32_t hz = rcg_rate(rd(GCC_QUP0_SE4_PERF_CFG(level)), rd(GCC_QUP0_SE4_PERF_M(level)),
			    rd(GCC_QUP0_SE4_PERF_N(level)));
			if (!hz) {
				continue;
			}
			if (hz >= target_hz && (se_source_hz == 0 || hz < se_source_hz)) {
				se_source_hz = hz;
				se_clock_select = level;
			}
			if (hz > best_hi) {
				best_hi = hz;
				best_hi_sel = level;
			}
		}
		if (se_source_hz == 0 && best_hi > 0) {
			se_source_hz = best_hi;
			se_clock_select = best_hi_sel;
		}
		if (se_source_hz == 0) {
			printf("goodix: no usable preset clock rate\n");
			return false;
		}
	} else {
		/* the crystal, undivided: nothing else has to be running */
		wr(GCC_QUP0_SE4_CFG, (rd(GCC_QUP0_SE4_CFG) & ~0x371fu) | 1);
		wr(GCC_QUP0_SE4_CMD, rd(GCC_QUP0_SE4_CMD) | 1);
		for (int i = 0; i < 500 && (rd(GCC_QUP0_SE4_CMD) & 1); i++) {
			sleep_us(10);
		}
		if (rd(GCC_QUP0_SE4_CMD) & 1) {
			printf("goodix: the serial engine's clock does not take its configuration (%#x)\n", rd(GCC_QUP0_SE4_CMD));
			return false;
		}
		se_source_hz = XO_HZ;
		se_clock_select = 0;
	}
	wr(GCC_VOTE, rd(GCC_VOTE) | VOTE_QUP0_SE4);
	for (int i = 0; i < 1000; i++) {
		if (clock_ready(GCC_QUP0_M_AHB_CBCR) &&
		    clock_ready(GCC_QUP0_S_AHB_CBCR) &&
		    clock_ready(GCC_QUP0_SE4_CBCR)) {
			return true;
		}
		sleep_us(10);
	}
	printf("goodix: clocks stay off: %#x %#x %#x\n", rd(GCC_QUP0_M_AHB_CBCR), rd(GCC_QUP0_S_AHB_CBCR),
	    rd(GCC_QUP0_SE4_CBCR));
	return false;
}

/* ---- SPI through the serial engine (GENI) ---------------------------------------------------------- */

#define SE_PHYS                 se_phys
#define SE_SIZE                 0x4000UL
#define GENI_FORCE_DEFAULT      (se + 0x020)
#define GENI_OUTPUT_CTRL        (se + 0x024)
#define GENI_CGC_CTRL           (se + 0x028)
#define GENI_STATUS             (se + 0x040)    /* bit 0: a command is running */
#define GENI_SER_M_CLK_CFG      (se + 0x048)    /* bit 0 enable, [15:4] divider */
#define GENI_IF_DISABLE_RO      (se + 0x064)    /* bit 0: no FIFO interface */
#define GENI_FW_REVISION_RO     (se + 0x068)    /* [15:8] protocol: 1 = SPI */
#define GENI_CLK_SEL            (se + 0x07c)
#define SPI_CPHA                (se + 0x224)
#define SPI_LOOPBACK            (se + 0x22c)
#define SPI_CPOL                (se + 0x230)
#define SPI_DEMUX_OUTPUT_INV    (se + 0x24c)
#define SPI_DEMUX_SEL           (se + 0x250)
#define GENI_BYTE_GRAN          (se + 0x254)
#define GENI_DMA_MODE_EN        (se + 0x258)
#define SPI_TRANS_CFG           (se + 0x25c)
#define GENI_TX_PACKING_CFG0    (se + 0x260)
#define GENI_TX_PACKING_CFG1    (se + 0x264)
#define SPI_WORD_LEN            (se + 0x268)
#define SPI_TX_TRANS_LEN        (se + 0x26c)
#define SPI_RX_TRANS_LEN        (se + 0x270)
#define SPI_DELAY_COUNTERS      (se + 0x278)
#define GENI_RX_PACKING_CFG0    (se + 0x284)
#define GENI_RX_PACKING_CFG1    (se + 0x288)
#define GENI_M_CMD0             (se + 0x600)    /* [31:27] operation */
#define GENI_M_CMD_CTRL         (se + 0x604)    /* bit 2 cancel, bit 1 abort */
#define GENI_M_IRQ_STATUS       (se + 0x610)
#define GENI_M_IRQ_EN           (se + 0x614)
#define GENI_M_IRQ_CLEAR        (se + 0x618)
#define GENI_S_IRQ_CLEAR        (se + 0x648)
#define GENI_TX_FIFO            (se + 0x700)
#define GENI_RX_FIFO            (se + 0x780)
#define GENI_TX_FIFO_STATUS     (se + 0x800)    /* [27:0] words */
#define GENI_RX_FIFO_STATUS     (se + 0x804)    /* bit 31 last, [30:28] bytes of the last word, [24:0] words */
#define GENI_TX_WATERMARK       (se + 0x80c)
#define GENI_RX_WATERMARK       (se + 0x810)
#define GENI_RX_RFR_WATERMARK   (se + 0x814)
#define SE_DMA_TX_PTR_L         (se + 0xc30)
#define SE_DMA_TX_PTR_H         (se + 0xc34)
#define SE_DMA_TX_ATTR          (se + 0xc38)
#define SE_DMA_TX_LEN           (se + 0xc3c)
#define SE_DMA_TX_IRQ_STAT      (se + 0xc40)
#define SE_DMA_TX_IRQ_CLR       (se + 0xc44)
#define SE_DMA_TX_IRQ_EN_SET    (se + 0xc4c)
#define SE_DMA_TX_FSM_RST       (se + 0xc58)
#define SE_DMA_RX_PTR_L         (se + 0xd30)
#define SE_DMA_RX_PTR_H         (se + 0xd34)
#define SE_DMA_RX_ATTR          (se + 0xd38)
#define SE_DMA_RX_LEN           (se + 0xd3c)
#define SE_DMA_RX_IRQ_STAT      (se + 0xd40)
#define SE_DMA_RX_IRQ_CLR       (se + 0xd44)
#define SE_DMA_RX_IRQ_EN_SET    (se + 0xd4c)
#define SE_DMA_RX_FSM_RST       (se + 0xd58)
#define SE_GSI_EVENT_EN         (se + 0xe18)
#define SE_IRQ_EN               (se + 0xe1c)
#define SE_HW_PARAM_0           (se + 0xe24)
#define SE_DMA_GENERAL_CFG      (se + 0xe30)

#define QUP_WRAPPER_PHYS        0x009c0000UL
#define QUP_WRAPPER_SID         0x5a3u          /* live DT geniqup@9c0000 iommus */
#define APPS_SMMU_PHYS          0x15000000UL
#define APPS_SMMU_SIZE          0xd0000UL

#define M_CMD_DONE              (1u << 0)
#define M_CMD_CANCEL            (1u << 4)
#define M_CMD_ABORT             (1u << 5)
#define M_ERRORS                ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 24) | (1u << 25) | (1u << 28) | (1u << 29))
#define SPI_TX_ONLY             (1u << 27)
#define SPI_FULL_DUPLEX         (3u << 27)
#define GENI_DMA_DONE           (1u << 0)
#define GENI_DMA_AHB_ERR        (1u << 2)
#define GENI_DMA_TX_ATTR_EOT    (1u << 0)

static uint32_t fifo_depth, geni_actual_hz;
static bool geni_use_dma;
static uint64_t geni_dma_tx_phys, geni_dma_rx_phys;
static const char *spi_transport_name = "bitbang";
static struct {
	uint8_t tx[4096];
	uint8_t rx[4096];
} __attribute__((aligned(16384))) geni_dma_buf;

static void
dma_cache(void *p, size_t n, int clean_only)
{
	uint64_t ctr = 0;
	__asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
	size_t line = 4u << ((ctr >> 16) & 15u);
	for (uintptr_t a = (uintptr_t)p & ~(line - 1), end = (uintptr_t)p + n; a < end; a += line) {
		if (clean_only) {
			__asm__ volatile("dc cvac, %0" :: "r"(a) : "memory");
		} else {
			__asm__ volatile("dc civac, %0" :: "r"(a) : "memory");
		}
	}
	__asm__ volatile("dsb sy" ::: "memory");
}

static uint64_t
dma_vtop(const void *p)
{
	vm_offset_t va = (vm_offset_t)p;
	boolean_t enabled = ml_set_interrupts_enabled(FALSE);
	uint64_t par = 0;
	__asm__ volatile("at s1e1r, %1\n\tisb\n\tmrs %0, par_el1" : "=r"(par) : "r"(va) : "memory");
	ml_set_interrupts_enabled(enabled);
	if (!(par & 1u)) {
		return (par & 0x0000fffffffff000ULL) | (va & 0xfffu);
	}
	return ml_vtophys(va);
}

static unsigned
smmu_addr_bits(unsigned code)
{
	switch (code) {
	case 0: return 32;
	case 1: return 36;
	case 2: return 40;
	case 3: return 42;
	case 4: return 44;
	case 5: return 48;
	default: return 0;
	}
}

static const char *
geni_dma_prepare(void)
{
	if (!diting_smmu_lease_lock || !diting_smmu_lease_unlock) {
		return "SMMU lease symbols unavailable for GENI DMA";
	}
	geni_dma_tx_phys = dma_vtop(geni_dma_buf.tx);
	geni_dma_rx_phys = dma_vtop(geni_dma_buf.rx);
	if (!geni_dma_tx_phys || !geni_dma_rx_phys || (geni_dma_tx_phys & 0xfffu) || (geni_dma_rx_phys & 0xfffu)) {
		return "GENI DMA page translation failed";
	}
	bzero(&geni_dma_buf, sizeof(geni_dma_buf));
	dma_cache(&geni_dma_buf, sizeof(geni_dma_buf), 0);

	vm_offset_t smmu = ml_io_map(APPS_SMMU_PHYS, APPS_SMMU_SIZE);
	if (!smmu) {
		return "apps_smmu map failed";
	}
	if (!diting_smmu_lease_lock()) {
		return "apps_smmu lease lock failed";
	}

	const char *err = NULL;
	uint32_t idr0 = rd(smmu + 0x20), idr1 = rd(smmu + 0x24), idr2 = rd(smmu + 0x28);
	unsigned count = idr0 & 255u, banks = idr1 & 255u;
	unsigned pages = 1u << (((idr1 >> 28) & 7u) + 1u);
	uint64_t rx_end = geni_dma_rx_phys + 4096u, tx_end = geni_dma_tx_phys + 4096u;
	uint64_t hi = rx_end > tx_end ? rx_end : tx_end;
	if (!(idr0 & (1u << 30)) || !(idr2 & (1u << 12))) {
		err = "SMMU stage 1 unsupported";
		goto out;
	}
	for (unsigned shift = 0; shift <= 8; shift += 4) {
		unsigned bits = smmu_addr_bits((idr2 >> shift) & 15u);
		if (!bits || hi > (1ULL << bits)) {
			err = "GENI DMA exceeds SMMU width";
			goto out;
		}
	}
	if ((idr1 >> 31) || !count || pages > 128 || banks < 2 || banks > 128 || (pages + banks) * 4096u > APPS_SMMU_SIZE) {
		err = "unsupported SMMU geometry";
		goto out;
	}
	if (count > 128) {
		count = 128;
	}
	unsigned bank = banks - 1u, slot = 0xffffffffu, matches = 0, sharing = 0, reuse = 0;
	for (unsigned i = 0; i < count; i++) {
		uint32_t smr = rd(smmu + 0x800 + 4 * i);
		if (!(smr & 0x80000000u)) {
			if (slot == 0xffffffffu) {
				slot = i;
			}
			continue;
		}
		uint32_t s2cr = rd(smmu + 0xc00 + 4 * i);
		uint32_t mask = (smr >> 16) & 0x7fffu;
		int our_bank = ((s2cr >> 16) & 3u) == 0 && (s2cr & 255u) == bank;
		if (((smr ^ QUP_WRAPPER_SID) & 0xffffu & ~mask) == 0) {
			if (++matches > 1 || mask || !our_bank) {
				err = "conflicting QUP SMMU mapping";
				goto out;
			}
			reuse = 1;
			slot = i;
		} else if (our_bank) {
			sharing++;
		}
	}
	vm_offset_t cb = smmu + (pages + bank) * 4096u;
	uint32_t sctlr = rd(cb), cbar = rd(smmu + 0x1000 + 4 * bank), cba2r = rd(smmu + 0x1800 + 4 * bank);
	int bank_ready = (sctlr == 0 && cbar == 0x10000u && (cba2r & ~1u) == 0 && (cba2r & 1u));
	if ((sctlr & 1u) || ((sharing || reuse) && !bank_ready)) {
		err = "SMMU bypass bank not ready";
		goto out;
	}
	if (slot == 0xffffffffu) {
		err = "no free SMMU group";
		goto out;
	}
	if (!bank_ready) {
		wr(cb, 0);
		wr(smmu + 0x1800 + 4 * bank, 1);
		wr(smmu + 0x1000 + 4 * bank, 0x10000u);
	}
	if (rd(cb) != 0 || rd(smmu + 0x1800 + 4 * bank) != 1 || rd(smmu + 0x1000 + 4 * bank) != 0x10000u) {
		err = "SMMU bypass bank readback failed";
		goto out;
	}
	if (!reuse) {
		wr(smmu + 0xc00 + 4 * slot, bank);
		if (rd(smmu + 0xc00 + 4 * slot) != bank) {
			err = "SMMU S2CR readback failed";
			goto out;
		}
		wr(smmu + 0x800 + 4 * slot, 0x80000000u | QUP_WRAPPER_SID);
	}
	if (rd(smmu + 0x800 + 4 * slot) != (0x80000000u | QUP_WRAPPER_SID) || rd(smmu + 0xc00 + 4 * slot) != bank) {
		err = "SMMU SMR readback failed";
		goto out;
	}
out:
	diting_smmu_lease_unlock();
	return err;
}

/* 0 = ready; otherwise why the engine cannot be used */
static const char *
geni_spi_init(uint32_t hz)
{
	uint32_t divider, fw_rev = rd(GENI_FW_REVISION_RO), if_dis = rd(GENI_IF_DISABLE_RO), hw0 = rd(SE_HW_PARAM_0);
	uint32_t oversampling = 1, depth_mask = 0x3fu;
	vm_offset_t wrap = ml_io_map(QUP_WRAPPER_PHYS, 0x1000);

	if (wrap) {
		uint32_t ver = rd(wrap + 4), major = ver >> 28, minor = (ver >> 16) & 0xfffu;
		if ((major == 3 && minor >= 10) || major > 3) {
			depth_mask = 0xffu;
		}
		if (major == 1 && minor == 0) {
			oversampling = 2;
		}
	}
	if (((fw_rev >> 8) & 0xff) != 1) {
		return "the serial engine has no SPI firmware";
	}
	fifo_depth = (hw0 >> 16) & depth_mask;
	if (fifo_depth < 4) {
		return "the serial engine reports no FIFO";
	}
	geni_use_dma = false;
	if (if_dis & 1) {
		const char *dma_err = geni_dma_prepare();
		if (dma_err) {
			return dma_err;
		}
		geni_use_dma = true;
		spi_transport_name = "GENI-DMA";
	} else {
		spi_transport_name = "GENI-FIFO";
	}
	wr(SE_GSI_EVENT_EN, 0);
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(GENI_S_IRQ_CLEAR, 0xffffffff);
	wr(SE_DMA_TX_IRQ_CLR, 0xffffffff);
	wr(SE_DMA_RX_IRQ_CLR, 0xffffffff);
	wr(GENI_CGC_CTRL, rd(GENI_CGC_CTRL) | 0x7f);
	wr(SE_DMA_GENERAL_CFG, rd(SE_DMA_GENERAL_CFG) | 0xf);
	wr(GENI_OUTPUT_CTRL, 0x7f);
	wr(GENI_FORCE_DEFAULT, 1);
	wr(GENI_RX_WATERMARK, fifo_depth - 3);
	wr(GENI_RX_RFR_WATERMARK, fifo_depth - 2);
	wr(GENI_TX_WATERMARK, 0);
	/* the status bits we poll follow these enables; the interrupt itself stays masked at the GIC */
	wr(SE_IRQ_EN, rd(SE_IRQ_EN) | 0xf);
	if (geni_use_dma) {
		wr(GENI_M_IRQ_EN, (rd(GENI_M_IRQ_EN) | 0x7fc0007f) & ~(M_CMD_DONE | (1u << 26) | (1u << 27) | (1u << 30)));
		wr(GENI_DMA_MODE_EN, rd(GENI_DMA_MODE_EN) | 1u);
	} else {
		wr(GENI_M_IRQ_EN, rd(GENI_M_IRQ_EN) | 0x7fc0007f);
		wr(GENI_DMA_MODE_EN, rd(GENI_DMA_MODE_EN) & ~1u);
	}

	/* mode 0, chip select 0, 8 bits, most significant first, four bytes in a FIFO word */
	wr(SPI_LOOPBACK, 0);
	wr(SPI_DEMUX_SEL, 0);
	wr(SPI_CPHA, 0);
	wr(SPI_CPOL, 0);
	wr(SPI_DEMUX_OUTPUT_INV, 0);
	wr(SPI_TRANS_CFG, rd(SPI_TRANS_CFG) & ~2u);
	wr(SPI_DELAY_COUNTERS, 0);
	wr(GENI_TX_PACKING_CFG0, 0x0007f8fe);
	wr(GENI_TX_PACKING_CFG1, 0x000ffefe);
	wr(GENI_RX_PACKING_CFG0, 0x0007f8fe);
	wr(GENI_RX_PACKING_CFG1, 0x000ffefe);
	wr(GENI_BYTE_GRAN, 0);
	wr(SPI_WORD_LEN, 8 - 4);

	uint64_t denom = (uint64_t)oversampling * hz;
	divider = (uint32_t)((se_source_hz + denom - 1) / denom);
	if (divider < 1) {
		divider = 1;
	}
	if (divider > 0xfffu) {
		divider = 0xfffu;
	}
	wr(GENI_CLK_SEL, se_clock_select & 7);
	wr(GENI_SER_M_CLK_CFG, divider << 4 | 1);
	geni_actual_hz = se_source_hz / (oversampling * divider);
	printf("goodix: %s ready (sclk=%u sel=%u div=%u spi=%u fifo=%u if_dis=%u)\n",
	    spi_transport_name, se_source_hz, se_clock_select, divider, geni_actual_hz, fifo_depth, if_dis);
	return NULL;
}

static void
geni_drain(uint8_t *rx, uint32_t length, uint32_t *received)
{
	uint32_t status = rd(GENI_RX_FIFO_STATUS), words = status & 0x01ffffff, bytes = words * 4;
	if (words > fifo_depth) return; /* inaccessible/bad status must not cause an unbounded MMIO drain */

	if (status & 0x80000000u) {
		uint32_t valid = status >> 28 & 7;

		if (valid && valid < 4) {
			bytes -= 4 - valid;
		}
	}
	for (uint32_t i = 0; i < words; i++) {
		uint32_t word = rd(GENI_RX_FIFO);

		for (int j = 0; j < 4 && bytes; j++, bytes--) {
			if (*received < length) {
				if (rx) {
					rx[*received] = (uint8_t)(word >> (8 * j));
				}
				(*received)++;
			}
		}
	}
}

static void
geni_recover(void)
{
	wr(GENI_M_CMD_CTRL, 1u << 2);
	for (int i = 0; i < 2000 && !(rd(GENI_M_IRQ_STATUS) & M_CMD_CANCEL); i++) {
		sleep_us(100);
	}
	if (!(rd(GENI_M_IRQ_STATUS) & M_CMD_CANCEL)) {
		wr(GENI_M_CMD_CTRL, 1u << 1);
		for (int i = 0; i < 2000 && !(rd(GENI_M_IRQ_STATUS) & M_CMD_ABORT); i++) {
			sleep_us(100);
		}
	}
	for (int i = 0; i < 64 && (rd(GENI_RX_FIFO_STATUS) & 0x01ffffff); i++) {
		(void)rd(GENI_RX_FIFO);
	}
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
}

/* One transfer with the chip selected throughout: `length` bytes out (zeros without tx) and as many in. */
static bool
geni_transfer_dma(const uint8_t *tx, uint8_t *rx, uint32_t length)
{
	static unsigned complaints;
	uint32_t ts = 0, rs = 0, ms = 0;
	uint64_t deadline;

	if (!length || length > sizeof(geni_dma_buf.tx)) {
		return false;
	}
	for (int i = 0; rd(GENI_STATUS) & 1; i++) {
		if (i > 2000) {
			geni_recover();
			break;
		}
		sleep_us(100);
	}
	if (tx) {
		memcpy(geni_dma_buf.tx, tx, length);
		dma_cache(geni_dma_buf.tx, length, 1);
		wr(SPI_TX_TRANS_LEN, length);
	}
	if (rx) {
		memset(geni_dma_buf.rx, 0x5a, length);
		dma_cache(geni_dma_buf.rx, length, 0);
		wr(SPI_RX_TRANS_LEN, length);
	}
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(SE_DMA_TX_IRQ_CLR, 0xffffffff);
	wr(SE_DMA_RX_IRQ_CLR, 0xffffffff);
	wr(GENI_M_CMD0, (tx && rx) ? SPI_FULL_DUPLEX : SPI_TX_ONLY);
	if (rx) {
		wr(SE_DMA_RX_IRQ_EN_SET, GENI_DMA_DONE | (1u << 1) | GENI_DMA_AHB_ERR);
		wr(SE_DMA_RX_PTR_L, (uint32_t)geni_dma_rx_phys);
		wr(SE_DMA_RX_PTR_H, (uint32_t)(geni_dma_rx_phys >> 32));
		wr(SE_DMA_RX_ATTR, 0);
		wr(SE_DMA_RX_LEN, length);
	}
	if (tx) {
		wr(SE_DMA_TX_IRQ_EN_SET, GENI_DMA_DONE | (1u << 1) | GENI_DMA_AHB_ERR);
		wr(SE_DMA_TX_PTR_L, (uint32_t)geni_dma_tx_phys);
		wr(SE_DMA_TX_PTR_H, (uint32_t)(geni_dma_tx_phys >> 32));
		wr(SE_DMA_TX_ATTR, GENI_DMA_TX_ATTR_EOT);
		wr(SE_DMA_TX_LEN, length);
	}
	bool tx_done = !tx, rx_done = !rx;
	clock_interval_to_deadline(200, NSEC_PER_MSEC, &deadline);
	while (mach_absolute_time() < deadline) {
		ts = tx ? rd(SE_DMA_TX_IRQ_STAT) : 0;
		rs = rx ? rd(SE_DMA_RX_IRQ_STAT) : 0;
		ms = rd(GENI_M_IRQ_STATUS);
		if (ts) {
			wr(SE_DMA_TX_IRQ_CLR, ts);
		}
		if (rs) {
			wr(SE_DMA_RX_IRQ_CLR, rs);
		}
		if (ms) {
			wr(GENI_M_IRQ_CLEAR, ms);
		}
		if ((ts & GENI_DMA_AHB_ERR) || (rs & GENI_DMA_AHB_ERR) || (ms & M_ERRORS)) {
			break;
		}
		if (ts & GENI_DMA_DONE) {
			tx_done = true;
		}
		if (rs & GENI_DMA_DONE) {
			rx_done = true;
		}
		if (tx_done && rx_done) {
			if (rx) {
				dma_cache(geni_dma_buf.rx, length, 0);
				memcpy(rx, geni_dma_buf.rx, length);
			}
			return true;
		}
		sleep_us(1);
	}
	if (complaints++ < 8) {
		printf("goodix: DMA SPI transfer of %u bytes failed: m=%#x tx=%#x rx=%#x\n", length, ms, ts, rs);
	}
	geni_recover();
	wr(SE_DMA_TX_FSM_RST, 1);
	wr(SE_DMA_RX_FSM_RST, 1);
	return false;
}

static bool
geni_transfer(const uint8_t *tx, uint8_t *rx, uint32_t length)
{
	static unsigned complaints;
	uint32_t sent = 0, received = 0, status = 0;
	uint64_t deadline;

	if (geni_use_dma) {
		return geni_transfer_dma(tx, rx, length);
	}
	for (int i = 0; rd(GENI_STATUS) & 1; i++) {
		if (i > 2000) {
			geni_recover();
			break;
		}
		sleep_us(100);
	}
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(SPI_TX_TRANS_LEN, length);
	wr(SPI_RX_TRANS_LEN, length);
	wr(GENI_M_CMD0, SPI_FULL_DUPLEX);
	clock_interval_to_deadline(200, NSEC_PER_MSEC, &deadline);
	while (mach_absolute_time() < deadline) {
		while (sent < length && (rd(GENI_TX_FIFO_STATUS) & 0x0fffffff) + 1 < fifo_depth) {
			uint32_t word = 0;

			for (int j = 0; j < 4 && sent < length; j++, sent++) {
				word |= (uint32_t)(tx ? tx[sent] : 0) << (8 * j);
			}
			wr(GENI_TX_FIFO, word);
		}
		status = rd(GENI_M_IRQ_STATUS);
		geni_drain(rx, length, &received);
		if (status & M_ERRORS) {
			break;
		}
		if (status & M_CMD_DONE) {
			geni_drain(rx, length, &received);
			wr(GENI_M_IRQ_CLEAR, status);
			if (received == length) {
				return true;
			}
			break;
		}
	}
	if (complaints++ < 8) {
		printf("goodix: SPI transfer of %u bytes failed: status %#x, sent %u, received %u\n", length, status, sent,
		    received);
	}
	geni_recover();
	return false;
}

/* ---- SPI by hand, on the same pins ----------------------------------------------------------------- */

static unsigned bitbang_in, bitbang_out;

static void
bitbang_init(void)
{
	gpio_output(PIN_CS, true, PULL_NONE, 6);
	gpio_output(PIN_CLK, false, PULL_NONE, 6);
	gpio_output(bitbang_out, false, PULL_NONE, 6);
	gpio_configure(bitbang_in, 0, PULL_NONE, 2, false);
}

static bool
bitbang_transfer(const uint8_t *tx, uint8_t *rx, uint32_t length)
{
	gpio_set(PIN_CS, false);
	for (uint32_t i = 0; i < length; i++) {
		uint8_t out = tx ? tx[i] : 0, in = 0;

		for (int bit = 7; bit >= 0; bit--) {
			gpio_set(bitbang_out, out >> bit & 1);
			gpio_set(PIN_CLK, true);
			in = (uint8_t)(in << 1 | gpio_get(bitbang_in));
			gpio_set(PIN_CLK, false);
		}
		if (rx) {
			rx[i] = in;
		}
	}
	gpio_set(PIN_CS, true);
	return true;
}

static bool (*spi_transfer)(const uint8_t *tx, uint8_t *rx, uint32_t length);

/* ---- the Goodix chip ------------------------------------------------------------------------------- */

#define GOODIX_FW_VERSION       0x10014         /* 28 bytes */
#define GOODIX_IC_INFO          0x10070         /* a 16-bit length, then that many bytes */
#define GOODIX_MAX_READ         4000    /* config packages and THP frames; fits inside one 4K DMA page */
#define GOODIX_MAX_TOUCHES      10
#define GOODIX_STATUS_TOUCH     0x80
#define GOODIX_STATUS_REQUEST   0x40
#define GOODIX_REQUEST_RESET    3

static uint8_t spi_tx[8 + GOODIX_MAX_READ], spi_rx[8 + GOODIX_MAX_READ];
static uint32_t touch_address, command_address, fw_buffer_address, ic_config_id;
static uint32_t frame_address, frame_skip;   /* misc +24, +28/+30/+32 */
static uint8_t thp_tx, thp_rx, thp_mode;
static uint16_t fw_buffer_max_len;

static bool
goodix_read(uint32_t address, uint8_t *data, uint32_t length)
{
	if (length > GOODIX_MAX_READ) {
		return false;
	}
	spi_tx[0] = 0xf1;
	spi_tx[1] = (uint8_t)(address >> 24);
	spi_tx[2] = (uint8_t)(address >> 16);
	spi_tx[3] = (uint8_t)(address >> 8);
	spi_tx[4] = (uint8_t)address;
	spi_tx[5] = spi_tx[6] = spi_tx[7] = 0xff;
	memset(spi_tx + 8, 0, length);
	if (!spi_transfer(spi_tx, spi_rx, 8 + length)) {
		return false;
	}
	memcpy(data, spi_rx + 8, length);
	return true;
}

static bool
goodix_write(uint32_t address, const uint8_t *data, uint32_t length)
{
	if (length > GOODIX_MAX_READ) {
		return false;
	}
	spi_tx[0] = 0xf0;
	spi_tx[1] = (uint8_t)(address >> 24);
	spi_tx[2] = (uint8_t)(address >> 16);
	spi_tx[3] = (uint8_t)(address >> 8);
	spi_tx[4] = (uint8_t)address;
	memcpy(spi_tx + 5, data, length);
	return spi_transfer(spi_tx, NULL, 5 + length);
}

/* The chip's blocks end in the 16-bit sum of their bytes; all zeros or all ones is no answer at all. */
static bool
goodix_checksum_ok(const uint8_t *data, uint32_t length)
{
	uint32_t sum = 0;
	bool blank = true;

	if (length < 2) {
		return false;
	}
	for (uint32_t i = 0; i < length - 2; i++) {
		sum += data[i];
		blank &= data[i] == 0 || data[i] == 0xff;
	}
	return !blank && (uint16_t)sum == (uint16_t)(data[length - 2] | data[length - 1] << 8);
}

static uint16_t
le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | p[1] << 8);
}

static uint32_t
le32(const uint8_t *p)
{
	return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

static void
touch_reset(void)
{
	gpio_set(PIN_RESET, false);
	sleep_ms(2);
	gpio_set(PIN_RESET, true);
	sleep_ms(100);
}

/* Is the chip there and running its firmware? Finds where it keeps the touch report. */
static bool
goodix_probe(bool verbose)
{
	static uint8_t info[GOODIX_MAX_READ];
	uint8_t version[28];
	uint32_t length, offset;

	if (!goodix_read(GOODIX_FW_VERSION, version, sizeof(version))) {
		return false;
	}
	if (!goodix_checksum_ok(version, sizeof(version))) {
		if (verbose) {
			printf("goodix: no valid firmware version: %02x %02x %02x %02x %02x %02x %02x %02x ... %02x %02x\n",
			    version[0], version[1], version[2], version[3], version[4], version[5], version[6], version[7],
			    version[26], version[27]);
		}
		return false;
	}
	printf("goodix: chip \"%.6s\", firmware \"%.8s\" %02x.%02x.%02x.%02x, sensor %u\n", (const char *)version,
	    (const char *)version + 10, version[18], version[19], version[20], version[21], version[23]);

	if (!goodix_read(GOODIX_IC_INFO, info, 2)) {
		return false;
	}
	length = le16(info);
	if (length < 40 || length >= GOODIX_MAX_READ || !goodix_read(GOODIX_IC_INFO, info, length) ||
	    !goodix_checksum_ok(info, length)) {
		printf("goodix: bad chip description (length %u)\n", length);
		return false;
	}
	/* the length, the version (16 bytes), the features (10), four counts, then five lists of 16-bit numbers */
	offset = 2 + 16 + 10 + 4;
	for (int i = 0; i < 5; i++) {
		if (offset >= length) {
			return false;
		}
		offset += 1 + 2u * info[offset];
	}
	if (offset + 48 + 2 > length) {
		printf("goodix: chip description too short (%u, lists end at %u)\n", length, offset);
		return false;
	}
	touch_address = le32(info + offset + 44);
	command_address = le32(info + offset + 0);      /* (the "misc" part: the command buffer comes first) */
	fw_buffer_address = le32(info + offset + 18);   /* where a config package is staged */
	fw_buffer_max_len = le16(info + offset + 22);
	frame_address = le32(info + offset + 24);
	frame_skip = le16(info + offset + 28) + le16(info + offset + 30) + le16(info + offset + 32);
	thp_tx = info[2 + 16 + 10];
	thp_rx = info[2 + 16 + 10 + 1];
	ic_config_id = le32(info + 6);                  /* ic_info_version.config_id (version+4) */
    /* Vendor misc +52/+54 are display-space axes (1220x2712). Touch reports
     * arrive in the vendor panel space (dts goodix,panel-max-*: 12200x27120),
     * which the DT max-x/max-y carries; keep DT authoritative. */
    if (offset+56+2 > length || le16(info+offset+48)!=8 || le16(info+offset+50)!=8) return false;
    uint16_t x=le16(info+offset+52), y=le16(info+offset+54);
	printf("goodix: touch report at %#x, display axes %u x %u, report space %u x %u\n", touch_address, x, y, max_x, max_y);
	return touch_address != 0;
}

/* ---- Berlin-D configuration upload -----------------------------------------------------------
 * The controller does not scan until it holds a panel configuration (sensor 0xff).
 * The vendor cfg group reaches the chip as a 12-byte-prefixed package written to the
 * ic_info firmware buffer, framed by 4-byte commands at the command address; command
 * replies carry the state machine at byte 0 and the ack at byte 1. Protocol and
 * status values follow the goodix_9916 (Berlin-D) module shipped on this device. */

#define GOODIX_CFG_CMD_LEN      4
#define GOODIX_CFG_START_BD     0x0a    /* prepare to receive a config buffer */
#define GOODIX_CFG_WRITE_BD     0x0b    /* the buffer holds a config: take it */
#define GOODIX_CFG_EXIT_BD      0x0c    /* leave the config exchange */
#define GOODIX_CFG_ACK_START_BD 0x04
#define GOODIX_CFG_ACK_WRITE_BD 0x07
#define GOODIX_CFG_DONE         0x80
#define GOODIX_CFG_PKG_ADDR     0x3e000 /* where the package is staged inside the chip */

/* A framed vendor command: {0,0,len=4+n,cmd,data[n],u16 sum over bytes 2..3+n}
 * written to command_address; the chip acks 0x80 at reply byte 1. */
static bool
goodix_send_cmd(uint8_t command, const uint8_t *data, uint32_t n)
{
	uint8_t buf[16] = {0}, reply[16] = {0};
	uint16_t sum = 0;

	if (!command_address || n > 8) {
		return false;
	}
	buf[2] = (uint8_t)(n + 4);
	buf[3] = command;
	for (uint32_t i = 0; i < n; i++) {
		buf[4 + i] = data[i];
	}
	for (uint32_t i = 2; i < 4 + n; i++) {
		sum += buf[i];
	}
	buf[4 + n] = (uint8_t)sum;
	buf[5 + n] = (uint8_t)(sum >> 8);
	for (int attempt = 0; attempt < 6; attempt++) {
		if (!goodix_write(command_address, buf, sizeof(buf))) {
			return false;
		}
		for (int i = 0; i < 6; i++) {
			if (!goodix_read(command_address, reply, sizeof(reply))) {
				return false;
			}
			if (reply[1] == 0x80) {
				sleep_ms(2);
				return true;
			}
			if (reply[1] == 0x02 || reply[1] == 0x00) {
				sleep_ms(1);
				continue;
			}
			sleep_ms(reply[1] == 0x03 ? 10 : 1);
			break;
		}
	}
	return false;
}

static bool
goodix_cfg_cmd(uint8_t command, uint8_t target)
{
	uint8_t cmd[16] = {0, 0, GOODIX_CFG_CMD_LEN, command}, reply[16] = {0};
	uint16_t sum = cmd[2] + cmd[3];             /* the vendor's u8-mode checksum over len+command */

	cmd[4] = (uint8_t)sum;
	cmd[5] = (uint8_t)(sum >> 8);
	for (int attempt = 0; attempt < 6; attempt++) {
		if (!goodix_write(command_address, cmd, sizeof(cmd))) {
			sleep_ms(1);
			continue;
		}
		for (int i = 0; i < 6; i++) {           /* accepted commands ack 0x80 at byte 1 */
			if (goodix_read(command_address, reply, sizeof(reply)) && reply[1] == GOODIX_CFG_DONE) {
				for (i = 0; i < 20; i++) {  /* then the state machine reaches its target */
					if (goodix_read(command_address, reply, sizeof(reply)) && reply[0] == target) {
						return true;
					}
					sleep_ms(20);
				}
				printf("goodix: config command %#x never reached state %#x (%02x %02x)\n",
				    command, target, reply[0], reply[1]);
				return false;
			}
			sleep_ms(1);
		}
	}
	printf("goodix: config command %#x not acknowledged (%02x %02x)\n", command, reply[0], reply[1]);
	return false;
}

/* The staged package: a u32 checksum of the u16 words over {address,length,data},
 * the 0x3e000 staging address, the payload length, then the configuration. */
static bool
goodix_send_config(const uint8_t *cfg, uint32_t length)
{
	static uint8_t pkg[12 + GOODIX_MAX_READ], back[12 + GOODIX_MAX_READ];
	uint32_t sum = 0, total = 12 + length;

	if (!length || !fw_buffer_address || length > fw_buffer_max_len || total > sizeof(pkg)) {
		printf("goodix: config %u cannot go to %#x (max %u)\n", length, fw_buffer_address,
		    fw_buffer_max_len);
		return false;
	}
	memset(pkg, 0, total);
	pkg[4] = 0x00; pkg[5] = 0xe0; pkg[6] = 0x03; pkg[7] = 0x00;
	pkg[8] = (uint8_t)length; pkg[9] = (uint8_t)(length >> 8);
	pkg[10] = (uint8_t)(length >> 16); pkg[11] = (uint8_t)(length >> 24);
	memcpy(pkg + 12, cfg, length);
	for (uint32_t i = 4; i + 2 <= total; i += 2) {
		sum += le16(pkg + i);
	}
	pkg[0] = (uint8_t)sum; pkg[1] = (uint8_t)(sum >> 8);
	pkg[2] = (uint8_t)(sum >> 16); pkg[3] = (uint8_t)(sum >> 24);
	if (!goodix_cfg_cmd(GOODIX_CFG_START_BD, GOODIX_CFG_ACK_START_BD)) {
		return false;
	}
	if (!goodix_write(fw_buffer_address, pkg, total) || !goodix_read(fw_buffer_address, back, total) ||
	    memcmp(pkg, back, total)) {
		printf("goodix: config buffer write/compare failed at %#x\n", fw_buffer_address);
		(void)goodix_cfg_cmd(GOODIX_CFG_EXIT_BD, GOODIX_CFG_DONE);
		return false;
	}
	if (!goodix_cfg_cmd(GOODIX_CFG_WRITE_BD, GOODIX_CFG_ACK_WRITE_BD)) {
		(void)goodix_cfg_cmd(GOODIX_CFG_EXIT_BD, GOODIX_CFG_DONE);
		return false;
	}
	if (!goodix_cfg_cmd(GOODIX_CFG_EXIT_BD, GOODIX_CFG_DONE)) {
		return false;
	}
	printf("goodix: config uploaded (%u bytes through %#x)\n", length, fw_buffer_address);
	return true;
}

/* What has been seen so far: said once a minute, so that the end of the log always tells. */
static struct {
	uint32_t polls, reports, touches, bad, last_x, last_y;
} seen;

/* Which fingers were down in the last report, and where. */
static struct {
	bool down, seen;
	uint32_t x, y;
} contact[GOODIX_MAX_TOUCHES];
static uint32_t flip;           /* boot-arg touch_flip: 1 = mirror left-right, 2 = mirror top-bottom */

static void
goodix_report(const uint8_t *touches, uint32_t count)
{
	static unsigned shown;

	for (int i = 0; i < GOODIX_MAX_TOUCHES; i++) {
		contact[i].seen = false;
	}
	for (uint32_t i = 0; i < count; i++) {
		const uint8_t *t = touches + 8 * i;
		uint32_t id = t[0] >> 4, kind = t[0] & 0xf;
		uint32_t x = le16(t + 2), y = le16(t + 4);

		if (id >= GOODIX_MAX_TOUCHES || kind == 1 || kind == 3) {       /* 1 and 3: a stylus */
			continue;
		}
		if (shown < 4) {
			shown++;
			printf("goodix: finger %u at %u, %u (of %u x %u)\n", id, x, y, max_x, max_y);
		}
		seen.touches++;
		seen.last_x = x;
		seen.last_y = y;
		x = x >= max_x ? 65535 : x * 65535 / max_x;
		y = y >= max_y ? 65535 : y * 65535 / max_y;
		if (flip & 1) {
			x = 65535 - x;
		}
		if (flip & 2) {
			y = 65535 - y;
		}
		contact[id].seen = true;
		if (!contact[id].down) {
			diting_touch_post_at(DITING_TOUCH_DOWN,id,x,y,touch_frame_time);
		} else if (contact[id].x != x || contact[id].y != y) {
			diting_touch_post_at(DITING_TOUCH_MOVE,id,x,y,touch_frame_time);
		}
		contact[id].down = true;
		contact[id].x = x;
		contact[id].y = y;
	}
	/* a finger that is no longer in the report has lifted */
	for (uint32_t id = 0; id < GOODIX_MAX_TOUCHES; id++) {
		if (contact[id].down && !contact[id].seen) {
			contact[id].down = false;
			diting_touch_post_at(DITING_TOUCH_UP,id,contact[id].x,contact[id].y,touch_frame_time);
		}
	}
}

/* ---- Berlin-D THP host touch processing --------------------------------------
 * Command 0x90 with data 0x82 asks the firmware for mutual-capacitance
 * difference frames instead of finished coordinates (Xiaomi brld_get_cap_data);
 * this GT9916 family leaves coordinate computation to the host. Bit 7 of the
 * byte at frame_data_addr marks a ready frame and clears when written 0; the
 * int16-LE matrix follows the frame head, attribute and log blocks (misc
 * +28/+30/+32 lengths) and 8 more bytes, tx-major order. The host picks peaks,
 * integrates a 3x3 centroid, tracks slots and emits contacts. Reference:
 * K70 k70_touch.c / vermeer_goodix_info.h. Boot args: diting-touch-thr
 * (default 200), diting-touch-map (bit0 long-axis swap, bit1 mirror x,
 * bit2 mirror y). */

#define THP_NODES_MAX   1500
static int16_t thp_m[THP_NODES_MAX];
static uint8_t thp_used[THP_NODES_MAX];
static uint8_t thp_fbuf[GOODIX_MAX_READ];
static uint32_t thp_thr = 200, thp_map, thp_frames;
/* Instrumentation only: cadence and cost of each stage, no policy change. */
static uint64_t thp_last_frame, thp_interval_total, thp_interval_max;
static uint64_t thp_proc_total, thp_proc_max;
static uint32_t thp_interval_count, thp_polls;
static uint64_t coord_last_report, coord_interval_total, coord_interval_max;
static uint32_t coord_interval_count, coord_polls;
static struct {
	bool active;
	uint32_t x, y;
} thp_slot[GOODIX_MAX_TOUCHES];

static void
thp_emit(uint32_t id, uint32_t type, uint32_t x, uint32_t y)
{
	x = x >= max_x ? 65535 : x * 65535 / max_x;
	y = y >= max_y ? 65535 : y * 65535 / max_y;
	if (flip & 1) {
		x = 65535 - x;
	}
	if (flip & 2) {
		y = 65535 - y;
	}
	diting_touch_post_at(type, id, x, y, touch_frame_time);
}

static void
thp_process(void)
{
	const uint32_t T = thp_tx, R = thp_rx, nodes = T * R;
	const int thr = (int)thp_thr;
	struct {
		long x, y;
	} peak[5];
	uint32_t np = 0;

	bzero(thp_used, nodes);
	for (uint32_t k = 0; k < 5; k++) {
		int best = 0;
		uint32_t bi = 0;
		for (uint32_t i = 0; i < nodes; i++) {
			int v = thp_m[i] < 0 ? -thp_m[i] : thp_m[i];
			if (!thp_used[i] && v > best) {
				best = v;
				bi = i;
			}
		}
		if (best < thr) {
			break;
		}
		int pt = (int)(bi / R), pr = (int)(bi % R);
		long sw = 0, st = 0, sr = 0;
		for (int dt = -1; dt <= 1; dt++) for (int dr = -1; dr <= 1; dr++) {
			int t = pt + dt, r = pr + dr;
			if (t < 0 || r < 0 || t >= (int)T || r >= (int)R) {
				continue;
			}
			int v = thp_m[t * (int)R + r];
			if (v < 0) {
				v = -v;
			}
			int w = v - thr / 2;
			if (w <= 0) {
				continue;
			}
			sw += w;
			st += (long)w * (2 * t + 1);
			sr += (long)w * (2 * r + 1);
		}
		for (int dt = -2; dt <= 2; dt++) for (int dr = -2; dr <= 2; dr++) {
			int t = pt + dt, r = pr + dr;
			if (t >= 0 && r >= 0 && t < (int)T && r < (int)R) {
				thp_used[t * (int)R + r] = 1;
			}
		}
		if (!sw) {
			continue;
		}
		/* centroid in half-node units; the long matrix axis is the screen's
		 * long axis (per the K70 reference) */
		int t_is_y = ((int)T >= (int)R) ^ ((thp_map & 1) != 0);
		long cx = t_is_y ? sr : st, cy = t_is_y ? st : sr;
		long nx = 2L * (t_is_y ? R : T), ny = 2L * (t_is_y ? T : R);
		long x = cx * max_x / (nx * sw), y = cy * max_y / (ny * sw);
		if (thp_map & 2) {
			x = (long)max_x - 1 - x;
		}
		if (thp_map & 4) {
			y = (long)max_y - 1 - y;
		}
		if (x < 0) {
			x = 0;
		}
		if (x >= (long)max_x) {
			x = (long)max_x - 1;
		}
		if (y < 0) {
			y = 0;
		}
		if (y >= (long)max_y) {
			y = (long)max_y - 1;
		}
		peak[np].x = x;
		peak[np].y = y;
		np++;
	}
	/* Keep a finger's slot across frames: nearest previous contact within a
	 * scaled radius (K70 used 300 px on a 1440-wide panel), else a free slot. */
	uint32_t claimed = 0;
	const long near = (long)max_x * 300 / 1440;
	bool now[GOODIX_MAX_TOUCHES] = {false};
	uint32_t nx[GOODIX_MAX_TOUCHES], ny[GOODIX_MAX_TOUCHES];
	for (uint32_t k = 0; k < np; k++) {
		int slot = -1;
		long bestd = near * near;
		for (uint32_t s_ = 0; s_ < GOODIX_MAX_TOUCHES; s_++) {
			if (!thp_slot[s_].active || (claimed & (1u << s_))) {
				continue;
			}
			long dx = (long)thp_slot[s_].x - peak[k].x, dy = (long)thp_slot[s_].y - peak[k].y;
			if (dx * dx + dy * dy < bestd) {
				bestd = dx * dx + dy * dy;
				slot = (int)s_;
			}
		}
		if (slot < 0) {
			for (uint32_t s_ = 0; s_ < GOODIX_MAX_TOUCHES; s_++) {
				if (!thp_slot[s_].active && !(claimed & (1u << s_))) {
					slot = (int)s_;
					break;
				}
			}
		}
		if (slot < 0) {
			continue;
		}
		claimed |= 1u << slot;
		now[slot] = true;
		nx[slot] = (uint32_t)peak[k].x;
		ny[slot] = (uint32_t)peak[k].y;
	}
	for (uint32_t s_ = 0; s_ < GOODIX_MAX_TOUCHES; s_++) {
		if (now[s_]) {
			if (!thp_slot[s_].active) {
				thp_emit(s_, DITING_TOUCH_DOWN, nx[s_], ny[s_]);
			} else if (thp_slot[s_].x != nx[s_] || thp_slot[s_].y != ny[s_]) {
				thp_emit(s_, DITING_TOUCH_MOVE, nx[s_], ny[s_]);
			}
			thp_slot[s_].active = true;
			thp_slot[s_].x = nx[s_];
			thp_slot[s_].y = ny[s_];
		} else if (thp_slot[s_].active) {
			thp_slot[s_].active = false;
			thp_emit(s_, DITING_TOUCH_UP, thp_slot[s_].x, thp_slot[s_].y);
		}
	}
	if (np) {
		seen.touches += np;
	}
}

/* 1 = a frame was consumed, 0 = none ready, -1 = bus failure */
static int
thp_frame(void)
{
	uint8_t flag = 0;
	const uint8_t zero = 0;
	const uint32_t nodes = (uint32_t)thp_tx * thp_rx, off = frame_skip + 8;

	if (!goodix_read(frame_address, &flag, 1)) {
		return -1;
	}
	if (!(flag & 0x80)) {
		return 0;
	}
	if (!goodix_read(frame_address, thp_fbuf, off + nodes * 2)) {
		return -1;
	}
	if (!goodix_write(frame_address, &zero, 1)) {
		return -1;
	}
	for (uint32_t i = 0; i < nodes; i++) {
		thp_m[i] = (int16_t)le16(thp_fbuf + off + 2 * i);
	}
	thp_frames++;
	return 1;
}

/* Read the report if there is one, and tell the chip it has been read. */
static void
goodix_poll(void)
{
	static uint8_t report[8 + 8 * GOODIX_MAX_TOUCHES + 2];
	static unsigned bad;
	const uint8_t zero = 0;
	const uint32_t first = 8 + 8 * 2 + 2;           /* the header and two fingers: all there is, most of the time */
	uint32_t count;

    const uint64_t irq_stamp=__atomic_exchange_n(&touch_irq_time,0,__ATOMIC_ACQ_REL);
    touch_frame_time=irq_stamp?irq_stamp:mach_absolute_time();
	seen.polls++;
	++coord_polls;
	if (!goodix_read(touch_address, report, first) || report[0] == 0) {
		return;
	}
	seen.reports++;
	count = report[2] & 0xf;
	if ((report[0] & GOODIX_STATUS_TOUCH) && count > 2 && count <= GOODIX_MAX_TOUCHES &&
	    !goodix_read(touch_address + 8, report + 8, 8 * count + 2)) {
		return;
	}
	goodix_write(touch_address, &zero, 1);
	if (!goodix_checksum_ok(report, 8)) {
		seen.bad++;
		if (bad++ < 8) {
			printf("goodix: bad report header: %02x %02x %02x %02x %02x %02x %02x %02x\n", report[0], report[1],
			    report[2], report[3], report[4], report[5], report[6], report[7]);
		}
		return;
	}
	if (report[0] & GOODIX_STATUS_REQUEST) {
		if ((report[2] & 0xf) == GOODIX_REQUEST_RESET || report[2] == GOODIX_REQUEST_RESET) {
			printf("goodix: the chip asks to be reset\n");
			touch_reset();
		}
		return;
	}
	if (!(report[0] & GOODIX_STATUS_TOUCH)) {
		return;
	}
	if (count > GOODIX_MAX_TOUCHES || (count && !goodix_checksum_ok(report + 8, count * 8 + 2))) {
		seen.bad++;
		if (bad++ < 8) {
			printf("goodix: bad report (%u touches)\n", count);
		}
		return;
	}
	goodix_report(report + 8, count);
    const uint64_t now=mach_absolute_time();
    if(coord_last_report){uint64_t ins=0;absolutetime_to_nanoseconds(now-coord_last_report,&ins);
        const uint64_t ius=ins/1000;coord_interval_total+=ius;++coord_interval_count;
        if(ius>coord_interval_max)coord_interval_max=ius;}
    coord_last_report=now;
    uint64_t ns=0;absolutetime_to_nanoseconds(now-touch_frame_time,&ns);
    const uint64_t us=ns/1000;touch_latency_total+=us;if(us>touch_latency_max)touch_latency_max=us;
    if(!(++touch_latency_count&255u))printf("Diting touch latency: reports=%u irq=%u spurious=%u polls=%u read_mean_us=%llu read_max_us=%llu interval_mean_us=%llu interval_max_us=%llu IRQ=%u spi=%s/%u\n",
        touch_latency_count,touch_irqs,touch_spurious,coord_polls,touch_latency_total/touch_latency_count,touch_latency_max,
        coord_interval_count?coord_interval_total/coord_interval_count:0,coord_interval_max,touch_irq_armed,spi_transport_name,geni_actual_hz);
}


/* No firmware upload, FocalTech strap scan, GPU or other Redwood subsystem. */
static bool
bring_up(void)
{
    uint32_t spi_hz = 8000000;
    PE_parse_boot_argn("diting_touch_hz", &spi_hz, sizeof(spi_hz));
    if (spi_hz < 1000000) spi_hz = 1000000;
    if (spi_hz > 12000000) spi_hz = 12000000;
    gpio_output(PIN_RESET,true,PULL_UP,6);
    gpio_configure(PIN_INT,0,PULL_UP,6,false);
    if (clocks_on(spi_hz)) {
        const char *problem=geni_spi_init(spi_hz);
        if (!problem) {
            gpio_configure(PIN_MISO,1,PULL_NONE,6,false);
            gpio_configure(PIN_MOSI,1,PULL_NONE,6,false);
            gpio_configure(PIN_CLK,1,PULL_NONE,6,false);
            gpio_configure(PIN_CS,1,PULL_UP,6,false);
            spi_transfer=geni_transfer;
            if (goodix_probe(false)) return true;
            if (io_supply_on()) { touch_reset();if (goodix_probe(true)) return true; }
        } else printf("Diting touch: GENI unavailable: %s; use GPIO SPI\n",problem);
    }
    /* Only the two known data pins can exchange direction; no arbitrary scan. */
    spi_transfer=bitbang_transfer;
    for (unsigned attempt=0;attempt<2;attempt++) {
        bitbang_in=pins[attempt];bitbang_out=pins[1-attempt];bitbang_init();
        if (goodix_probe(false)) return true;
        if (attempt==0 && io_supply_on()) {touch_reset();if (goodix_probe(true)) return true;}
    }
    return false;
}

static void
touch_thread(__unused void *parameter, __unused wait_result_t result)
{
    tlmm=ml_io_map(TLMM_PHYS,TLMM_SIZE);gcc=ml_io_map(GCC_PHYS,GCC_SIZE);
    se=ml_io_map(SE_PHYS,SE_SIZE);rsc=ml_io_map(RSC_PHYS,RSC_SIZE);
    cmd_db=ml_io_map_wcomb(CMD_DB_PHYS,CMD_DB_SIZE);
    if (!tlmm || !gcc || !se || !rsc || !cmd_db || !bring_up()) {
        printf("Diting touch: Goodix initialization failed; normal UI boot continues\n");
        thread_terminate(current_thread());return;
    }
    /* A readable firmware identity does not mean the controller is scanning:
     * Android/REC may leave it in sleep or gesture mode. Diting's vendor
     * brl_resume() resets this already identified Berlin controller. Resume
     * before publishing input, and require the real firmware/report ABI again.
     * No firmware upload or fabricated contacts are used. */
    touch_reset();
    if (!goodix_probe(true)) {
        printf("Diting touch: controller did not resume after reset; SKIP\n");
        thread_terminate(current_thread());return;
    }
    /* Berlin-D leaves sensor 0xff until it owns a panel config. The vendor driver
     * uploads its cfg group when the stored config_id differs from the chip's; the
     * blob rides the generated ADT as goodix,cfg-data (a local artifact, never
     * committed), so its absence stays a normal unconfigured boot. */
    {
        DTEntry entry; void *blob = NULL; unsigned bloblen = 0;
        if (SecureDTLookupEntry(NULL, "/arm-io/diting-touch", &entry) == kSuccess &&
            SecureDTGetProperty(entry, "goodix,cfg-data", &blob, &bloblen) == kSuccess &&
            blob && bloblen >= 64 && 64u + le16((const uint8_t *)blob + 59) == bloblen) {
            uint32_t file_id = le32((const uint8_t *)blob + 30);
            if (file_id == ic_config_id) {
                printf("goodix: config %#x already loaded; no upload\n", file_id);
            } else {
                printf("goodix: config %#x for chip %#x via buffer %#x (max %u)\n", file_id,
                    ic_config_id, fw_buffer_address, fw_buffer_max_len);
                if (goodix_send_config(blob, bloblen)) {
                    touch_reset();
                    if (!goodix_probe(true)) {
                        printf("Diting touch: controller lost after config upload; SKIP\n");
                        thread_terminate(current_thread());return;
                    }
                } else {
                    printf("goodix: config upload failed; continuing unconfigured\n");
                }
            }
        }
    }
    /* Berlin-D leaves sensor 0xff until it owns a panel config. The vendor driver
     * uploads its cfg group when the stored config_id differs from the chip's; the
     * blob rides the generated ADT as goodix,cfg-data (a local artifact, never
     * committed), so its absence stays a normal unconfigured boot. */
    {
        DTEntry entry; void *blob = NULL; unsigned bloblen = 0;
        if (SecureDTLookupEntry(NULL, "/arm-io/diting-touch", &entry) == kSuccess &&
            SecureDTGetProperty(entry, "goodix,cfg-data", &blob, &bloblen) == kSuccess &&
            blob && bloblen >= 64 && 64u + le16((const uint8_t *)blob + 59) == bloblen) {
            uint32_t file_id = le32((const uint8_t *)blob + 30);
            if (file_id == ic_config_id) {
                printf("goodix: config %#x already loaded; no upload\n", file_id);
            } else {
                printf("goodix: config %#x for chip %#x via buffer %#x (max %u)\n", file_id,
                    ic_config_id, fw_buffer_address, fw_buffer_max_len);
                if (goodix_send_config(blob, bloblen)) {
                    touch_reset();
                    if (!goodix_probe(true)) {
                        printf("Diting touch: controller lost after config upload; SKIP\n");
                        thread_terminate(current_thread());return;
                    }
                } else {
                    printf("goodix: config upload failed; continuing unconfigured\n");
                }
            }
        }
    }
    /* Berlin-D leaves sensor 0xff until it owns a panel config. The vendor driver
     * uploads its cfg group when the stored config_id differs from the chip's; the
     * blob rides the generated ADT as goodix,cfg-data (a local artifact, never
     * committed), so its absence stays a normal unconfigured boot. */
    {
        DTEntry entry; void *blob = NULL; unsigned bloblen = 0;
        if (SecureDTLookupEntry(NULL, "/arm-io/diting-touch", &entry) == kSuccess &&
            SecureDTGetProperty(entry, "goodix,cfg-data", &blob, &bloblen) == kSuccess &&
            blob && bloblen >= 64 && 64u + le16((const uint8_t *)blob + 59) == bloblen) {
            uint32_t file_id = le32((const uint8_t *)blob + 30);
            if (file_id == ic_config_id) {
                printf("goodix: config %#x already loaded; no upload\n", file_id);
            } else {
                printf("goodix: config %#x for chip %#x via buffer %#x (max %u)\n", file_id,
                    ic_config_id, fw_buffer_address, fw_buffer_max_len);
                if (goodix_send_config(blob, bloblen)) {
                    touch_reset();
                    if (!goodix_probe(true)) {
                        printf("Diting touch: controller lost after config upload; SKIP\n");
                        thread_terminate(current_thread());return;
                    }
                } else {
                    printf("goodix: config upload failed; continuing unconfigured\n");
                }
            }
        }
    }
    /* Berlin-D leaves sensor 0xff until it owns a panel config. The vendor driver
     * uploads its cfg group when the stored config_id differs from the chip's; the
     * blob rides the generated ADT as goodix,cfg-data (a local artifact, never
     * committed), so its absence stays a normal unconfigured boot. */
    {
        DTEntry entry; void *blob = NULL; unsigned bloblen = 0;
        if (SecureDTLookupEntry(NULL, "/arm-io/diting-touch", &entry) == kSuccess &&
            SecureDTGetProperty(entry, "goodix,cfg-data", &blob, &bloblen) == kSuccess &&
            blob && bloblen >= 64 && 64u + le16((const uint8_t *)blob + 59) == bloblen) {
            uint32_t file_id = le32((const uint8_t *)blob + 30);
            if (file_id == ic_config_id) {
                printf("goodix: config %#x already loaded; no upload\n", file_id);
            } else {
                printf("goodix: config %#x for chip %#x via buffer %#x (max %u)\n", file_id,
                    ic_config_id, fw_buffer_address, fw_buffer_max_len);
                if (goodix_send_config(blob, bloblen)) {
                    touch_reset();
                    if (!goodix_probe(true)) {
                        printf("Diting touch: controller lost after config upload; SKIP\n");
                        thread_terminate(current_thread());return;
                    }
                } else {
                    printf("goodix: config upload failed; continuing unconfigured\n");
                }
            }
        }
    }
    /* Berlin-D leaves sensor 0xff until it owns a panel config. The vendor driver
     * uploads its cfg group when the stored config_id differs from the chip's; the
     * blob rides the generated ADT as goodix,cfg-data (a local artifact, never
     * committed), so its absence stays a normal unconfigured boot. */
    {
        DTEntry entry; void *blob = NULL; unsigned bloblen = 0;
        if (SecureDTLookupEntry(NULL, "/arm-io/diting-touch", &entry) == kSuccess &&
            SecureDTGetProperty(entry, "goodix,cfg-data", &blob, &bloblen) == kSuccess &&
            blob && bloblen >= 64 && 64u + le16((const uint8_t *)blob + 59) == bloblen) {
            uint32_t file_id = le32((const uint8_t *)blob + 30);
            if (file_id == ic_config_id) {
                printf("goodix: config %#x already loaded; no upload\n", file_id);
            } else {
                printf("goodix: config %#x for chip %#x via buffer %#x (max %u)\n", file_id,
                    ic_config_id, fw_buffer_address, fw_buffer_max_len);
                if (goodix_send_config(blob, bloblen)) {
                    touch_reset();
                    if (!goodix_probe(true)) {
                        printf("Diting touch: controller lost after config upload; SKIP\n");
                        thread_terminate(current_thread());return;
                    }
                } else {
                    printf("goodix: config upload failed; continuing unconfigured\n");
                }
            }
        }
    }
    /* Berlin-D leaves sensor 0xff until it owns a panel config. The vendor driver
     * uploads its cfg group when the stored config_id differs from the chip's; the
     * blob rides the generated ADT as goodix,cfg-data (a local artifact, never
     * committed), so its absence stays a normal unconfigured boot. */
    {
        DTEntry entry; void *blob = NULL; unsigned bloblen = 0;
        if (SecureDTLookupEntry(NULL, "/arm-io/diting-touch", &entry) == kSuccess &&
            SecureDTGetProperty(entry, "goodix,cfg-data", &blob, &bloblen) == kSuccess &&
            blob && bloblen >= 64 && 64u + le16((const uint8_t *)blob + 59) == bloblen) {
            uint32_t file_id = le32((const uint8_t *)blob + 30);
            if (file_id == ic_config_id) {
                printf("goodix: config %#x already loaded; no upload\n", file_id);
            } else {
                printf("goodix: config %#x for chip %#x via buffer %#x (max %u)\n", file_id,
                    ic_config_id, fw_buffer_address, fw_buffer_max_len);
                if (goodix_send_config(blob, bloblen)) {
                    touch_reset();
                    if (!goodix_probe(true)) {
                        printf("Diting touch: controller lost after config upload; SKIP\n");
                        thread_terminate(current_thread());return;
                    }
                } else {
                    printf("goodix: config upload failed; continuing unconfigured\n");
                }
            }
        }
    }
    const uint8_t clear=0;
    if (!goodix_write(touch_address,&clear,1)) {
        printf("Diting touch: cannot acknowledge resumed report; SKIP\n");
        thread_terminate(current_thread());return;
    }
    if (!diting_touch_device_ready()) {thread_terminate(current_thread());return;}
    printf("Diting touch: Berlin-D ready, SPI 0x%llx (%s/%u Hz) GPIO reset=%u irq=%u, axes %ux%u\n",
        se_phys,spi_transport_name,geni_actual_hz,PIN_RESET,PIN_INT,max_x,max_y);
    touch_irq_armed=arm_touch_irq();
    printf("Diting touch: IRQ240 GPIO21 falling-edge=%u; bounded polling fallback retained\n",touch_irq_armed);
    uint32_t use_thp = 0;
    PE_parse_boot_argn("diting_touch_thp",&use_thp,sizeof(use_thp));
    PE_parse_boot_argn("diting-touch-thp",&use_thp,sizeof(use_thp));
    PE_parse_boot_argn("diting-touch-thr",&thp_thr,sizeof(thp_thr));
    PE_parse_boot_argn("diting-touch-map",&thp_map,sizeof(thp_map));
    if (use_thp == 1 && frame_address && thp_tx >= 4 && thp_rx >= 4 &&
        (uint32_t)thp_tx * thp_rx <= THP_NODES_MAX &&
        frame_skip + 8 + (uint32_t)thp_tx * thp_rx * 2 <= GOODIX_MAX_READ) {
        const uint8_t diff = 0x82, zero = 0;
        thp_mode = goodix_send_cmd(0x90, &diff, 1);
        (void)goodix_write(frame_address, &zero, 1);
        printf("goodix: THP mode %u; matrix %ux%u frame %#x skip %u thr %u\n",
            thp_mode, thp_tx, thp_rx, frame_address, frame_skip, thp_thr);
    } else {
        printf("goodix: coordinate mode active (thp_req=%u frame=%#x matrix=%ux%u)\n",
            use_thp, frame_address, thp_tx, thp_rx);
    }
    unsigned idle=0, thp_wait=0;
    uint32_t thp_irq_base = touch_irqs;
    for (;;) {
        if (thp_mode) {
            const uint64_t irq_stamp = __atomic_exchange_n(&touch_irq_time, 0, __ATOMIC_ACQ_REL);
            const uint64_t t0 = mach_absolute_time();
            touch_frame_time = irq_stamp ? irq_stamp : t0;
            ++thp_polls;
            int fr = thp_frame();
            if (fr > 0) {
                const uint64_t t1 = mach_absolute_time();
                uint64_t ns = 0;
                absolutetime_to_nanoseconds(t1 - t0, &ns);
                const uint64_t us = ns / 1000;
                touch_latency_total += us;
                if (us > touch_latency_max) touch_latency_max = us;
                if (thp_last_frame) {
                    uint64_t ins = 0;
                    absolutetime_to_nanoseconds(t1 - thp_last_frame, &ins);
                    const uint64_t ius = ins / 1000;
                    thp_interval_total += ius;
                    ++thp_interval_count;
                    if (ius > thp_interval_max) thp_interval_max = ius;
                }
                thp_last_frame = t1;
                if (!(++touch_latency_count & 255u)) {
                    printf("Diting touch THP latency: frames=%u irq=%u polls=%u read_mean_us=%llu read_max_us=%llu interval_mean_us=%llu interval_max_us=%llu proc_mean_us=%llu proc_max_us=%llu spi=%s/%u\n",
                        thp_frames, touch_irqs, thp_polls, touch_latency_total / touch_latency_count,
                        touch_latency_max,
                        thp_interval_count ? thp_interval_total / thp_interval_count : 0, thp_interval_max,
                        touch_latency_count ? thp_proc_total / touch_latency_count : 0, thp_proc_max,
                        spi_transport_name, geni_actual_hz);
                }
                thp_wait = 0;
                const uint64_t t2 = mach_absolute_time();
                thp_process();
                uint64_t pns = 0;
                absolutetime_to_nanoseconds(mach_absolute_time() - t2, &pns);
                const uint64_t pus = pns / 1000;
                thp_proc_total += pus;
                if (pus > thp_proc_max) thp_proc_max = pus;
            } else if (fr < 0) {
                printf("goodix: THP frame read failed; coordinate mode restored\n");
                const uint8_t coord = 0;
                (void)goodix_send_cmd(0x90, &coord, 1);
                thp_mode = 0;
            } else if (!thp_frames && ++thp_wait > 750) {
                printf("goodix: no THP frames in %u polls; coordinate mode restored\n", thp_wait);
                const uint8_t coord = 0;
                (void)goodix_send_cmd(0x90, &coord, 1);
                thp_mode = 0;
            }
            if (thp_mode) {
                bool irq_drives_thp = touch_irq_armed && (touch_irqs != thp_irq_base);
                touch_wait(irq_drives_thp ? 8 : 2);
                continue;
            }
        }
        if (__atomic_load_n(&touch_pending,__ATOMIC_ACQUIRE)||!gpio_get(PIN_INT)||++idle>=10) {idle=0;goodix_poll();}
        touch_wait(!gpio_get(PIN_INT) ? 1 : (touch_irq_armed ? 8 : 2));
    }
}

void
diting_goodix_init(void)
{
    uint32_t enabled=0;thread_t worker;
    if (!PE_parse_boot_argn("diting_touch",&enabled,sizeof(enabled)) || !enabled) return;
    if (!configuration()) {printf("Diting touch: missing/incompatible Diting DT; SKIP\n");return;}
    printf("Diting touch: DT ok; SPI %#llx pins %u/%u/%u/%u int=%u reset=%u max %ux%u\n",
        se_phys,pins[0],pins[1],pins[2],pins[3],PIN_INT,PIN_RESET,max_x,max_y);
    if (kernel_thread_start(touch_thread,NULL,&worker)==KERN_SUCCESS) thread_deallocate(worker);
}
#endif /* DITING */
