/*
 * TODO:
 *   - implement external pin support
 */

#define DEBUG

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/timecounter.h>
#include <linux/clocksource.h>
#include <linux/workqueue.h>
#include <linux/ptp_clock_kernel.h>
#include <asm/io.h>

#define MVNETA_PTP_CMD_REG   (0U)
#define         MVNETA_PTP_CMD_PORT_SHIFT (8U)
#define         MVNETA_PTP_CMD_OP_SHIFT   (12U)
#define         MVNETA_PTP_CMD_OP_WR      (3U << MVNETA_PTP_CMD_OP_SHIFT)
#define         MVNETA_PTP_CMD_OP_RD      (4U << MVNETA_PTP_CMD_OP_SHIFT)
#define         MVNETA_PTP_CMD_OP_RDNINC  (6U << MVNETA_PTP_CMD_OP_SHIFT)
#define         MVNETA_PTP_CMD_BUSY       BIT(15)
#define MVNETA_PTP_DATA_REG  (0x8U)
#define MVNETA_PTP_RESET_REG (0x10U)
#define         MVNETA_PTP_RESET_EN       BIT(1)
#define MVNETA_PTP_CLOCK_REG (0x18U)
#define         MVNETA_PTP_CLOCK_EXT      BIT(1)

/*
 * PTP port registers.
 */
#define MVNETA_PTP_PORT_CFG0_REG                 (0x0U)
#define         MVNETA_PTP_PORT_CFG0_DIS         BIT(1)
#define         MVNETA_PTP_PORT_CFG0_TSPEC_SHIFT (12)

/*
 * PTP global registers.
 */
#define MVNETA_PTP_GLOB_CFG0_REG                 (0x0U)

/*
 * Time Application Interface registers.
 */
#define MVNETA_TAI_GLOB_CFG_REG     (0x1U)
#define MVNETA_TAI_GLOB_CFG3_REG    (0x6U)
/* Oddly enough PTP Global Status1 registers belong to TAI. */
#define MVNETA_PTP_GLOB_STAT1_LOREG (0xeU)
#define MVNETA_PTP_GLOB_STAT1_HIREG (0xfU)
#define MVNETA_PTP_GLOB_STAT1_BITS  (sizeof(u32) * 8)

#define MHZ               (1000000U)
#define MVNETA_PTP_CLK_HZ (125U * MHZ)
#define MVNETA_PTP_CLK_NS (NSEC_PER_SEC / MVNETA_PTP_CLK_HZ)
#define MVNETA_PTP_CYCLE_LOG2_MULT_MAX \
	((sizeof(cycle_t) * 8) - MVNETA_PTP_GLOB_STAT1_BITS)
#define MVNETA_PTP_CYCLE_SHIFT            \
	(MVNETA_PTP_CYCLE_LOG2_MULT_MAX - \
	 (constant_fls(2 * MVNETA_PTP_CLK_NS) - 1))
#define MVNETA_PTP_CYCLE_MASK \
	CYCLECOUNTER_MASK(MVNETA_PTP_GLOB_STAT1_BITS)

struct mvneta_ptp {
	void __iomem          *regs;
	rwlock_t               lock;
	struct cyclecounter    cycle;
	struct timecounter     time;
	u64                    mult;
	struct ptp_clock_info  info;
	struct ptp_clock      *clock;
	unsigned long          delay;
	struct delayed_work    unwrap;
	struct device         *dev;
};

void mvneta_fill_hwtstamp(struct mvneta_ptp           *ptp,
			  u64                          timestamp,
			  struct skb_shared_hwtstamps *hwtstamps)
{
	u64 nsec;

	read_lock(&ptp->lock);
	nsec = timecounter_cyc2time(&ptp->time, timestamp);
	read_unlock(&ptp->lock);

	hwts->hwtstamp = ns_to_ktime(nsec);
}
EXPORT_SYMBOL(mvneta_fill_hwtstamp);

static int mvneta_setup_ptp(const struct mvneta_ptp *ptp,
			    __be16                   ethtype,
			    u16                      msgids,
			    u16                      arrptr)
{
	/* Setup ethertype matching. */
	int err = mvneta_write_glob(ptp, port, MVNETA_PTP_GLOB_CFG0_REG,
				    ethtype)
	if (err)
		return err;

	/* Setup PTP message ids matching mask. */
	err = mvneta_write_glob(ptp, port, MVNETA_PTP_GLOB_CFG1_REG, msgids)
	if (err)
		return err;

	/* Setup PTP arrival timestamping counter mask. */
	return mvneta_write_glob(ptp, port, MVNETA_PTP_GLOB_CFG2_REG, arrptr)
}

int mvneta_setup_port_ptp(const struct mvneta_ptp *ptp,
			  u8                       port,
			  u8                       tspec)
{
	/* Disable port PTP logic first. */
	int err = mvneta_write_ptp(ptp, port, MVNETA_PTP_PORT_CFG0_REG,
				   MVNETA_PTP_PORT_CFG0_DIS);

	if (err)
		return err;

	if (tspec > 0xf)
		/* Transport Specific field is 4 bits wide. */
		return -EINVAL;

	/*
	 * Enable generation of interrupt upon hardware timestamping of outgoing
	 * and incoming frames.
	 */
	err = mvneta_write_ptp(ptp, port, MVNETA_PTP_PORT_CFG2_REG,
			        MVNETA_PTP_GLOB_CFG2_DEPIRQ |
			        MVNETA_PTP_GLOB_CFG2_ARRIRQ);
	if (err)
		return err;

	/*
	 * Now that everything is ready, setup transport specific field then
	 * enable port PTP logic.
	 */
	return mvneta_write_ptp(ptp, port, MVNETA_PTP_PORT_CFG0_REG,
			        tspec << MVNETA_PTP_PORT_CFG0_TSPEC_SHIFT);
}
EXPORT_SYMBOL(mvneta_setup_txtstamp);

static int mvneta_wait_ptp(const struct mvneta_ptp *ptp)
{
	u16                 cmd = readw(ptp->regs + MVNETA_PTP_CMD_REG);
	const unsigned long tmout = jiffies + 1;

	while (cmd & MVNETA_PTP_CMD_BUSY) {
		if (time_after(jiffies, tmout)) {
			dev_warn(ptp->dev, "stalled\n");
			return -EBUSY;
		}

		cond_resched();
		cmd = readw(ptp->regs + MVNETA_PTP_CMD_REG);
	}

	return 0;
}

static int mvneta_read_ptp(const struct mvneta_ptp *ptp,
			   u8                       port,
			   u8                       offset,
			   u16                     *data)
{
	BUG_ON(port > 0xf);
	BUG_ON(offset > 0x1f);

	if (mvneta_wait_ptp(ptp))
		return -EBUSY;
	writew_relaxed(MVNETA_PTP_CMD_BUSY | MVNETA_PTP_CMD_OP_RD |
		       (port << MVNETA_PTP_CMD_PORT_SHIFT) | offset,
		       ptp->regs + MVNETA_PTP_CMD_REG);
	__iowmb();

	if (mvneta_wait_ptp(ptp))
		return -EBUSY;

	*data = readw(ptp->regs + MVNETA_PTP_DATA_REG);
	return 0;
}

static int mvneta_write_ptp(const struct mvneta_ptp *ptp,
			    u8                       port,
			    u8                       offset,
			    u16                      data)
{
	BUG_ON(port > 0xf);
	BUG_ON(offset > 0x1f);

	if (mvneta_wait_ptp(ptp))
		return -EBUSY;

	writew_relaxed(data, ptp->regs + MVNETA_PTP_DATA_REG);
	writew_relaxed(MVNETA_PTP_CMD_BUSY | MVNETA_PTP_CMD_OP_WR |
		       (port << MVNETA_PTP_CMD_PORT_SHIFT) | offset,
		       ptp->regs + MVNETA_PTP_CMD_REG);
	__iowmb();

	return 0;
}

/*
 * Read a Time Application Interface register.
 */
static int mvneta_read_tai(const struct mvneta_ptp *ptp,
			   u8                       offset,
			   u16                     *data)
{
	return mvneta_read_ptp(ptp, 0xe, offset, data);
}

/*
 * Write a Time Application Interface register.
 */
static int mvneta_write_tai(const struct mvneta_ptp *ptp,
			    u8                       offset,
			    u16                      data)
{
	return mvneta_write_ptp(ptp, 0xe, offset, data);
}

/*
 * Read a GLOBal PTP register.
 */
static int mvneta_read_glob(const struct mvneta_ptp *ptp,
			    u8                       offset,
			    u16                     *data)
{
	return mvneta_read_ptp(ptp, 0xf, offset, data);
}

/*
 * Write a GLOBal PTP register.
 */
static int mvneta_write_glob(const struct mvneta_ptp *ptp,
			     u8                       offset,
			     u16                      data)
{
	return mvneta_write_ptp(ptp, 0xf, offset, data);
}

static cycle_t mvneta_read_ptp_cycles(const struct cyclecounter *cc)
{
	const struct mvneta_ptp *ptp = container_of(cc, struct mvneta_ptp,
						    cycle);
	u16                      lo, hi;

	if (mvneta_read_tai(ptp, MVNETA_PTP_GLOB_STAT1_LOREG, &lo) ||
	    mvneta_read_tai(ptp, MVNETA_PTP_GLOB_STAT1_HIREG, &hi))
		return 0;

	return (cycle_t)hi << 16 | lo;
}

static int mvneta_adj_ptp_freq(struct ptp_clock_info *info,
			       s32                    delta)
{
	const bool         neg = delta < 0;
	struct mvneta_ptp *ptp = container_of(info, struct mvneta_ptp, info);
	u32                mult;

	if (neg)
		delta = -delta;
	mult = div_u64(ptp->mult * delta, 1000000000U);
	mult = neg ? ptp->mult - mult : ptp->mult + mult;

	write_lock(&ptp->lock);
	timecounter_read(&ptp->time);
	ptp->cycle.mult = mult;
	write_unlock(&ptp->lock);

	dev_dbg(ptp->dev, "adj freq: %d ppb\n", delta);

	return 0;
}

static int mvneta_adj_ptp_time(struct ptp_clock_info *info,
			       s64                    delta)
{
	struct mvneta_ptp *ptp = container_of(info, struct mvneta_ptp, info);

	write_lock(&ptp->lock);
	timecounter_adjtime(&ptp->time, delta);
	write_unlock(&ptp->lock);

	dev_dbg(ptp->dev, "adj time: %lld ns\n", delta);

	return 0;
}

static int mvneta_get_ptp_time(struct ptp_clock_info *info,
			       struct timespec64     *tp)
{
	struct mvneta_ptp *ptp = container_of(info, struct mvneta_ptp, info);
	u64                ns;

	write_lock(&ptp->lock);
	ns = timecounter_read(&ptp->time);
	write_unlock(&ptp->lock);

	*tp = ns_to_timespec64(ns);
	return 0;
}

static int mvneta_set_ptp_time(struct ptp_clock_info   *info,
			       const struct timespec64 *ts)
{
	struct mvneta_ptp *ptp = container_of(info, struct mvneta_ptp, info);
	u64                ns = timespec64_to_ns(ts);

	write_lock(&ptp->lock);
	timecounter_init(&ptp->time, &ptp->cycle, ns);
	write_unlock(&ptp->lock);

	dev_dbg(ptp->dev, "set time to %lld.%ld\n", ts->tv_sec, ts->tv_nsec);

	return 0;
}

static int mvneta_enable_ptp(struct ptp_clock_info    *info,
			     struct ptp_clock_request *req,
			     int                       on)
{
	return -EOPNOTSUPP;
}

static const struct ptp_clock_info mvneta_ptp_info = {
	.owner      = THIS_MODULE,
	.name       = "mvneta_ptp_clock",
	.max_adj    = 999999999,
	.n_alarm    = 0,
	.n_ext_ts   = 0,
	.n_per_out  = 0,
	.n_pins     = 0,
	.pps        = 0,
	.adjfreq    = mvneta_adj_ptp_freq,
	.adjtime    = mvneta_adj_ptp_time,
	.gettime64  = mvneta_get_ptp_time,
	.settime64  = mvneta_set_ptp_time,
	.enable     = mvneta_enable_ptp,
};

static void mvneta_unwrap_ptp(struct work_struct *work)
{
	struct delayed_work *unwrap = to_delayed_work(work);
	struct mvneta_ptp   *ptp = container_of(unwrap, struct mvneta_ptp,
					        unwrap);

	write_lock(&ptp->lock);
	timecounter_read(&ptp->time);
	write_unlock(&ptp->lock);

	schedule_delayed_work(&ptp->unwrap, ptp->delay);

	dev_dbg(ptp->dev, "counter reloaded\n");
}

static struct mvneta_ptp mvneta_ptp_dev;

static int mvneta_probe_ptp(struct platform_device *pdev)
{
	struct resource   *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct mvneta_ptp *ptp = &mvneta_ptp_dev;

	/* Remap registers space. */
	ptp->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(ptp->regs)) {
		dev_err(&pdev->dev, "registers mapping failed\n");
		return PTR_ERR(ptp->regs);
	}

	rwlock_init(&ptp->lock);

	/*
	 * We use the internal clock only. It toggles at 125 mHz, corresponding
	 * to an 8 nano-seconds long counter increment. A simple x8 multiplier
	 * is enough, i.e. no fractional part.
	 * To ensure proper cyclecounter a2's substraction, set a 32 bits mask
	 * since the global PTP counter is 32 bits wide.
	 */
	ptp->cycle.read = mvneta_read_ptp_cycles;
	ptp->cycle.mask = MVNETA_PTP_CYCLE_MASK;
	ptp->mult = clocksource_hz2mult(MVNETA_PTP_CLK_HZ,
					MVNETA_PTP_CYCLE_SHIFT);
	ptp->cycle.mult = ptp->mult;
	ptp->cycle.shift = MVNETA_PTP_CYCLE_SHIFT;
	timecounter_init(&ptp->time, &ptp->cycle,
			 ktime_to_ns(ktime_get_real()));

	/* Reset PTP core */
	writel_relaxed(0, ptp->regs + MVNETA_PTP_RESET_REG);
	/* Select internal 125mHz PTP clock */
	writel(0, ptp->regs + MVNETA_PTP_CLOCK_REG);
	/* De-reset PTP core */
	writel(MVNETA_PTP_RESET_EN, ptp->regs + MVNETA_PTP_RESET_REG);

	/*
	 * Calculate PTP timestamping counter overflowing period in number of
	 * jiffies and setup periodic timecounter reloading work.
	 */
	ptp->delay = nsecs_to_jiffies(MVNETA_PTP_CLK_NS *
				      (~((u64)0) >>
				       MVNETA_PTP_GLOB_STAT1_BITS)) / 2;
	INIT_DELAYED_WORK(&ptp->unwrap, mvneta_unwrap_ptp);
	ptp->dev = &pdev->dev;
	timecounter_read(&ptp->time);

	/*
	 * Register PTP clock.
	 */
	ptp->info = mvneta_ptp_info;
	ptp->clock = ptp_clock_register(&ptp->info, &pdev->dev);
	if (!IS_ERR(ptp->clock)) {
	 	/*
		 * Launch periodic work to ensure timecounter is checked at
		 * least once every wrap around.
		 */
		schedule_delayed_work(&ptp->unwrap, ptp->delay);
		dev_dbg(&pdev->dev, "registered\n");
		return 0;
	}

	ptp->clock = NULL;
	dev_err(&pdev->dev, "setup failed\n");
	return PTR_ERR(ptp->clock);
}

static int mvneta_remove_ptp(struct platform_device *pdev)
{
	struct mvneta_ptp *ptp = &mvneta_ptp_dev;

	cancel_delayed_work_sync(&ptp->unwrap);
	ptp_clock_unregister(ptp->clock);

	return 0;
}

static const struct of_device_id mvneta_ptp_match[] = {
	{ .compatible = "marvell,armada-370-neta-ptp" },
	{ .compatible = "marvell,armada-xp-neta-ptp" },
	{ }
};
MODULE_DEVICE_TABLE(of, mvneta_ptp_match);

static struct platform_driver mvneta_ptp_driver = {
	.probe  = mvneta_probe_ptp,
	.remove = mvneta_remove_ptp,
	.driver = {
		.name = "mvneta_ptp",
		.of_match_table = mvneta_ptp_match,
	},
};

module_platform_driver(mvneta_ptp_driver);

MODULE_DESCRIPTION("Marvell NETA PTP Driver");
MODULE_AUTHOR("Gregor Boirie <gregor.boirie@free.fr>");
MODULE_LICENSE("GPL");
