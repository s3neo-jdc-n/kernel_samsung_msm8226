/* drivers/cpufreq/qcom-cpufreq.c
 *
 * MSM architecture cpufreq driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2015, The Linux Foundation. All rights reserved.
 * Author: Mike A. Chan <mikechan@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <trace/events/power.h>
#include <mach/socinfo.h>
#include <soc/qcom/cpufreq.h>

#include "../../arch/arm/mach-msm/acpuclock.h"

static struct clk *cpu_clk[NR_CPUS];
static struct clk *l2_clk;
static struct cpufreq_frequency_table *freq_table;
static bool is_clk;
static bool is_sync;

struct cpufreq_work_struct {
	struct work_struct work;
	struct cpufreq_policy *policy;
	struct completion complete;
	int frequency;
	unsigned int index;
	int status;
};

static DEFINE_PER_CPU(struct cpufreq_work_struct, cpufreq_work);
static struct workqueue_struct *msm_cpufreq_wq;
#ifdef CONFIG_SEC_DVFS
static unsigned int upper_limit_freq[NR_CPUS] = {0, 0, 0, 0};
static unsigned int lower_limit_freq[NR_CPUS];
static unsigned int cpuinfo_max_freq;
static unsigned int cpuinfo_min_freq;

unsigned int get_cpu_min_lock(unsigned int cpu)
{
	if (cpu >= 0 && cpu < NR_CPUS)
		return lower_limit_freq[cpu];
	else
		return 0;
}
EXPORT_SYMBOL(get_cpu_min_lock);

unsigned int get_min_lock(void)
{
	unsigned int cpu;
	unsigned int min = UINT_MAX;
	
	for_each_possible_cpu(cpu) {
		if (min > lower_limit_freq[cpu] 
			&& lower_limit_freq[cpu] > 0)
				min = lower_limit_freq[cpu];
	}
	if (min == UINT_MAX)
		min = 0;

	return min;
}
EXPORT_SYMBOL(get_min_lock);

unsigned int get_cpu_max_lock(unsigned int cpu)
{
	if (cpu >= 0 && cpu < NR_CPUS)
		return upper_limit_freq[cpu];
	else
		return 0;
}
EXPORT_SYMBOL(get_cpu_max_lock);

unsigned int get_max_lock(void)
{
	unsigned int cpu;
	unsigned int max = 0;

	for_each_possible_cpu(cpu) {
		if (max < upper_limit_freq[cpu])
			max = upper_limit_freq[cpu];
	}

	return max;
}
EXPORT_SYMBOL(get_max_lock);

void set_cpu_min_lock(unsigned int cpu, int freq)
{
	if (cpu >= 0 && cpu < NR_CPUS) {
		if (freq <= MIN_FREQ_LIMIT || freq > MAX_FREQ_LIMIT)
			lower_limit_freq[cpu] = 0;
		else
			lower_limit_freq[cpu] = freq;
	}
}
EXPORT_SYMBOL(set_cpu_min_lock);

void set_min_lock(int freq)
{
	unsigned int cpu;
	unsigned l_freq = 0;

	if (freq <= MIN_FREQ_LIMIT)
		l_freq = 0;
	else if (freq > MAX_FREQ_LIMIT)
		l_freq = 0;
	else
		l_freq = freq;

	for_each_possible_cpu(cpu) {
		lower_limit_freq[cpu] = l_freq;
	}
}
EXPORT_SYMBOL(set_min_lock);

void set_cpu_max_lock(unsigned int cpu, int freq)
{
	if (cpu >= 0 && cpu < NR_CPUS) {
		if (freq < MIN_FREQ_LIMIT || freq >= MAX_FREQ_LIMIT)
			upper_limit_freq[cpu] = 0;
		else
			upper_limit_freq[cpu] = freq;
	}
}
EXPORT_SYMBOL(set_cpu_max_lock);

void set_max_lock(int freq)
{
	unsigned int cpu;
	unsigned l_freq = 0;

	if (freq < MIN_FREQ_LIMIT)
		l_freq = 0;
	else if (freq >= MAX_FREQ_LIMIT)
		l_freq = 0;
	else
		l_freq = freq;

	for_each_possible_cpu(cpu) {
		upper_limit_freq[cpu] = l_freq;
	}
}
EXPORT_SYMBOL(set_max_lock);

int get_max_freq(void)
{
	return cpuinfo_max_freq;
}
EXPORT_SYMBOL(get_max_freq);

int get_min_freq(void)
{
	return cpuinfo_min_freq;
}
EXPORT_SYMBOL(get_min_freq);
#endif
struct cpufreq_suspend_t {
	struct mutex suspend_mutex;
	int device_suspended;
};

static DEFINE_PER_CPU(struct cpufreq_suspend_t, cpufreq_suspend);

static int set_cpu_freq(struct cpufreq_policy *policy, unsigned int new_freq,
			unsigned int index)
{
	int ret = 0;
	struct cpufreq_freqs freqs;
#ifdef CONFIG_SEC_DVFS
	unsigned int ll_freq = lower_limit_freq[policy->cpu];
	unsigned int ul_freq = upper_limit_freq[policy->cpu];

	if (ll_freq || ul_freq) {
		unsigned int t_freq = new_freq;

		if (ll_freq && new_freq < ll_freq)
			t_freq = ll_freq;

		if (ul_freq && new_freq > ul_freq)
			t_freq = ul_freq;

		new_freq = t_freq;

		if (new_freq < policy->min)
			new_freq = policy->min;
		if (new_freq > policy->max)
			new_freq = policy->max;
	}
	if (new_freq == policy->cur)
		return 0;
#endif

	freqs.old = policy->cur;
	freqs.new = new_freq;
	freqs.cpu = policy->cpu;

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

	if (is_clk) {
		unsigned long rate = new_freq * 1000;
		rate = clk_round_rate(cpu_clk[policy->cpu], rate);
		ret = clk_set_rate(cpu_clk[policy->cpu], rate);
	} else {
		ret = acpuclk_set_rate(policy->cpu, new_freq, SETRATE_CPUFREQ);
	}

	if (!ret)
		cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);

	return ret;

}

static void set_cpu_work(struct work_struct *work)
{
	struct cpufreq_work_struct *cpu_work =
		container_of(work, struct cpufreq_work_struct, work);

	cpu_work->status = set_cpu_freq(cpu_work->policy, cpu_work->frequency,
					cpu_work->index);
	complete(&cpu_work->complete);
}

static int msm_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	int ret = 0;
	int index;
	struct cpufreq_frequency_table *table;

	struct cpufreq_work_struct *cpu_work = NULL;

	mutex_lock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);

	if (target_freq == policy->cur)
		goto done;

	if (per_cpu(cpufreq_suspend, policy->cpu).device_suspended) {
		pr_debug("cpufreq: cpu%d scheduling frequency change "
				"in suspend.\n", policy->cpu);
		ret = -EFAULT;
		goto done;
	}

	table = cpufreq_frequency_get_table(policy->cpu);
	if (!table) {
		pr_err("cpufreq: Failed to get frequency table for CPU%u\n",
		       policy->cpu);
		ret = -ENODEV;
		goto done;
	}
	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
			&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		ret = -EINVAL;
		goto done;
	}

	pr_debug("CPU[%d] target %d relation %d (%d-%d) selected %d\n",
		policy->cpu, target_freq, relation,
		policy->min, policy->max, table[index].frequency);

	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	cpu_work->policy = policy;
	cpu_work->frequency = table[index].frequency;
	cpu_work->index = table[index].driver_data;
	cpu_work->status = -ENODEV;

	cancel_work_sync(&cpu_work->work);
	INIT_COMPLETION(cpu_work->complete);
	queue_work_on(policy->cpu, msm_cpufreq_wq, &cpu_work->work);
	wait_for_completion(&cpu_work->complete);

	ret = cpu_work->status;

done:
	mutex_unlock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);
	return ret;
}

static int msm_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
			policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int msm_cpufreq_get_freq(unsigned int cpu)
{
#if defined(CONFIG_ARCH_MSM8226) || defined(CONFIG_ARCH_MSM8610)
	if (is_clk && is_sync)
		cpu = 0;
#endif

	if (is_clk)
		return clk_get_rate(cpu_clk[cpu]) / 1000;

	return acpuclk_get_rate(cpu);
}

static int __cpuinit msm_cpufreq_init(struct cpufreq_policy *policy)
{
	int cur_freq;
	int index;
	int ret = 0;
	struct cpufreq_frequency_table *table;
	struct cpufreq_work_struct *cpu_work = NULL;

	table = cpufreq_frequency_get_table(policy->cpu);
	if (table == NULL)
		return -ENODEV;
	/*
	 * In 8625, 8610, and 8226 both cpu core's frequency can not
	 * be changed independently. Each cpu is bound to
	 * same frequency. Hence set the cpumask to all cpu.
	 */
	if (cpu_is_msm8625() || cpu_is_msm8625q() || cpu_is_msm8226()
		|| cpu_is_msm8610() || (is_clk && is_sync))
		cpumask_setall(policy->cpus);

#if defined(CONFIG_ARCH_MSM8226) || defined(CONFIG_ARCH_MSM8610)
	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	INIT_WORK(&cpu_work->work, set_cpu_work);
	init_completion(&cpu_work->complete);

	/* synchronous cpus share the same policy */
	if (!cpu_clk[policy->cpu])
		return 0;
#endif

	if (cpufreq_frequency_table_cpuinfo(policy, table)) {		
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
		policy->cpuinfo.min_freq = CONFIG_MSM_CPU_FREQ_MIN;
		policy->cpuinfo.max_freq = CONFIG_MSM_CPU_FREQ_MAX;
#endif
	}
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
	policy->min = CONFIG_MSM_CPU_FREQ_MIN;
	policy->max = CONFIG_MSM_CPU_FREQ_MAX;
#else
	policy->min = MIN_FREQ_LIMIT;
	policy->max = MAX_FREQ_LIMIT;
#endif

	if (is_clk)
		cur_freq = clk_get_rate(cpu_clk[policy->cpu])/1000;
	else
		cur_freq = acpuclk_get_rate(policy->cpu);

	if (cpufreq_frequency_table_target(policy, table, cur_freq,
	    CPUFREQ_RELATION_H, &index) &&
	    cpufreq_frequency_table_target(policy, table, cur_freq,
	    CPUFREQ_RELATION_L, &index)) {
		pr_info("cpufreq: cpu%d at invalid freq: %d\n",
				policy->cpu, cur_freq);
		return -EINVAL;
	}
	/*
	 * Call set_cpu_freq unconditionally so that when cpu is set to
	 * online, frequency limit will always be updated.
	 */
	ret = set_cpu_freq(policy, table[index].frequency, table[index].driver_data);
	if (ret)
		return ret;
	pr_debug("cpufreq: cpu%d init at %d switching to %d\n",
			policy->cpu, cur_freq, table[index].frequency);
	policy->cur = table[index].frequency;

	policy->cpuinfo.transition_latency =
		acpuclk_get_switch_time() * NSEC_PER_USEC;

#if !defined(CONFIG_ARCH_MSM8226) && !defined(CONFIG_ARCH_MSM8610)
	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	INIT_WORK(&cpu_work->work, set_cpu_work);
	init_completion(&cpu_work->complete);
#endif

	return 0;
}

static int __cpuinit msm_cpufreq_cpu_callback(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	int rc;

	switch (action & ~CPU_TASKS_FROZEN) {

	case CPU_DYING:
		if (is_clk) {
			clk_disable(cpu_clk[cpu]);
			clk_disable(l2_clk);
		}
		break;
	/*
	 * Scale down clock/power of CPU that is dead and scale it back up
	 * before the CPU is brought up.
	 */
	case CPU_DEAD:
	case CPU_UP_CANCELED:
		if (is_clk) {
			clk_unprepare(cpu_clk[cpu]);
			clk_unprepare(l2_clk);
		}
		break;
	case CPU_UP_PREPARE:
		if (is_clk) {
			rc = clk_prepare_enable(l2_clk);
			if (rc < 0)
				return NOTIFY_BAD;
			rc = clk_prepare_enable(cpu_clk[cpu]);
			if (rc < 0)
				return NOTIFY_BAD;
			rc = clk_prepare(cpu_clk[cpu]);
			if (rc < 0) {
				clk_unprepare(l2_clk);
				return NOTIFY_BAD;
			}
		}
		break;

	case CPU_STARTING:
		if (is_clk) {
			rc = clk_enable(l2_clk);
			if (rc < 0)
				return NOTIFY_BAD;
			rc = clk_enable(cpu_clk[cpu]);
			if (rc) {
				clk_disable(l2_clk);
				return NOTIFY_BAD;
			}
		}
		break;

	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block __refdata msm_cpufreq_cpu_notifier = {
	.notifier_call = msm_cpufreq_cpu_callback,
};

static int msm_cpufreq_suspend(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		mutex_lock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
		per_cpu(cpufreq_suspend, cpu).device_suspended = 1;
		mutex_unlock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
	}

	return NOTIFY_DONE;
}

static int msm_cpufreq_resume(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

	return NOTIFY_DONE;
}

static int msm_cpufreq_pm_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	switch (event) {
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		return msm_cpufreq_resume();
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		return msm_cpufreq_suspend();
	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block msm_cpufreq_pm_notifier = {
	.notifier_call = msm_cpufreq_pm_event,
};

static struct freq_attr *msm_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver msm_cpufreq_driver = {
	/* lps calculations are handled here. */
	.flags		= CPUFREQ_STICKY | CPUFREQ_CONST_LOOPS,
	.init		= msm_cpufreq_init,
	.verify		= msm_cpufreq_verify,
	.target		= msm_cpufreq_target,
	.get		= msm_cpufreq_get_freq,
	.name		= "msm",
	.attr		= msm_freq_attr,
};

#define PROP_TBL "qcom,cpufreq-table"
static int cpufreq_parse_dt(struct device *dev)
{
	int ret, nf, i;
	u32 *data;

	/* Parse list of usable CPU frequencies. */
	if (!of_find_property(dev->of_node, PROP_TBL, &nf))
		return -EINVAL;
	nf /= sizeof(*data);

	if (nf == 0)
		return -EINVAL;

	data = devm_kzalloc(dev, nf * sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = of_property_read_u32_array(dev->of_node, PROP_TBL, data, nf);
	if (ret)
		return ret;

	/* Allocate all data structures. */
	freq_table = devm_kzalloc(dev, (nf + 1) * sizeof(*freq_table),
				  GFP_KERNEL);
	if (!freq_table)
		return -ENOMEM;

	for (i = 0; i < nf; i++) {
		unsigned long f;

		f = clk_round_rate(cpu_clk[0], data[i] * 1000);
		if (IS_ERR_VALUE(f))
			break;
		f /= 1000;

		/*
		 * Check if this is the last feasible frequency in the table.
		 *
		 * The table listing frequencies higher than what the HW can
		 * support is not an error since the table might be shared
		 * across CPUs in different speed bins. It's also not
		 * sufficient to check if the rounded rate is lower than the
		 * requested rate as it doesn't cover the following example:
		 *
		 * Table lists: 2.2 GHz and 2.5 GHz.
		 * Rounded rate returns: 2.2 GHz and 2.3 GHz.
		 *
		 * In this case, we can CPUfreq to use 2.2 GHz and 2.3 GHz
		 * instead of rejecting the 2.5 GHz table entry.
		 */
		if (i > 0 && f <= freq_table[i-1].frequency)
			break;

		freq_table[i].driver_data = i;
		freq_table[i].frequency = f;
	}

	freq_table[i].driver_data = i;
	freq_table[i].frequency = CPUFREQ_TABLE_END;

	devm_kfree(dev, data);

	return 0;
}

static int __init msm_cpufreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	char clk_name[] = "cpu??_clk";
	struct clk *c;
	int cpu, ret;

	l2_clk = devm_clk_get(dev, "l2_clk");
	if (IS_ERR(l2_clk))
		l2_clk = NULL;

	for_each_possible_cpu(cpu) {
		snprintf(clk_name, sizeof(clk_name), "cpu%d_clk", cpu);
		c = devm_clk_get(dev, clk_name);
		if (!IS_ERR(c))
			cpu_clk[cpu] = c;
		else
			is_sync = true;
	}

	if (!cpu_clk[0])
		return -ENODEV;

	ret = cpufreq_parse_dt(dev);
	if (ret)
		return ret;

	for_each_possible_cpu(cpu) {
		cpufreq_frequency_table_get_attr(freq_table, cpu);
	}

	is_clk = true;

	return 0;
}

static struct of_device_id match_table[] = {
	{ .compatible = "qcom,msm-cpufreq" },
	{}
};

static struct platform_driver msm_cpufreq_plat_driver = {
	.driver = {
		.name = "msm-cpufreq",
		.of_match_table = match_table,
		.owner = THIS_MODULE,
	},
};

static int __init msm_cpufreq_register(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		mutex_init(&(per_cpu(cpufreq_suspend, cpu).suspend_mutex));
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

	platform_driver_probe(&msm_cpufreq_plat_driver, msm_cpufreq_probe);
	msm_cpufreq_wq = alloc_workqueue("msm-cpufreq", WQ_HIGHPRI, 0);
	if (is_clk)
		register_hotcpu_notifier(&msm_cpufreq_cpu_notifier);
	register_pm_notifier(&msm_cpufreq_pm_notifier);
	return cpufreq_register_driver(&msm_cpufreq_driver);
}

device_initcall(msm_cpufreq_register);
