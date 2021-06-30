/*
 * Copyright (C) 2012-2015 - ARM Ltd
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <clocksource/arm_arch_timer.h>
#include <linux/compiler.h>
#include <linux/kvm_host.h>
#include <linux/delay.h>

#include <asm/kvm_hyp.h>

#ifdef CONFIG_MARVELL_ERRATUM_38627
/* Workaround is to ensure maximum 2us of time gap between timer expiry
 * and timer programming which can de-assert timer interrupt.
 * Time calculation below is based on 100MHz as timer frequency is fixed
 * to 100MHz on all affected parts.
 */
static void apply_mrvl_erratum_38627(void)
{
	int32_t tval;

	tval = read_sysreg(cntv_tval_el0);

	/* Timer already expired, wait for (2 - expired time)us */
	if ((tval > -200) && (tval < 0))
		udelay(2 + tval/100);

	/* Timer is about to expire, wait for 2us + time to expire */
	if (tval >= 0 && tval < 200)
		udelay(3 + tval/100);
}
#else
static void apply_mrvl_erratum_38627(void)
{
}
#endif

/* vcpu is already in the HYP VA space */
void __hyp_text __timer_save_state(struct kvm_vcpu *vcpu)
{
	struct arch_timer_cpu *timer = &vcpu->arch.timer_cpu;
	struct arch_timer_context *vtimer = vcpu_vtimer(vcpu);
	u64 val;

	if (timer->enabled) {
		apply_mrvl_erratum_38627();
		vtimer->cnt_ctl = read_sysreg_el0(cntv_ctl);
		vtimer->cnt_cval = read_sysreg_el0(cntv_cval);
	}

	/* Disable the virtual timer */
	write_sysreg_el0(0, cntv_ctl);

	/*
	 * We don't need to do this for VHE since the host kernel runs in EL2
	 * with HCR_EL2.TGE ==1, which makes those bits have no impact.
	 */
	if (!has_vhe()) {
		/* Allow physical timer/counter access for the host */
		val = read_sysreg(cnthctl_el2);
		val |= CNTHCTL_EL1PCTEN | CNTHCTL_EL1PCEN;
		write_sysreg(val, cnthctl_el2);
	}

	/* Clear cntvoff for the host */
	write_sysreg(0, cntvoff_el2);
}

void __hyp_text __timer_restore_state(struct kvm_vcpu *vcpu)
{
	struct arch_timer_cpu *timer = &vcpu->arch.timer_cpu;
	struct arch_timer_context *vtimer = vcpu_vtimer(vcpu);
	u64 val;

	/* Those bits are already configured at boot on VHE-system */
	if (!has_vhe()) {
		/*
		 * Disallow physical timer access for the guest
		 * Physical counter access is allowed
		 */
		val = read_sysreg(cnthctl_el2);
		val &= ~CNTHCTL_EL1PCEN;
		val |= CNTHCTL_EL1PCTEN;
		write_sysreg(val, cnthctl_el2);
	}

	if (timer->enabled) {
		write_sysreg(vtimer->cntvoff, cntvoff_el2);
		write_sysreg_el0(vtimer->cnt_cval, cntv_cval);
		isb();
		write_sysreg_el0(vtimer->cnt_ctl, cntv_ctl);
	}
}
