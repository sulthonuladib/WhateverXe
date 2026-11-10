//
//  kern_xepm.hpp
//  WhateverXe
//
//  Hybrid-CPU power profile, ported from JulioNectar/XPS13Plus-IrisXe-Tahoe "XE-PM"
//  (src/NootedGreen/kern_xepm.cpp, GPL/ISC mix - see REFERENCES.md). Only the parts that matter on
//  this machine are kept; boot-args and sysctls are renamed to `wexpm*` / `debug.wexpm.*` so they
//  cannot be confused with NootedGreen's.
//
//  Why: XCPM writes IA32_HWP_REQUEST in 100 MHz ratio units, but on hybrid P-cores one HWP
//  performance unit is ~78.7 MHz (Linux intel_pstate's hybrid factor 78741). A request of "47"
//  therefore lands at ~3.7 GHz instead of 4.7, which is exactly the ceiling measured here
//  (3646 MHz on one thread). The hook rescales max/desired after Apple's own write, on cores whose
//  HWP_CAPABILITIES highest exceeds the requested max. EPP, energy-efficient turbo and the idle
//  table are handled the same way the original does.
//
//  Everything is opt-in: without `-wexpm`/`wexpm=W` this file installs nothing but read-only
//  sensors. The XCPM route is only installed when the prologue and the wrmsr site match byte for
//  byte, and it logs `route 3` instead of patching when they do not.
//
//  Boot-args: -wexpm (profile), wexpm=W (profile + PL1 in W), wexpmepp=N (0-255, default 0),
//             wexpmcst=0 (keep XCPM's idle table), wexpmpl1=W (PL1 only).
//  Live:      sysctl debug.wexpm.{hwp,epp,eet_off,cst,d3,route,pl1_w,pl1_set,sensors}
//
#ifndef KERN_XEPM_HPP
#define KERN_XEPM_HPP

#include <Headers/kern_patcher.hpp>
#include <Headers/kern_util.hpp>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <kern/thread_call.h>
#include <sys/sysctl.h>

extern "C" void mp_rendezvous_no_intrs(void (*fn)(void *), void *arg);
extern "C" int cpu_number(void);

namespace XEPM {
static inline uint64_t rdmsr(uint32_t m) {
	uint32_t lo, hi;
	asm volatile("rdmsr" : "=a"(lo), "=d"(hi) : "c"(m));
	return (uint64_t(hi) << 32) | lo;
}
static inline void wrmsr(uint32_t m, uint64_t v) {
	asm volatile("wrmsr" : : "c"(m), "a"(uint32_t(v)), "d"(uint32_t(v >> 32)));
}

static bool on = false;
static int hwp = 0, epp = -1, eet = 0, cst = 0, d3n = 0, route = 0, pl1Want = 0, pl1Set = 0;
static mach_vm_address_t orgSetMult = 0, cstPtr = 0;
static bool cstWant = true;

// ─── HWP rescale (the turbo fix) ────────────────────────────────────────────────────────────────────────────────
static uint64_t setMult(uint64_t mult, uint64_t a1, uint64_t a2, uint64_t a3) {
	const uint64_t r = FunctionCast(setMult, orgSetMult)(mult, a1, a2, a3);
	// XCPM calls this before enabling HWP; the HWP MSRs #GP until IA32_PM_ENABLE bit 0 is set.
	if (hwp && (rdmsr(0x770) & 1)) {
		const uint64_t v = rdmsr(0x774);
		if (v & (1ULL << 42)) return r; // package control: leave it
		const uint32_t hi = uint32_t(rdmsr(0x771) & 0xff), mx = uint32_t(v >> 8) & 0xff, des = uint32_t(v >> 16) & 0xff;
		uint64_t nv = v;
		if (mx && hi > mx && hi < 0xff) {
			uint32_t d = des ? (des * hi + mx / 2) / mx : 0;
			if (d > hi) d = hi;
			nv = (nv & ~0xFFFF00ULL) | (uint64_t(hi) << 8) | (uint64_t(d) << 16);
		}
		if (epp >= 0) nv = (nv & ~0xFF000000ULL) | (uint64_t(epp) << 24);
		if (nv != v) wrmsr(0x774, nv);
	}
	return r;
}

// ─── energy-efficient turbo off ────────────────────────────────────────────────────────────────────────────────
static void eetOne(void *) {
	const uint64_t v = rdmsr(0x1FC);
	if (!(v & (1ULL << 19))) wrmsr(0x1FC, v | (1ULL << 19));
}

// ─── XCPM idle table: C8/C10 on, C3 off ────────────────────────────────────────────────────────────────────────
static int cstSet(bool want) {
	uint8_t *tab = cstPtr ? *reinterpret_cast<uint8_t **>(cstPtr) : nullptr;
	if (!tab) return 2;
	auto hint = [&](int i) { return *reinterpret_cast<volatile uint32_t *>(tab + i * 24 + 4); };
	auto en = [&](int i) { return reinterpret_cast<volatile uint32_t *>(tab + i * 24 + 8); };
	if (hint(0) != 0x60 || hint(2) != 0x40 || hint(6) != 0x10) return 3;
	const uint32_t w = want ? 1 : 0;
	if (*en(0) == w && *en(2) == w && *en(6) == !w) return want ? 1 : 0;
	if (MachInfo::setKernelWriting(true, KernelPatcher::kernelWriteLock) != KERN_SUCCESS) return 4;
	*en(0) = w;
	*en(2) = w;
	*en(6) = !w;
	MachInfo::setKernelWriting(false, KernelPatcher::kernelWriteLock);
	return want ? 1 : 0;
}

// ─── D3hot for driverless Intel functions ─────────────────────────────────────────────────────────────────────
static int d3() {
	static const uint16_t ids[] {
		0xa75d, 0x465d, 0x9a19,                     // IPU6 camera
		0xa74f, 0x464f, 0x9a11,                     // GNA
		0x51fc, 0x54fc, 0xa0fc,                     // ISH
		0xa71d, 0x461d, 0x9a03,                     // DPTF participant
		0xa73e, 0xa76d, 0x463e, 0x466d, 0x9a1b, 0x9a1d, // TB4/USB4 DMA
	};
	int n = 0;
	OSIterator *it = IOService::getMatchingServices(IOService::serviceMatching("IOPCIDevice"));
	if (!it) return 0;
	while (auto *o = it->getNextObject()) {
		auto *d = OSDynamicCast(IOPCIDevice, o);
		if (!d) continue;
		if (IOService *cl = d->getClient())
			if (cl->getState() & (kIOServiceMatchedState | kIOServiceRegisteredState)) continue;
		const uint32_t id = d->configRead32(0);
		if ((id & 0xffff) != 0x8086) continue;
		bool want = false;
		for (uint16_t w : ids) if (w == (id >> 16)) want = true;
		uint8_t pm = 0;
		if (!want || !d->findPCICapability(kIOPCIPowerManagementCapability, &pm) || !pm) continue;
		const uint16_t cs = d->configRead16(pm + 4);
		if ((cs & 3) != 3) {
			d->configWrite16(pm + 4, uint16_t(cs | 3));
			n++;
		}
	}
	it->release();
	return n;
}

// ─── optional MCHBAR PL1 ──────────────────────────────────────────────────────────────────────────────────────
static void pl1() {
	if (pl1Want <= 0) return;
	uint64_t mch = 0;
	OSIterator *it = IOService::getMatchingServices(IOService::serviceMatching("IOPCIDevice"));
	if (!it) return;
	while (auto *o = it->getNextObject()) {
		auto *d = OSDynamicCast(IOPCIDevice, o);
		if (d && !d->getBusNumber() && !d->getDeviceNumber() && !d->getFunctionNumber()) {
			const uint64_t v = (uint64_t(d->configRead32(0x4C)) << 32) | d->configRead32(0x48);
			if (v & 1) mch = v & 0x7FFFFE0000ULL;
			break;
		}
	}
	it->release();
	if (!mch) return;
	const uint32_t unit = uint32_t(rdmsr(0x606) & 0xF);
	IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(mch + 0x5000, 0x1000, kIODirectionInOut);
	if (!md) return;
	if (IOMemoryMap *mm = md->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapInhibitCache)) {
		auto *lo = reinterpret_cast<volatile uint32_t *>(mm->getVirtualAddress() + 0x9A0);
		auto *hi = reinterpret_cast<volatile uint32_t *>(mm->getVirtualAddress() + 0x9A4);
		if (!(*hi & 0x80000000U)) {
			const uint32_t want = uint32_t(pl1Want) << unit;
			if (want && want <= 0x7FFF && (*lo & 0x7FFF) != want) *lo = (*lo & ~0x7FFFU) | want | 0x8000U;
			pl1Set = int(*lo & 0x7FFF);
		} else pl1Set = -1;
		mm->release();
	}
	md->release();
}

static void apply(thread_call_param_t, thread_call_param_t) {
	pl1();
	cst = cstSet(cstWant);
	d3n += d3();
	if (!eet) {
		mp_rendezvous_no_intrs(eetOne, nullptr);
		eet = int((rdmsr(0x1FC) >> 19) & 1);
	}
	if (orgSetMult) hwp = 1; // applies at XCPM's next request on each CPU
	IOLog("WhateverXe PM: profile on: HWP %d (route %d), EPP %d, EE turbo off %d, C8/C10 %d, D3 %d, PL1 %d W (0x%x)\n",
	      hwp, route, epp, eet, cst, d3n, pl1Want, pl1Set);
}

// ─── read-only sensors (debug.wexpm.sensors) ──────────────────────────────────────────────────────────────────
struct Sensors { uint32_t ver, ncpu, tjmax, pkg; uint8_t core[32], type[32]; uint32_t eunit, pad; uint64_t epkg, eia, egt; };
static Sensors sens;
static void sensOne(void *) {
	const int c = cpu_number();
	if (c < 0 || c >= 32) return;
	const uint32_t tj = uint32_t(rdmsr(0x1A2) >> 16) & 0xff, dro = uint32_t(rdmsr(0x19C) >> 16) & 0x7f;
	sens.core[c] = uint8_t(tj > dro ? tj - dro : 0);
	uint32_t r[4];
	asm volatile("cpuid" : "=a"(r[0]), "=b"(r[1]), "=c"(r[2]), "=d"(r[3]) : "a"(0x1A), "c"(0));
	sens.type[c] = uint8_t(r[0] >> 24);
	__sync_fetch_and_add(&sens.ncpu, 1);
}
static int sensProc SYSCTL_HANDLER_ARGS {
	bzero(&sens, sizeof(sens));
	sens.ver = 1;
	mp_rendezvous_no_intrs(sensOne, nullptr);
	sens.tjmax = uint32_t(rdmsr(0x1A2) >> 16) & 0xff;
	const uint32_t pd = uint32_t(rdmsr(0x1B1) >> 16) & 0x7f;
	sens.pkg = sens.tjmax > pd ? sens.tjmax - pd : 0;
	sens.eunit = uint32_t(rdmsr(0x606) >> 8) & 0x1f;
	sens.epkg = rdmsr(0x611) & 0xffffffffULL;
	sens.eia = rdmsr(0x639) & 0xffffffffULL;
	sens.egt = rdmsr(0x641) & 0xffffffffULL;
	return SYSCTL_OUT(req, &sens, sizeof(sens));
}
SYSCTL_NODE(_debug, OID_AUTO, wexpm, CTLFLAG_RW, 0, "WhateverXe XE-PM");
SYSCTL_PROC(_debug_wexpm, OID_AUTO, sensors, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_ANYBODY, nullptr, 0, sensProc, "S", "temperatures and RAPL energy");
SYSCTL_INT(_debug_wexpm, OID_AUTO, hwp, CTLFLAG_RW, &hwp, 0, "HWP rescale on hybrid P-cores");
SYSCTL_INT(_debug_wexpm, OID_AUTO, epp, CTLFLAG_RW, &epp, 0, "EPP override, -1 = XCPM's");
SYSCTL_INT(_debug_wexpm, OID_AUTO, eet_off, CTLFLAG_RD, &eet, 0, "");
SYSCTL_INT(_debug_wexpm, OID_AUTO, d3, CTLFLAG_RD, &d3n, 0, "");
SYSCTL_INT(_debug_wexpm, OID_AUTO, route, CTLFLAG_RD, &route, 0, "1 = HWP route installed");
SYSCTL_INT(_debug_wexpm, OID_AUTO, pl1_w, CTLFLAG_RD, &pl1Want, 0, "requested PL1");
SYSCTL_INT(_debug_wexpm, OID_AUTO, pl1_set, CTLFLAG_RD, &pl1Set, 0, "MCHBAR PL1 after write, -1 = locked");
static int cstCtl SYSCTL_HANDLER_ARGS {
	int v = cst;
	int err = SYSCTL_OUT(req, &v, sizeof(v));
	if (err || !req->newptr) return err;
	if ((err = SYSCTL_IN(req, &v, sizeof(v)))) return err;
	cstWant = v != 0;
	cst = cstSet(cstWant);
	return cst >= 2 ? EIO : 0;
}
SYSCTL_PROC(_debug_wexpm, OID_AUTO, cst, CTLTYPE_INT | CTLFLAG_RW, nullptr, 0, cstCtl, "I", "1 = C8/C10 on (write 0/1, root)");

// ─── entry point ─────────────────────────────────────────────────────────────────────────────────────────────
static void install(KernelPatcher &patcher) {
	int pmw = 0;
	const bool pmVal = PE_parse_boot_argn("wexpm", &pmw, sizeof(pmw));
	sysctl_register_oid(&sysctl__debug_wexpm);
	sysctl_register_oid(&sysctl__debug_wexpm_sensors);
	if (!checkKernelArgument("-wexpm") && !pmVal) return;
	on = true;
	int e = 0;
	epp = (PE_parse_boot_argn("wexpmepp", &e, sizeof(e)) && e >= 0 && e <= 255) ? e : 0;
	int cw = 1;
	if (PE_parse_boot_argn("wexpmcst", &cw, sizeof(cw))) cstWant = cw != 0;
	int w = 0;
	if (PE_parse_boot_argn("wexpmpl1", &w, sizeof(w)) && w >= 5 && w <= 64) pl1Want = w;
	else if (pmVal && pmw >= 5 && pmw <= 64) pl1Want = pmw;

	cstPtr = patcher.solveSymbol(KernelPatcher::KernelID, "_xcpm_cstate_table");
	patcher.clearError();
	const mach_vm_address_t base = patcher.solveSymbol(KernelPatcher::KernelID, "_xcpm_enable_pstate_ctrs");
	patcher.clearError();
	if (base) {
		const mach_vm_address_t fn = base + 0x1190;
		static const uint8_t pro[] {0x55, 0x48, 0x89, 0xE5, 0x53, 0x50, 0x89, 0xFB};
		static const uint8_t wr[] {0x89, 0xF8, 0xC1, 0xEA, 0x18, 0xB9, 0x74, 0x07, 0x00, 0x00, 0x0F, 0x30};
		auto *p = reinterpret_cast<const uint8_t *>(fn);
		bool ok = true;
		for (size_t i = 0; i < sizeof(pro); i++) if (p[i] != pro[i]) ok = false;
		for (size_t i = 0; i < sizeof(wr); i++) if (p[0x168 + i] != wr[i]) ok = false;
		if (ok) {
			orgSetMult = patcher.routeFunction(fn, reinterpret_cast<mach_vm_address_t>(setMult), true);
			if (patcher.getError() != KernelPatcher::Error::NoError) {
				orgSetMult = 0;
				patcher.clearError();
			}
		}
		route = orgSetMult ? 1 : (ok ? 5 : 3);
	} else route = 2;
	sysctl_register_oid(&sysctl__debug_wexpm_hwp);
	sysctl_register_oid(&sysctl__debug_wexpm_epp);
	sysctl_register_oid(&sysctl__debug_wexpm_eet_off);
	sysctl_register_oid(&sysctl__debug_wexpm_cst);
	sysctl_register_oid(&sysctl__debug_wexpm_d3);
	sysctl_register_oid(&sysctl__debug_wexpm_route);
	sysctl_register_oid(&sysctl__debug_wexpm_pl1_w);
	sysctl_register_oid(&sysctl__debug_wexpm_pl1_set);

	uint64_t dl;
	thread_call_t t20 = thread_call_allocate(apply, nullptr);
	if (!t20) return;
	clock_interval_to_deadline(20, kSecondScale, &dl);
	thread_call_enter_delayed(t20, dl);
	thread_call_t t60 = thread_call_allocate(apply, nullptr);
	if (t60) {
		clock_interval_to_deadline(60, kSecondScale, &dl);
		thread_call_enter_delayed(t60, dl);
	}
	IOLog("WhateverXe PM: -wexpm, HWP route %d, idle table %s; profile in 20 s\n", route, cstPtr ? "found" : "missing");
}
} // namespace XEPM

#endif
