//
//  kern_adlp_wa.hpp
//  WhateverXe
//
//  ADL-P (Display 13) display workarounds, ported from Linux i915 via
//  JulioNectar/XPS13Plus-IrisXe-Tahoe's "V212" (src/NootedGreen/kern_gen11.cpp). Apple's kext is a
//  Display-12 (TGL) driver; these bits are REQUIRED on Display 13+ and their absence shows up as
//  display-engine misbehaviour: pipe underruns, error-recovery loops and a panel that does not come
//  back after a display-off. Our own symptom on this machine was exactly that:
//  2491 `Display Pipe Underrun` events in one boot, and a display-off that ended in 12208 CoreDisplay
//  "Fallback GPU Display Pipe transaction submission failed" lines with a dead panel.
//
//  Opt-in with -wex-adlp-wa so it can be bisected against the stock behaviour. Only valid on the
//  spoofed ADL-P path (the caller is the AlderLakeN power-init hook).
//
//  Writes (all Display 13 registers, see Linux i915 intel_display_power.c / icl_display_core_init):
//    0x46540 GEN9_CLKGATE_DIS_5      set   bit 17  DPCE_GATING_DIS      (Wa_22011091694)
//    0x46430 GEN8_CHICKEN_DCPR_1     clear bit 7   DDI_CLOCK_REG_ACCESS (Bspec 49189)
//    0x70038 PIPE_CHICKEN (pipe A)   set   bit 30  UNDERRUN_RECOVERY_DISABLE_ADLP
//                                    set   bit 15  PIXEL_ROUNDING_TRUNC_FB_PASSTHRU (WA 1605353570)
//                                    set   bit 7   PER_PIXEL_ALPHA_BYPASS_EN (WA 1153)
//    0x4421C XELPD_DISPLAY_ERR_FATAL_MASK = 0xFFFFFFFF  (mask fatal display errors)
//
#ifndef KERN_ADLP_WA_HPP
#define KERN_ADLP_WA_HPP

#include <Headers/kern_util.hpp>

namespace ADLPWA {
static inline uint32_t read32(IOMemoryMap *map, uint32_t reg) {
	return *reinterpret_cast<volatile uint32_t *>(map->getVirtualAddress() + reg);
}
static inline void write32(IOMemoryMap *map, uint32_t reg, uint32_t v) {
	*reinterpret_cast<volatile uint32_t *>(map->getVirtualAddress() + reg) = v;
}
static inline void rmw(IOMemoryMap *map, uint32_t reg, uint32_t clear, uint32_t set) {
	write32(map, reg, (read32(map, reg) & ~clear) | set);
}

// Returns true when the registers were written; publishes the readback as WEXADLPWA for verification.
static inline bool apply(IOMemoryMap *map) {
	if (!map || map->getLength() != 0x1000000 || !map->getVirtualAddress()) return false;
	rmw(map, 0x46540, 0, 1u << 17);                                   // DPCE_GATING_DIS
	rmw(map, 0x46430, 1u << 7, 0);                                    // DDI_CLOCK_REG_ACCESS off
	rmw(map, 0x70038, 0, (1u << 30) | (1u << 15) | (1u << 7));        // pipe A chicken
	write32(map, 0x4421C, 0xFFFFFFFFu);                               // fatal display errors masked
	uint32_t record[] {1, read32(map, 0x70038), read32(map, 0x46540), read32(map, 0x46430), read32(map, 0x4421C)};
	IORegistryEntry::getRegistryRoot()->setProperty("WEXADLPWA", record, sizeof(record));
	SYSLOG("igfx", "ADL-P WA: PIPE_CHICKEN(A)=0x%x CLKGATE_DIS_5=0x%x CHICKEN_DCPR_1=0x%x ERR_FATAL_MASK=0x%x",
	       record[1], record[2], record[3], record[4]);
	return true;
}
} // namespace ADLPWA

#endif
