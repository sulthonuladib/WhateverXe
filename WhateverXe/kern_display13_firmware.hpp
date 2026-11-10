#ifndef KERN_DISPLAY13_FIRMWARE_HPP
#define KERN_DISPLAY13_FIRMWARE_HPP
#include "kern_vbt.hpp"
#include "kern_xelp_panel_power.hpp"
#include "kern_display13_output_wiring.hpp"
namespace XELP { namespace VBT {
    // ADLP/ADLN VBT DDC1/2 => Combo pins1/2, DDC3..6 => TC pins9..12.
    // Capability bits are not live modes. HDMI-only records may advertise
    // USB-C/TBT capability; the TC runtime must separately prove legacy mode.
    inline Display13OutputWiring display13HDMI(const Ports &ports,uint32_t backend) {
        Display13OutputWiring w;w.valid=true;
        if(!ports.valid())return w;
        uint32_t used=0;
        for(unsigned d=0;d<7;++d) {
            if(!(ports.present&backend&Display13OutputWiring::Physical&(1U<<d)) || !ports.port[d].tmdsOnly())continue;
            const auto selector=ports.port[d].ddc;
            const auto pin=selector>=3 && selector<=6 ? selector+6 : selector<=2 ? selector : 0;
            if(!pin || (d<3 ? pin>2 : pin<9) || (used&(1U<<pin)))return {};
            used|=1U<<pin;w.hdmi|=1U<<d;w.ddc[d]=pin;
        }
        return w;
    }
    // Display13 VBT AUX F..I maps directly to TC1..4. DP-only USB-C records
    // describe a possible DP-alt route, not a live connection or ownership.
    // panelFallback: opt-in (-wex-panel-fallback) route adoption for firmware whose
    // BDB names the internal eDP DDI but whose LFP option/eDP/backlight/power blocks
    // are unusable. Off by default: the shipped contract only wires a panel it can
    // also power, and tests/display13_panel.cpp pins that.
    inline Display13OutputWiring display13Fixed(const Ports &ports,uint32_t backend,Family family=Family::AlderLakeN,
                                                bool panelFallback=false) {
        auto w=display13HDMI(ports,backend);
        if(!w.valid || !ports.valid())return w;
        if(PanelPower::nativePPS0(ports,family) && (backend&(1U<<ports.panel.ddi)))
            w.panel=1U<<ports.panel.ddi;
        if(!w.panel && panelFallback) {
            // Only the wiring bit: panel power, backlight and delay facts are never
            // invented here. Firmware may carry a second internal DP child next to the
            // real LFP record (HP ADL-P: DDI A aux 0x40 and a duplicate DDI B aux 0x10),
            // so take the lowest internal DP DDI instead of treating the pair as
            // ambiguous and dropping the route entirely.
            unsigned internal=PortCount;
            for(unsigned d=0;d<2;++d)
                if((ports.present&backend&(1U<<d)) && ports.port[d].internal() && ports.port[d].dp()) {
                    internal=d;break;
                }
            if(internal<2)w.panel=1U<<internal;
        }
        for(unsigned d=3;d<7;++d) {
            const auto &p=ports.port[d];const auto aux=(d+3)*0x10U;
            const bool alt=p.dp() && !(p.type&0x10) && (p.capabilities&1);
            if(!(ports.present&backend&(1U<<d)) || (!alt && (p.type&0x14)!=0x14) ||
               (p.type&(0x1000|0x400|0x20|1)) || (p.flags&0x17) || p.aux!=aux)continue;
            bool shared=false;
            for(unsigned other=0;other<PortCount;++other)if(other!=d && (ports.present&(1U<<other)) &&
                ports.port[other].dp() && ports.port[other].aux==aux)shared=true;
            if(!shared){w.dp|=1U<<d;if(alt)w.typeC|=1U<<d;}
        }
        return w;
    }

} }
#endif
