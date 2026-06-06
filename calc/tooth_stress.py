"""
Gear Tooth Stress Calculator with Iterative Solver
===================================================
Calculates Lewis bending stress and Hertzian contact stress for
the QDD planetary gear set. Includes an iterative solver that finds
minimum module/face-width to meet a target safety factor.

Skills demonstrated: iterative solvers, engineering analysis
"""

import math
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from calc.utils.data import GearParams, PlanetarySet, MaterialProps, PLA_PLUS, NYLON_PA6
from calc.utils.constants import (
    OUTPUT_PEAK_TORQUE_NM, GEAR_RATIO, NUM_PLANETS,
    PRESSURE_ANGLE_DEG, MIN_SAFETY_FACTOR_BENDING, MIN_SAFETY_FACTOR_CONTACT,
)
from calc.gear_geometry import design_planetary_set, compute_contact_ratio


# ── Lewis Form Factor ────────────────────────────────────────────────

def lewis_form_factor(num_teeth, pressure_angle_deg=20.0):
    """
    Approximate Lewis form factor Y for standard full-depth teeth.
    Uses the classic approximation: Y = 2/3 * (1 - 2.87/z) for 20° PA.
    More accurate values should come from AGMA tables.
    """
    z = num_teeth
    if pressure_angle_deg == 20.0:
        # Standard approximation (Shigley's)
        y = (2.0 / 3.0) * (1.0 - 2.87 / z)
    else:
        # Fallback for other pressure angles
        y = 0.154 - 0.912 / z
    return max(y, 0.05)  # Floor to avoid negative for very small z


# ── Lewis Bending Stress ─────────────────────────────────────────────

def lewis_bending_stress(tangential_force_n, module_mm, face_width_mm, num_teeth,
                         pressure_angle_deg=20.0):
    """
    Lewis bending stress at the tooth root (MPa).

    σ_b = F_t / (m * b * Y)

    where:
        F_t = tangential force (N)
        m   = module (mm)
        b   = face width (mm)
        Y   = Lewis form factor
    """
    y = lewis_form_factor(num_teeth, pressure_angle_deg)
    sigma_b = tangential_force_n / (module_mm * face_width_mm * y)
    return sigma_b


# ── Hertzian Contact Stress ──────────────────────────────────────────

def hertzian_contact_stress(tangential_force_n, gear1: GearParams, gear2: GearParams,
                            material: MaterialProps):
    """
    Hertzian contact stress at the pitch point (MPa).

    σ_c = sqrt( F_t / (b * d1) * (1/sin(α)cos(α)) * ((1/r1 + 1/r2) * E*) )

    Simplified AGMA-style:
        σ_c = Z_E * sqrt( F_t * K_H / (b * d1 * Z_I) )

    Using fundamental Hertz approach for two cylinders in contact.
    """
    alpha_rad = math.radians(gear1.pressure_angle_deg)
    r1 = gear1.pitch_diameter_mm / 2.0  # mm
    r2 = gear2.pitch_diameter_mm / 2.0  # mm
    b = gear1.face_width_mm             # mm

    # Radii of curvature at pitch point
    rho1 = r1 * math.sin(alpha_rad)  # mm
    rho2 = r2 * math.sin(alpha_rad)  # mm

    # Equivalent radius of curvature
    rho_eq = (rho1 * rho2) / (rho1 + rho2)  # mm

    # Effective elastic modulus (both gears same material)
    E_star = material.elastic_modulus_mpa / (2.0 * (1.0 - material.poisson_ratio**2))

    # Hertzian contact stress (cylinders)
    # σ_c = sqrt( F_t * E* / (π * b * ρ_eq) )
    sigma_c = math.sqrt(tangential_force_n * E_star / (math.pi * b * rho_eq))

    return sigma_c


# ── AGMA K-Factor Stack ──────────────────────────────────────────────
#
# Per AGMA 2101-D04 (metric). Theory + symbol reference:
#   MegaVault/Resources/Engineering/Machine Design/01_Theory/AGMA_Bending_Stress.md
#   MegaVault/Resources/Engineering/Machine Design/01_Theory/AGMA_Contact_Stress.md
# Project defaults (Ko=1.25, Qv=5, Ks=1.0, KH=1.4, KB=1.0):
#   _dev/qdd-gearbox/docs/design/agma-stress-reference.md
# ─────────────────────────────────────────────────────────────────────

def agma_dynamic_factor(pitch_line_velocity_m_s, qv=5):
    """AGMA Kv (metric). Qv = transmission accuracy (3=rough, 12=precision)."""
    b_exp = 0.25 * (12.0 - qv) ** (2.0 / 3.0)
    a = 50.0 + 56.0 * (1.0 - b_exp)
    v = max(pitch_line_velocity_m_s, 0.0)
    return ((a + math.sqrt(200.0 * v)) / a) ** b_exp


def agma_geometry_factor_yj(z_loaded, z_mate, pressure_angle_deg=20.0):
    """
    Bending geometry factor Y_J (=J) for 20° PA, full-depth external spur.
    Coarse interpolation of Norton Fig 12-23 / AGMA 908-B89 representative data.
    Verify against the actual table for production design.
    """
    if pressure_angle_deg != 20.0:
        # No table for other PA — fall back to scaled Lewis form factor
        return 0.85 * lewis_form_factor(z_loaded, pressure_angle_deg)

    # Anchor points: J(z_loaded, z_mate=85) at standard tooth counts
    anchors_at_zm85 = {12: 0.25, 17: 0.31, 25: 0.37, 35: 0.41, 50: 0.45, 85: 0.49}
    zl_keys = sorted(anchors_at_zm85.keys())
    zl_c = min(max(z_loaded, zl_keys[0]), zl_keys[-1])
    for i in range(len(zl_keys) - 1):
        if zl_keys[i] <= zl_c <= zl_keys[i + 1]:
            zlo, zhi = zl_keys[i], zl_keys[i + 1]
            t = (zl_c - zlo) / (zhi - zlo) if zhi > zlo else 0.0
            j_base = anchors_at_zm85[zlo] + t * (anchors_at_zm85[zhi] - anchors_at_zm85[zlo])
            break

    # Mate-tooth-count adjustment: J drops slightly as mate teeth decrease
    # Approx -0.02 swing from z_mate=1000 (=0.51 at z_loaded=85) down to z_mate=17
    zm_factor = 1.0 - 0.02 * max(0.0, (85 - z_mate)) / 68.0
    return j_base * zm_factor


def agma_geometry_factor_zi(gear_ratio, pressure_angle_deg=20.0,
                            internal_mesh=False, load_sharing_mn=1.0):
    """
    Pitting geometry factor Z_I (=I) for spur mesh.
    External: Z_I = cosφ·sinφ / (2·m_N) * m_G/(m_G+1)
    Internal: Z_I = cosφ·sinφ / (2·m_N) * m_G/(m_G-1)
    """
    phi = math.radians(pressure_angle_deg)
    mg = max(gear_ratio, 1.0)
    base = math.cos(phi) * math.sin(phi) / (2.0 * load_sharing_mn)
    if internal_mesh:
        return base * mg / (mg - 1.0)
    return base * mg / (mg + 1.0)


def agma_elastic_coefficient_ze(mat_pinion: MaterialProps, mat_gear: MaterialProps):
    """Z_E in sqrt(MPa). Steel/steel ≈ 191."""
    e_p = mat_pinion.elastic_modulus_mpa
    e_g = mat_gear.elastic_modulus_mpa
    term = (1.0 - mat_pinion.poisson_ratio ** 2) / e_p \
         + (1.0 - mat_gear.poisson_ratio ** 2) / e_g
    return math.sqrt(1.0 / (math.pi * term))


def agma_bending_stress(tangential_force_n, gear_loaded: GearParams, z_mate,
                        ko=1.25, kv=None, ks=1.0, kh=1.4, kb=1.0,
                        pitch_line_velocity_m_s=None, qv=5):
    """
    AGMA 2101-D04 bending stress (MPa).
        σ_F = (F_t / (b·m)) · K_o · K_v · K_s · (K_H · K_B / Y_J)
    """
    if kv is None:
        v = pitch_line_velocity_m_s if pitch_line_velocity_m_s is not None else 0.0
        kv = agma_dynamic_factor(v, qv)
    yj = agma_geometry_factor_yj(gear_loaded.num_teeth, z_mate,
                                 gear_loaded.pressure_angle_deg)
    base = tangential_force_n / (gear_loaded.face_width_mm * gear_loaded.module_mm)
    return base * ko * kv * ks * (kh * kb / yj), {"Kv": kv, "YJ": yj}


def agma_contact_stress(tangential_force_n, pinion: GearParams, gear: GearParams,
                        mat_pinion: MaterialProps, mat_gear: MaterialProps,
                        ko=1.25, kv=None, ks=1.0, kh=1.4, zr=1.0,
                        pitch_line_velocity_m_s=None, qv=5,
                        internal_mesh=False, load_sharing_mn=1.0):
    """
    AGMA 2101-D04 contact stress (MPa).
        σ_H = Z_E · sqrt( F_t · K_o · K_v · K_s · K_H · Z_R / (d_w1 · b · Z_I) )
    pinion = smaller gear (drives d_w1 = pinion pitch diameter).
    """
    if kv is None:
        v = pitch_line_velocity_m_s if pitch_line_velocity_m_s is not None else 0.0
        kv = agma_dynamic_factor(v, qv)
    ze = agma_elastic_coefficient_ze(mat_pinion, mat_gear)
    gear_ratio = gear.num_teeth / pinion.num_teeth
    zi = agma_geometry_factor_zi(gear_ratio, pinion.pressure_angle_deg,
                                 internal_mesh, load_sharing_mn)
    d_w1 = pinion.pitch_diameter_mm
    b = min(pinion.face_width_mm, gear.face_width_mm)
    radicand = (tangential_force_n * ko * kv * ks * kh * zr) / (d_w1 * b * zi)
    sigma_h = ze * math.sqrt(max(radicand, 0.0))
    return sigma_h, {"ZE": ze, "ZI": zi, "Kv": kv}


def analyze_agma_stresses(gear_set: PlanetarySet, material: MaterialProps,
                          output_torque_nm=OUTPUT_PEAK_TORQUE_NM,
                          input_speed_rpm=0.0,
                          ko=1.25, ks=1.0, kh=1.4, kb_ring=1.0, qv=5,
                          unequal_share_factor=1.10,
                          sigma_fp_mpa=None, sigma_hp_mpa=None):
    """
    Full AGMA stress workup for the planetary set.

    Sun is the pinion in the sun-planet mesh (smaller, drives contact stress).
    Planet-ring contact is the internal mesh (lower stress, not returned).

    sigma_fp_mpa / sigma_hp_mpa: allowable bending and contact stress for the
    material at 1e7 cycles. If None, falls back to material yield strength
    (rough proxy; replace with measured/AGMA-table values for real designs).
    """
    # Tangential force per planet, with unequal-load-share derating
    ft_nominal = tangential_force_on_sun(output_torque_nm, gear_set)
    ft = ft_nominal * unequal_share_factor

    # Pitch-line velocity at sun (m/s) — used by Kv
    sun_pitch_radius_m = gear_set.sun.pitch_diameter_mm / 2000.0
    pitch_line_v = sun_pitch_radius_m * (input_speed_rpm * 2.0 * math.pi / 60.0)

    kv = agma_dynamic_factor(pitch_line_v, qv)

    # Bending — sun
    sigma_b_sun, b_sun_meta = agma_bending_stress(
        ft, gear_set.sun, gear_set.planet.num_teeth,
        ko=ko, kv=kv, ks=ks, kh=kh, kb=1.0, qv=qv,
    )
    # Bending — planet (mates with sun for outer flank, ring for inner; use sun-side)
    sigma_b_planet, b_pl_meta = agma_bending_stress(
        ft, gear_set.planet, gear_set.sun.num_teeth,
        ko=ko, kv=kv, ks=ks, kh=kh, kb=1.0, qv=qv,
    )
    # Bending — ring (internal teeth; KB applies)
    ring_gear = GearParams(
        module_mm=gear_set.sun.module_mm,
        num_teeth=gear_set.ring_teeth,
        pressure_angle_deg=gear_set.sun.pressure_angle_deg,
        face_width_mm=gear_set.planet.face_width_mm,
    )
    sigma_b_ring, b_ring_meta = agma_bending_stress(
        ft, ring_gear, gear_set.planet.num_teeth,
        ko=ko, kv=kv, ks=ks, kh=kh, kb=kb_ring, qv=qv,
    )

    # Contact — sun-planet (external mesh, sun is pinion)
    sigma_c_sp, c_meta = agma_contact_stress(
        ft, gear_set.sun, gear_set.planet, material, material,
        ko=ko, kv=kv, ks=ks, kh=kh, zr=1.0, qv=qv,
        internal_mesh=False, load_sharing_mn=1.0,
    )

    # Allowables — fall back to yield if not provided
    sFP = sigma_fp_mpa if sigma_fp_mpa is not None else material.yield_strength_mpa
    sHP = sigma_hp_mpa if sigma_hp_mpa is not None else material.yield_strength_mpa

    return {
        "tangential_force_n": ft,
        "pitch_line_velocity_m_s": pitch_line_v,
        "Kv": kv,
        "bending_stress_sun_mpa": sigma_b_sun,
        "bending_stress_planet_mpa": sigma_b_planet,
        "bending_stress_ring_mpa": sigma_b_ring,
        "contact_stress_sp_mpa": sigma_c_sp,
        "sf_bending_sun": sFP / sigma_b_sun,
        "sf_bending_planet": sFP / sigma_b_planet,
        "sf_bending_ring": sFP / sigma_b_ring,
        "sf_contact_sp": sHP / sigma_c_sp,
        "geometry_yj_sun": b_sun_meta["YJ"],
        "geometry_yj_planet": b_pl_meta["YJ"],
        "geometry_zi_sp": c_meta["ZI"],
        "elastic_ze": c_meta["ZE"],
        "sigma_FP_used_mpa": sFP,
        "sigma_HP_used_mpa": sHP,
    }


def print_agma_results(results, material_name=""):
    """Print AGMA stress analysis results."""
    print(f"=== AGMA Stress Analysis{' (' + material_name + ')' if material_name else ''} ===\n")
    print(f"  Tangential Force per planet:  {results['tangential_force_n']:.1f} N")
    print(f"  Pitch-line velocity (sun):    {results['pitch_line_velocity_m_s']:.2f} m/s")
    print(f"  K_v (dynamic factor):         {results['Kv']:.2f}")
    print(f"  Z_E (elastic coefficient):    {results['elastic_ze']:.1f} sqrt(MPa)")
    print(f"  Y_J  sun / planet:            {results['geometry_yj_sun']:.3f} / {results['geometry_yj_planet']:.3f}")
    print(f"  Z_I  sun-planet:              {results['geometry_zi_sp']:.4f}")
    print(f"  Allowables (sigma_FP / sigma_HP): {results['sigma_FP_used_mpa']:.0f} / {results['sigma_HP_used_mpa']:.0f} MPa")
    print()
    print(f"  sigma_F (sun bending):    {results['bending_stress_sun_mpa']:>7.1f} MPa    "
          f"S_F = {results['sf_bending_sun']:.2f}   "
          f"{'PASS' if results['sf_bending_sun'] >= MIN_SAFETY_FACTOR_BENDING else 'FAIL'}")
    print(f"  sigma_F (planet bending): {results['bending_stress_planet_mpa']:>7.1f} MPa    "
          f"S_F = {results['sf_bending_planet']:.2f}   "
          f"{'PASS' if results['sf_bending_planet'] >= MIN_SAFETY_FACTOR_BENDING else 'FAIL'}")
    print(f"  sigma_F (ring bending):   {results['bending_stress_ring_mpa']:>7.1f} MPa    "
          f"S_F = {results['sf_bending_ring']:.2f}   "
          f"{'PASS' if results['sf_bending_ring'] >= MIN_SAFETY_FACTOR_BENDING else 'FAIL'}")
    print(f"  sigma_H (sun-planet):     {results['contact_stress_sp_mpa']:>7.1f} MPa    "
          f"S_H = {results['sf_contact_sp']:.2f}   "
          f"{'PASS' if results['sf_contact_sp'] >= MIN_SAFETY_FACTOR_CONTACT else 'FAIL'}")
    print()


# ── Force Analysis ────────────────────────────────────────────────────

def tangential_force_on_sun(output_torque_nm, gear_set: PlanetarySet):
    """
    Tangential force on the sun gear from the mesh with planets.

    For planetary with fixed ring, carrier output:
        T_sun = T_output / ratio
        F_t = T_sun / (r_sun / 1000)  (convert mm to m)
        Force per planet = F_t / num_planets
    """
    sun_torque_nm = output_torque_nm / gear_set.ratio
    sun_radius_m = gear_set.sun.pitch_diameter_mm / 2000.0
    total_tangential = sun_torque_nm / sun_radius_m  # N
    force_per_planet = total_tangential / gear_set.num_planets
    return force_per_planet


# ── Stress Analysis ──────────────────────────────────────────────────

def analyze_stresses(gear_set: PlanetarySet, material: MaterialProps,
                     output_torque_nm=OUTPUT_PEAK_TORQUE_NM):
    """Run full stress analysis on the planetary gear set."""
    ft = tangential_force_on_sun(output_torque_nm, gear_set)

    # Sun gear bending
    sigma_b_sun = lewis_bending_stress(
        ft, gear_set.sun.module_mm, gear_set.sun.face_width_mm,
        gear_set.sun.num_teeth, gear_set.sun.pressure_angle_deg
    )

    # Planet gear bending
    sigma_b_planet = lewis_bending_stress(
        ft, gear_set.planet.module_mm, gear_set.planet.face_width_mm,
        gear_set.planet.num_teeth, gear_set.planet.pressure_angle_deg
    )

    # Sun-planet contact stress
    sigma_c_sp = hertzian_contact_stress(ft, gear_set.sun, gear_set.planet, material)

    # Safety factors
    sf_bend_sun = material.yield_strength_mpa / sigma_b_sun
    sf_bend_planet = material.yield_strength_mpa / sigma_b_planet
    sf_contact = material.yield_strength_mpa / sigma_c_sp

    results = {
        "tangential_force_n": ft,
        "bending_stress_sun_mpa": sigma_b_sun,
        "bending_stress_planet_mpa": sigma_b_planet,
        "contact_stress_mpa": sigma_c_sp,
        "sf_bending_sun": sf_bend_sun,
        "sf_bending_planet": sf_bend_planet,
        "sf_contact": sf_contact,
    }
    return results


def print_stress_results(results, material_name=""):
    """Print stress analysis results."""
    print(f"=== Tooth Stress Analysis{' (' + material_name + ')' if material_name else ''} ===\n")
    print(f"  Tangential Force per Planet:  {results['tangential_force_n']:.1f} N")
    print()
    print(f"  Bending Stress (Sun):         {results['bending_stress_sun_mpa']:.1f} MPa")
    print(f"  Bending Stress (Planet):      {results['bending_stress_planet_mpa']:.1f} MPa")
    print(f"  Contact Stress (Sun-Planet):  {results['contact_stress_mpa']:.1f} MPa")
    print()
    print(f"  Safety Factor — Bending (Sun):    {results['sf_bending_sun']:.2f}"
          f"  {'PASS' if results['sf_bending_sun'] >= MIN_SAFETY_FACTOR_BENDING else 'FAIL'}")
    print(f"  Safety Factor — Bending (Planet): {results['sf_bending_planet']:.2f}"
          f"  {'PASS' if results['sf_bending_planet'] >= MIN_SAFETY_FACTOR_BENDING else 'FAIL'}")
    print(f"  Safety Factor — Contact:          {results['sf_contact']:.2f}"
          f"  {'PASS' if results['sf_contact'] >= MIN_SAFETY_FACTOR_CONTACT else 'FAIL'}")
    print()


# ── Iterative Solver ──────────────────────────────────────────────────

def find_minimum_geometry(material: MaterialProps, output_torque_nm=OUTPUT_PEAK_TORQUE_NM,
                          target_sf_bending=MIN_SAFETY_FACTOR_BENDING,
                          target_sf_contact=MIN_SAFETY_FACTOR_CONTACT,
                          module_range=(0.5, 3.0), module_step=0.25,
                          fw_range=(5.0, 25.0), fw_step=1.0,
                          sun_teeth_range=(10, 24)):
    """
    Iterative solver: sweep module, face width, and sun teeth to find the
    minimum geometry that meets target safety factors.

    Returns list of valid configurations sorted by compactness (ring OD).
    """
    valid = []

    mod = module_range[0]
    while mod <= module_range[1]:
        for zs in range(sun_teeth_range[0], sun_teeth_range[1] + 1):
            zr = int(zs * (GEAR_RATIO - 1))
            zp = (zr - zs) // 2
            if (zr - zs) % 2 != 0 or zp < 5:
                continue
            if (zs + zr) % NUM_PLANETS != 0:
                continue

            # Check contact ratio
            cr = compute_contact_ratio(mod, zs, zp)
            if cr < 1.2:
                continue

            fw = fw_range[0]
            while fw <= fw_range[1]:
                gs = design_planetary_set(mod, zs, PRESSURE_ANGLE_DEG, NUM_PLANETS, fw)
                res = analyze_stresses(gs, material, output_torque_nm)

                if (res["sf_bending_sun"] >= target_sf_bending and
                        res["sf_bending_planet"] >= target_sf_bending and
                        res["sf_contact"] >= target_sf_contact):
                    ring_od = mod * zr + 2 * mod  # approximate ring OD
                    valid.append({
                        "module_mm": mod,
                        "sun_teeth": zs,
                        "planet_teeth": zp,
                        "ring_teeth": zr,
                        "face_width_mm": fw,
                        "ring_od_mm": ring_od,
                        "sf_bending_sun": res["sf_bending_sun"],
                        "sf_bending_planet": res["sf_bending_planet"],
                        "sf_contact": res["sf_contact"],
                        "contact_ratio": cr,
                    })
                    break  # Found min face width for this module/teeth combo
                fw += fw_step
        mod += module_step

    # Sort by ring OD (most compact first)
    valid.sort(key=lambda x: x["ring_od_mm"])
    return valid


# ── Main ──────────────────────────────────────────────────────────────

def main():
    # Baseline analysis with PLA+ and starting geometry
    print("=" * 60)
    print("  QDD Gearbox — Tooth Stress Calculator")
    print("=" * 60)
    print()

    module_mm = 1.5
    sun_teeth = 12
    face_width = 10.0

    gear_set = design_planetary_set(module_mm, sun_teeth, PRESSURE_ANGLE_DEG,
                                    NUM_PLANETS, face_width)

    print(f"Baseline: m={module_mm} mm, Z_sun={sun_teeth}, b={face_width} mm\n")

    for mat in [PLA_PLUS, NYLON_PA6]:
        results = analyze_stresses(gear_set, mat)
        print_stress_results(results, mat.name)

    # AGMA pass on the baseline (project defaults: Ko=1.25, Qv=5, KH=1.4)
    print("=" * 60)
    print("  AGMA K-factor stack — baseline geometry")
    print("=" * 60)
    print()
    for mat in [PLA_PLUS, NYLON_PA6]:
        agma_results = analyze_agma_stresses(
            gear_set, mat,
            input_speed_rpm=3000.0,  # representative motor speed
        )
        print_agma_results(agma_results, mat.name)

    # Run iterative solver
    print("=" * 60)
    print("  Iterative Solver — Finding Minimum Geometry")
    print("=" * 60)
    print()

    for mat in [PLA_PLUS, NYLON_PA6]:
        print(f"--- Material: {mat.name} (Sy = {mat.yield_strength_mpa} MPa) ---\n")
        configs = find_minimum_geometry(mat)

        if not configs:
            print("  No valid configuration found in search range.\n")
            continue

        print(f"  Found {len(configs)} valid configurations. Top 5 most compact:\n")
        print(f"  {'m':>5} {'Zs':>4} {'Zp':>4} {'Zr':>4} {'b':>6} {'Ring OD':>8} "
              f"{'SF_b(s)':>8} {'SF_b(p)':>8} {'SF_c':>6} {'CR':>5}")
        print("  " + "-" * 65)

        for cfg in configs[:5]:
            print(f"  {cfg['module_mm']:>5.2f} {cfg['sun_teeth']:>4} "
                  f"{cfg['planet_teeth']:>4} {cfg['ring_teeth']:>4} "
                  f"{cfg['face_width_mm']:>6.1f} {cfg['ring_od_mm']:>8.1f} "
                  f"{cfg['sf_bending_sun']:>8.2f} {cfg['sf_bending_planet']:>8.2f} "
                  f"{cfg['sf_contact']:>6.2f} {cfg['contact_ratio']:>5.2f}")
        print()

        best = configs[0]
        print(f"  >> Recommended: m={best['module_mm']} mm, Z_sun={best['sun_teeth']}, "
              f"b={best['face_width_mm']} mm, Ring OD={best['ring_od_mm']:.1f} mm\n")


if __name__ == "__main__":
    main()
