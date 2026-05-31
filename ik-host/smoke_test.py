"""Offline smoke test: load the config, build the IK chain, solve at
home, solve at a perturbed target, verify the active joints are in
the URDF chain. Touches no hardware."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ik_host import IkSolver, load_config  # noqa: E402


def main() -> int:
    here = Path(__file__).resolve().parent
    cfg = load_config(here.parent / "config-ik.toml")
    urdf = (here.parent / cfg.urdf).resolve()
    print(f"loading {urdf}")
    ik = IkSolver(urdf, cfg.chain_elements, cfg.chain_base_type,
                  cfg.active_joints)
    print(f"chain links ({len(ik.chain.links)}):")
    for i, link in enumerate(ik.chain.links):
        flag = "*" if link.name in ik.joint_index else " "
        print(f"  {flag}[{i:2d}] {link.name}")
    home = cfg.cartesian.home_xyz
    print(f"\nsolving home {home}")
    sol = ik.solve(home)
    for j in ik.joint_index:
        print(f"  {j:24s} = {ik.joint_angle(sol, j):+.4f} rad")

    targets = [
        (home[0] + 0.02, home[1], home[2]),
        (home[0], home[1] + 0.02, home[2]),
        (home[0], home[1], home[2] + 0.02),
    ]
    for t in targets:
        sol = ik.solve(t)
        delta = [ik.joint_angle(sol, j) for j in ik.joint_index]
        print(f"\nsolving {t}")
        for j, a in zip(ik.joint_index, delta):
            print(f"  {j:24s} = {a:+.4f} rad")

    # Cross-check config -> URDF mapping
    cfg_joints = {jm.joint for b in cfg.boards for jm in b.joints}
    missing = cfg_joints - set(ik.joint_index.keys())
    if missing:
        print(f"\nFAIL: config joints not in IK chain: {sorted(missing)}")
        return 1
    print(f"\nOK: all {len(cfg_joints)} config-mapped joints found in chain")
    return 0


if __name__ == "__main__":
    sys.exit(main())
