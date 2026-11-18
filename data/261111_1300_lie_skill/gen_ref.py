"""학습 환경 lieupright_env.py 의 식 (target_R_base · _stand_spot · _pre_physics_step β/ρ) 을 torch 로 그대로 옮겨 시험 기준값을 냄 (도구)."""
import math, torch
torch.set_default_dtype(torch.float64)
J1X = 0.096
def target_R(gb, beta, rho):
    h = torch.stack((gb[:, 0] - J1X, gb[:, 1], torch.zeros_like(gb[:, 0])), dim=1)
    h = h / h.norm(dim=1, keepdim=True).clamp(min=1e-6)
    zb = torch.tensor([[0.0, 0.0, 1.0]]).expand(len(gb), 3)
    t = torch.cross(zb, h, dim=1)
    cb, sb = torch.cos(beta)[:, None], torch.sin(beta)[:, None]
    a = -cb * zb + sb * h
    u0 = torch.cross(a, t, dim=1)
    cr, sr = torch.cos(rho)[:, None], torch.sin(rho)[:, None]
    c = cr * t + sr * u0
    u = -sr * t + cr * u0
    return torch.stack((a, c, u), dim=2)
BOX_SIZE=0.205; spot_edge=0.030
def stand_spot(bc, byaw, nbp):
    gu = torch.arange(-(BOX_SIZE / 2 - spot_edge), BOX_SIZE / 2 - spot_edge + 1e-6, 0.0145)
    UU, VV = torch.meshgrid(gu, gu, indexing="xy"); UU, VV = UU.reshape(-1), VV.reshape(-1)
    cy, sy = torch.cos(byaw)[:, None], torch.sin(byaw)[:, None]
    cx = bc[:, 0:1] + cy * UU - sy * VV; cyy = bc[:, 1:2] + sy * UU + cy * VV
    rr = torch.hypot(cx - J1X, cyy)
    dn = torch.hypot(cx - nbp[:, 0:1], cyy - nbp[:, 1:2])
    okd = (rr >= 0.27) & (rr <= 0.32) & (dn >= 0.07)
    sc = (rr - 0.285).abs() - 0.2 * dn.clamp(max=0.12); sc = torch.where(okd, sc, sc + 10.0)
    k = sc.argmin(dim=1); ar = torch.arange(len(bc))
    return torch.stack((cx[ar, k], cyy[ar, k]), dim=1), len(gu)

if __name__ == "__main__":
    gb = torch.tensor([[0.376, 0.0, 0.12], [0.33, -0.09, 0.15], [0.30, 0.11, 0.10]])
    be = torch.tensor([math.pi / 4, 1.2, -0.1]); rh = torch.tensor([0.0, 0.7, -2.0])
    R = target_R(gb, be, rh)
    for i in range(3):
        print(f"// target {i}: g {gb[i].tolist()} beta {be[i].item()} rho {rh[i].item()}")
        print("{" + ", ".join(f"{v:.9f}" for v in R[i].reshape(-1).tolist()) + "},")
    C, PRE = -0.04035, 0.3625
    def rot(p, th):
        q = torch.tensor(p) - torch.tensor([C, 0.0]); c, s = math.cos(th), math.sin(th)
        return [c * q[0].item() - s * q[1].item() + C, s * q[0].item() + c * q[1].item()]
    TURN = math.asin(0.04 / (J1X - C))
    cases = []
    for s_ in (1, -1):
        th = -s_ * TURN
        bc = rot([PRE, 0.0], th); nb = rot([PRE, -s_ * 0.04], th)
        cases.append((bc, th, nb))
    cases.append(([PRE, 0.0], 0.0, [PRE + 0.04, 0.0]))
    cases.append(([0.35, 0.02], 0.05, [0.33, 0.05]))
    bcs = torch.tensor([c[0] for c in cases]); ys = torch.tensor([c[1] for c in cases]); nbs = torch.tensor([c[2] for c in cases])
    D, n = stand_spot(bcs, ys, nbs)
    print("// 격자 한 변", n)
    for i in range(len(cases)):
        print(f"{{{{{bcs[i,0]:.9f}, {bcs[i,1]:.9f}}}, {ys[i]:.9f}, {{{nbs[i,0]:.9f}, {nbs[i,1]:.9f}}}, {{{D[i,0]:.9f}, {D[i,1]:.9f}}}}},")
