"""RSL-RL 체크포인트의 관측 수를 늘린 사본을 만든다 (관측을 맨 뒤에 붙였을 때 이어서 학습하려고). 2026-11-14 규칙 v2 (53 → 55).

  python widen_obs.py 원본.pt 사본.pt 늘릴_수 [탐색_폭]

actor · critic 첫 층 가중치에 새 열을 0 으로 붙이고 (= 새 관측을 처음엔 안 봄 → 늘리기 전과 똑같이 행동), 관측 정규화는 새 칸을
평균 0 · 분산 1 (그대로 통과) 로 붙인다. 옵티마이저 (Adam) 상태도 같은 모양으로 0 을 붙인다. 탐색 폭을 주면 set_std.py 처럼 바꾼다.
"""

import sys

import torch


def widen(t: torch.Tensor, old: int, add: int, fill: float) -> torch.Tensor:
    """마지막 차원이 old 인 텐서 → old + add (새 칸 = fill)"""
    pad = torch.full(t.shape[:-1] + (add,), fill, dtype=t.dtype)
    return torch.cat((t, pad), dim=-1)


if __name__ == "__main__":
    src, dst, add = sys.argv[1], sys.argv[2], int(sys.argv[3])
    ck = torch.load(src, map_location="cpu", weights_only=False)
    old = ck["actor_state_dict"]["mlp.0.weight"].shape[1]
    for part in ("actor_state_dict", "critic_state_dict"):
        sd = ck[part]
        assert sd["mlp.0.weight"].shape[1] == old and sd["obs_normalizer._mean"].shape[-1] == old, part
        sd["mlp.0.weight"] = widen(sd["mlp.0.weight"], old, add, 0.0)
        sd["obs_normalizer._mean"] = widen(sd["obs_normalizer._mean"], old, add, 0.0)
        sd["obs_normalizer._var"] = widen(sd["obs_normalizer._var"], old, add, 1.0)
        sd["obs_normalizer._std"] = widen(sd["obs_normalizer._std"], old, add, 1.0)
    n_opt = 0
    for st in ck["optimizer_state_dict"]["state"].values():
        for k in ("exp_avg", "exp_avg_sq"):
            if k in st and st[k].dim() == 2 and st[k].shape[1] == old:
                st[k] = widen(st[k], old, add, 0.0)
                n_opt += 1
    if len(sys.argv) > 4:
        p = ck["actor_state_dict"]["distribution.std_param"]
        print(f"탐색 폭 {[round(v, 3) for v in p.tolist()]} → {sys.argv[4]}")
        ck["actor_state_dict"]["distribution.std_param"] = torch.full_like(p, float(sys.argv[4]))
    torch.save(ck, dst)
    print(f"관측 {old} → {old + add} (actor · critic 첫 층 · 정규화, 옵티마이저 상태 {n_opt} 개) → {dst}")
