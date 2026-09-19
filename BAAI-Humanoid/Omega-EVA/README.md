# ω-EVA

**ω-EVA: Envision, Verify, and Act with Latent Interactive World Models**

[![Project Page](https://img.shields.io/badge/Project-Page-blue)](https://baai-humanoid.github.io/Omega-EVA/)
[![Paper](https://img.shields.io/badge/arXiv-2606.09457-b31b1b)](https://arxiv.org/abs/2606.09457)
[![License](https://img.shields.io/badge/License-Apache--2.0-green)](LICENSE)

> Code, datasets, checkpoints, and reproduction instructions are being prepared for release. We currently plan to open-source the project in **late August 2026**.

## About

ω-EVA introduces an **Envision-Verify-Act** loop for embodied action generation. Instead of mapping observations directly to actions, the policy first proposes an action chunk, an action-conditioned latent world model envisions the proposal-conditioned future, and a tri-branch refiner jointly reasons over the current state, imagined future, and proposed action before producing the final action.

The key idea is to make the world model an active action-feedback module inside the policy loop. Consequence reasoning stays in latent feature space, so ω-EVA can inspect candidate-action futures without generating future videos during inference.

## Release Status / TODO

- [x] Paper available
- [x] Project page available
- [ ] Code release
- [ ] Datasets
- [ ] Checkpoints
- [ ] Reproduction and evaluation instructions

## License

This project will be released under the [Apache License 2.0](LICENSE).

## Citation

If you find ω-EVA useful for your research, please consider citing our paper:

```bibtex
@article{sun2026omegaeva,
  title   = {$\omega$-EVA: Envision, Verify, and Act
             with Latent Interactive World Models},
  author  = {Sun, Zhenguo and Sun, Yu and
             Huang, Hande and Knoll, Alois},
  journal = {arXiv preprint arXiv:2606.09457},
  year    = {2026}
}
```
