# SPDX-License-Identifier: Apache-2.0
"""Reproducible disturbances indexed by message sequence number.

Every disturbance source ("sync.tx_jitter", "dreq.net", ...) owns an independent PRNG
stream derived from ``(seed, source name)``.  The value for index ``i`` is the ``i``-th
element of that stream, whatever the order in which the engine asks for it, so a different
controller (hence a different execution path) sees exactly the same exogenous disturbances.
"""
from __future__ import annotations

import zlib

import numpy as np

from .config import JitterSpec

_CHUNK = 2048


class IndexedStream:
    def __init__(self, seed: int, name: str):
        ss = np.random.SeedSequence(entropy=int(seed) & 0xFFFFFFFFFFFFFFFF,
                                    spawn_key=(zlib.crc32(name.encode()),))
        gn, gu = ss.spawn(2)
        self._gn = np.random.Generator(np.random.PCG64(gn))
        self._gu = np.random.Generator(np.random.PCG64(gu))
        self._z: list[float] = []
        self._u: list[float] = []

    def z(self, i: int) -> float:
        """i-th standard normal."""
        while len(self._z) <= i:
            self._z.extend(self._gn.standard_normal(_CHUNK).tolist())
        return self._z[i]

    def u(self, i: int) -> float:
        """i-th uniform in [0, 1)."""
        while len(self._u) <= i:
            self._u.extend(self._gu.random(_CHUNK).tolist())
        return self._u[i]


class Disturbances:
    def __init__(self, seed: int):
        self.seed = int(seed)
        self._streams: dict[str, IndexedStream] = {}

    def stream(self, name: str) -> IndexedStream:
        s = self._streams.get(name)
        if s is None:
            s = self._streams[name] = IndexedStream(self.seed, name)
        return s

    def jitter_ns(self, name: str, i: int, spec: JitterSpec) -> float:
        """Draw for index ``i`` of source ``name`` (always consumes the same stream slot)."""
        if spec.kind == "none" or spec.scale_ns == 0.0:
            return 0.0
        st = self.stream(name)
        if spec.kind == "uniform":
            return (2.0 * st.u(i) - 1.0) * spec.scale_ns
        if spec.kind == "normal":
            z = st.z(i)
            c = spec.clip_sigma
            return max(-c, min(c, z)) * spec.scale_ns
        if spec.kind == "exponential":
            return -np.log1p(-st.u(i)) * spec.scale_ns
        raise ValueError(f"unknown jitter kind {spec.kind!r}")

    def normal_ns(self, name: str, i: int, sigma_ns: float) -> float:
        if sigma_ns == 0.0:
            return 0.0
        return self.stream(name).z(i) * sigma_ns

    def uniform(self, name: str, i: int) -> float:
        return self.stream(name).u(i)

    def lost(self, name: str, i: int, p: float) -> bool:
        return p > 0.0 and self.stream(name).u(i) < p
