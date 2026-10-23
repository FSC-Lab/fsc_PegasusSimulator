#!/usr/bin/env python
"""
| File: inertia_utils.py
| License: BSD-3-Clause
| Description: Author a FULL body-frame inertia tensor (products of inertia included) onto a
|   USD rigid body.
|
|   WHY THIS EXISTS. USD/PhysX has no attribute for products of inertia. A rigid body stores
|   ``physics:diagonalInertia`` (three principal moments) plus ``physics:principalAxes`` (the
|   quaternion orienting the principal frame in the body frame), related to the body-frame
|   tensor by
|
|       I_body = R diag(I_p) R^T,      R = principalAxes
|
|   and PhysX itself is the same shape underneath (PxRigidBody keeps a mass-space diagonal
|   inertia plus the CMass pose rotation). So writing a non-diagonal tensor straight into
|   diagonalInertia does not fail -- it SILENTLY DROPS the off-diagonal terms. Every writer
|   of a tensor goes through author_inertia_tensor() instead.
|
|   The conversion is EXACT, not an approximation: the eigendecomposition reproduces the
|   input tensor to ~1e-17. It is not a modelling choice, just the change of representation
|   the engine requires.
"""

import numpy as np
from pxr import Gf, UsdPhysics


def principal_inertia(inertia):
    """Eigen-decompose a body-frame inertia into (principal moments, principal-axes quat).

    Args:
        inertia: symmetric 3x3 tensor, or a length-3 diagonal (-> identity axes).

    Returns:
        (moments, quat_wxyz): moments (3,) principal moments, ascending; quat_wxyz (4,) the
        principal-axes rotation as (w, x, y, z), i.e. I_body = R diag(moments) R^T.
    """
    arr = np.asarray(inertia, dtype=float)
    if arr.shape == (3,):
        arr = np.diag(arr)
    if arr.shape != (3, 3):
        raise ValueError(f"inertia must be 3x3 or length-3, got shape {arr.shape}")
    if not np.allclose(arr, arr.T, atol=1e-12):
        raise ValueError(f"inertia tensor is not symmetric:\n{arr}")

    # eigh: ascending eigenvalues, orthonormal eigenvectors as COLUMNS (each column is a
    # principal axis expressed in body coordinates), so eigvecs IS R above. Mirror one
    # column if the sort produced an improper (reflecting) matrix -- R must be a rotation.
    moments, R = np.linalg.eigh(arr)
    if np.linalg.det(R) < 0.0:
        R[:, 0] = -R[:, 0]

    tr = np.trace(R)                                    # Shepperd's method
    if tr > 0.0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
             (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = (R[k, j] - R[j, k]) / s
        q[i + 1] = 0.25 * s
        q[j + 1] = (R[j, i] + R[i, j]) / s
        q[k + 1] = (R[k, i] + R[i, k]) / s
    q = np.asarray(q, dtype=float)
    return moments, q / np.linalg.norm(q)


def author_inertia_tensor(mass_api, inertia):
    """Write a body-frame inertia tensor onto a UsdPhysics.MassAPI, products included.

    Authors BOTH physics:diagonalInertia and physics:principalAxes. principalAxes is written
    unconditionally -- identity for an already-diagonal input -- so switching a vehicle back
    to a diagonal inertia can never leave a stale principal-axes tilt behind on the prim.

    Must run BEFORE world.reset() on an articulation: PhysX snapshots mass properties when
    the articulation is created, and a later USD write is silently ignored.

    Args:
        mass_api: a UsdPhysics.MassAPI (already applied to the prim).
        inertia: symmetric 3x3 tensor, or a length-3 diagonal.

    Returns:
        (moments, quat_wxyz) as returned by principal_inertia(), for logging.
    """
    moments, quat = principal_inertia(inertia)
    mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*[float(x) for x in moments]))
    mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(*[float(x) for x in quat]))
    return moments, quat


def read_inertia_tensor(prim):
    """Reassemble the body-frame tensor authored on a prim, for verification/logging.

    Returns:
        (3,3) ndarray = R diag(diagonalInertia) R^T, or None if the prim carries no
        diagonalInertia opinion.
    """
    mass_api = UsdPhysics.MassAPI(prim)
    if not mass_api:
        return None
    attr = mass_api.GetDiagonalInertiaAttr()
    if not attr or not attr.HasValue():
        return None
    moments = np.array(attr.Get(), dtype=float)

    q_attr = mass_api.GetPrincipalAxesAttr()
    if q_attr and q_attr.HasValue():
        q = q_attr.Get()
        w = float(q.GetReal())
        x, y, z = (float(v) for v in q.GetImaginary())
    else:
        w, x, y, z = 1.0, 0.0, 0.0, 0.0

    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])
    return R @ np.diag(moments) @ R.T
