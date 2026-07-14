import struct
import numpy as np


def load_stl(path: str):
    with open(path, "rb") as f:
        header = f.read(80)
        rest = f.read()

    is_binary = False
    if len(rest) >= 4:
        n_tri = struct.unpack("<I", rest[:4])[0]
        expected_size = 4 + n_tri * 50
        if expected_size == len(rest):
            is_binary = True

    if is_binary:
        vertices, _ = _load_binary(rest, n_tri)
    else:
        vertices, _ = _load_ascii(header + rest)

    normals = _compute_face_normals(vertices)
    return vertices, normals


def _compute_face_normals(vertices: np.ndarray):
    tri = vertices.reshape(-1, 3, 3)
    e1 = tri[:, 1, :] - tri[:, 0, :]
    e2 = tri[:, 2, :] - tri[:, 0, :]
    n = np.cross(e1, e2)
    lengths = np.linalg.norm(n, axis=1, keepdims=True)
    lengths[lengths == 0] = 1.0
    n = n / lengths
    return np.repeat(n, 3, axis=0).astype(np.float32)


def _load_binary(rest: bytes, n_tri: int):
    dtype = np.dtype([
        ("normal", "<f4", (3,)),
        ("v0", "<f4", (3,)),
        ("v1", "<f4", (3,)),
        ("v2", "<f4", (3,)),
        ("attr", "<u2"),
    ])
    data = np.frombuffer(rest, dtype=dtype, count=n_tri, offset=4)

    vertices = np.stack([data["v0"], data["v1"], data["v2"]], axis=1).reshape(-1, 3)
    normals = np.repeat(data["normal"], 3, axis=0)

    return vertices.astype(np.float32), normals.astype(np.float32)


def _load_ascii(raw: bytes):
    text = raw.decode("utf-8", errors="ignore")
    tokens = text.split()

    normals = []
    verts = []

    i = 0
    n = len(tokens)
    while i < n:
        tok = tokens[i]
        if tok == "facet" and i + 4 < n and tokens[i + 1] == "normal":
            normals.append((
                float(tokens[i + 2]),
                float(tokens[i + 3]),
                float(tokens[i + 4]),
            ))
            i += 5
        elif tok == "vertex" and i + 3 < n:
            verts.append((
                float(tokens[i + 1]),
                float(tokens[i + 2]),
                float(tokens[i + 3]),
            ))
            i += 4
        else:
            i += 1

    vertices = np.array(verts, dtype=np.float32).reshape(-1, 3)
    normals_arr = np.array(normals, dtype=np.float32)
    normals_per_vertex = np.repeat(normals_arr, 3, axis=0)

    return vertices, normals_per_vertex


def load_stl_filtered(path: str, y_min=None, y_max=None):
    v, n = load_stl(path)
    tri_v = v.reshape(-1, 3, 3)
    tri_n = n.reshape(-1, 3, 3)
    centroid_y = tri_v[:, :, 1].mean(axis=1)

    mask = np.ones_like(centroid_y, dtype=bool)
    if y_min is not None:
        mask &= centroid_y >= y_min
    if y_max is not None:
        mask &= centroid_y <= y_max

    v_out = tri_v[mask].reshape(-1, 3)
    n_out = tri_n[mask].reshape(-1, 3)
    return v_out.astype(np.float32), n_out.astype(np.float32)
