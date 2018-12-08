import numpy as np

def data_adapter(q, t):
    if not isinstance(q, np.ndarray):
        q = np.array([q.x, q.y, q.z, q.w])
    if not isinstance(t, np.ndarray):
        t = np.array([t.x, t.y, t.z])
    return q,t