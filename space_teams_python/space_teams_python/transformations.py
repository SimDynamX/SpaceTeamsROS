import numpy as np
import numpy.typing as npt


def normalize(vec: npt.NDArray) -> npt.NDArray:
    mag = np.linalg.norm(vec)
    if mag > 0.0:
        return vec / np.linalg.norm(vec)
    else:
        return vec


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolate on the scale given by a to b, using t as the point on that scale.
    Examples
    --------
        50 == lerp(0, 100, 0.5)
        4.2 == lerp(1, 5, 0.8)
    """
    return (1 - t) * a + t * b


def inv_lerp(a: float, b: float, v: float) -> float:
    """Inverse Linar Interpolation, get the fraction between a and b on which v resides.
    Examples
    --------
        0.5 == inv_lerp(0, 100, 50)
        0.8 == inv_lerp(1, 5, 4.2)
    """
    return (v - a) / (b - a)


def remap(i_min: float, i_max: float, o_min: float, o_max: float, v: float) -> float:
    """Remap values from one linear scale to another, a combination of lerp and inv_lerp.
    i_min and i_max are the scale on which the original value resides,
    o_min and o_max are the scale to which it should be mapped.
    Examples
    --------
        45 == remap(0, 100, 40, 50, 50)
        6.2 == remap(1, 5, 3, 7, 4.2)
    """
    return lerp(o_min, o_max, inv_lerp(i_min, i_max, v))


def clamp(v_min: float, v_max: float, v: float) -> float:
    return max(v_min, min(v_max, v))


def remap_clamp(i_min: float, i_max: float, o_min: float, o_max: float, v: float) -> float:
    return clamp(o_min, o_max, remap(i_min, i_max, o_min, o_max, v))


def kph_to_mps(speed_kph: float) -> float:
    return speed_kph / 3.6


def mps_to_kph(speed_mps: float) -> float:
    return speed_mps * 3.6


class Quat:
    def __init__(self, w: float, x: float, y: float, z: float):
        self.w = w
        self.x = x
        self.y = y
        self.z = z
    
    @classmethod
    def FromMatrix(quat, m: npt.NDArray):
        r__ = np.array([m[0, 0], m[0, 1], m[0, 2], 
                        m[1, 0], m[1, 1], m[1, 2], 
                        m[2, 0], m[2, 1], m[2, 2]])
        s = np.zeros(3)

        trace = r__[0] + r__[4] + r__[8]
        mtrace = 1. - trace
        cc4 = trace + 1.
        s114 = mtrace + r__[0] * 2.
        s224 = mtrace + r__[4] * 2.
        s334 = mtrace + r__[8] * 2.

        if (1. <= cc4):
            c__ = np.sqrt(cc4 * .25)
            factor = 1. / (c__ * 4.)
            s[0] = (r__[5] - r__[7]) * factor
            s[1] = (r__[6] - r__[2]) * factor
            s[2] = (r__[1] - r__[3]) * factor
        elif (1. <= s114):
            s[0] = np.sqrt(s114 * .25)
            factor = 1. / (s[0] * 4.)
            c__ = (r__[5] - r__[7]) * factor
            s[1] = (r__[3] + r__[1]) * factor
            s[2] = (r__[6] + r__[2]) * factor
        elif (1. <= s224):
            s[1] = np.sqrt(s224 * .25)
            factor = 1. / (s[1] * 4.)
            c__ = (r__[6] - r__[2]) * factor
            s[0] = (r__[3] + r__[1]) * factor
            s[2] = (r__[7] + r__[5]) * factor
        else:
            s[2] = np.sqrt(s334 * .25)
            factor = 1. / (s[2] * 4.)
            c__ = (r__[1] - r__[3]) * factor
            s[0] = (r__[6] + r__[2]) * factor
            s[1] = (r__[7] + r__[5]) * factor

        q = np.zeros(4)
        l2 = c__ * c__ + s[0] * s[0] + s[1] * s[1] + s[2] * s[2];
        if (l2 != 1.):
            polish = 1. / np.sqrt(l2)
            c__ *= polish
            s[0] *= polish
            s[1] *= polish
            s[2] *= polish
        if (c__ > 0.):
            q[0] = c__
            q[1] = s[0]
            q[2] = s[1]
            q[3] = s[2]
        else:
            q[0] = -c__
            q[1] = -s[0]
            q[2] = -s[1]
            q[3] = -s[2]
        
        return quat(q[0], q[1], q[2], q[3])

    def mult(self, other):
        w1 = self.w
        x1 = self.x
        y1 = self.y
        z1 = self.z
        w2 = other.w
        x2 = other.x
        y2 = other.y
        z2 = other.z
        return Quat(w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                          w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                          w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                          w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)
    
    def normalize(self):
        mag_1 = 1.0 / np.sqrt(self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2)
        return Quat(self.w * mag_1, self.x * mag_1, self.y * mag_1, self.z * mag_1)
    
    def conjugate(self):
        return Quat(-self.w, self.x, self.y, self.z)
    
    def as_w_first_array(self):
        return np.array([self.w, self.x, self.y, self.z])
    
    def as_w_last_array(self):
        return np.array([self.x, self.y, self.z, self.w])
    
    def to_matrix(self):
        # double l2, q01, q02, q03, q12, q13, q23, sharpn, q1s, q2s, q3s;

        q01 = self.w * self.x
        q02 = self.w * self.y
        q03 = self.w * self.z
        q12 = self.x * self.y
        q13 = self.x * self.z
        q23 = self.y * self.z
        q1s = self.x * self.x
        q2s = self.y * self.y
        q3s = self.z * self.z

        l2 = self.w * self.w + q1s + q2s + q3s
        if l2 != 1.0 and l2 != 0.0: 
            sharpn = 1.0 / l2
            q01 *= sharpn
            q02 *= sharpn
            q03 *= sharpn
            q12 *= sharpn
            q13 *= sharpn
            q23 *= sharpn
            q1s *= sharpn
            q2s *= sharpn
            q3s *= sharpn

        m = np.zeros((3, 3))
        m[0][0] = 1.0 - (q2s + q3s) * 2.0
        m[0][1] = (q12 + q03) * 2.0
        m[0][2] = (q13 - q02) * 2.0
        m[1][0] = (q12 - q03) * 2.0
        m[1][1] = 1.0 - (q1s + q3s) * 2.0
        m[1][2] = (q23 + q01) * 2.0
        m[2][0] = (q13 + q02) * 2.0
        m[2][1] = (q23 - q01) * 2.0
        m[2][2] = 1.0 - (q1s + q2s) * 2.0

        return m
