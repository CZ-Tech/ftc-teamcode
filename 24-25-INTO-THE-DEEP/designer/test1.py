import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

def spline_fit(x0, dx0, x1, dx1):
    a = 2 * x0 + dx0 - 2 * x1 + dx1
    b = -3 * x0 - 2 * dx0 + 3 * x1 - dx1
    c = dx0
    d = x0
    return a, b, c, d

def spline_get(spline, u):
    a, b, c, d = spline
    return a * u**3 + b * u**2 + c * u + d


u = np.linspace(0, 1, 100)
x_spline = spline_fit(-6, 0, -6, 0)
y_spline = spline_fit(60, 0, 36, 0)
print(x_spline)
print(spline_get(x_spline, u))