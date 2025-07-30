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

def spline_deriv(spline, u):
    a, b, c, d = spline
    return 3 * a * u**2 + 2 * b * u + c

# image=Image.open("into-the-deep.png")
image = Image.open("D:\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\bouncyFTC\\into-the-deep.png")
image=image.resize((144, 144), Image.Resampling.LANCZOS)

plt.imshow(np.rot90(np.array(image),3),extent=[-72,72,-72,72])
u = np.linspace(0, 1, 100)

# 0-5
x_spline = spline_fit(-6, 20, -6, 20)
y_spline = spline_fit(60, 0, 36, 0)
plt.plot(spline_get(x_spline, u), spline_get(y_spline, u))
# plt.show()

# 5-10
x_spline = spline_fit(-6, -100, -48, 0)
y_spline = spline_fit(36, 100, 12, 100)
plt.plot(spline_get(x_spline, u), spline_get(y_spline, u))

# 10-15
x_spline = spline_fit(-48, 0, -48, 0)
y_spline = spline_fit(12, 100, 48, 0)
plt.plot(spline_get(x_spline, u), spline_get(y_spline, u))

# 15-20
x_spline = spline_fit(-48, 0, -57, 0)
y_spline = spline_fit(48, 0, 12, 100)
plt.plot(spline_get(x_spline, u), spline_get(y_spline, u))

# 20-25
x_spline = spline_fit(-57, 0, -57, 0)
y_spline = spline_fit(12, 0, 48, 0)
plt.plot(spline_get(x_spline, u), spline_get(y_spline, u))

# 25-30
x_spline = spline_fit(-57, 0, -6, 0)
y_spline = spline_fit(48, 0, 36, 0)
plt.plot(spline_get(x_spline, u), spline_get(y_spline, u))

# 30-35
x_spline = spline_fit(-6, 0, -48, 0)
y_spline = spline_fit(36, 0, 60, 0)
plt.plot(spline_get(x_spline, u), spline_get(y_spline, u))

plt.title('Trajectory')
plt.xlabel('x [in]')
plt.ylabel('y [in]')

# plt.plot(spline_deriv(x_spline, u), spline_deriv(y_spline, u))
plt.show()