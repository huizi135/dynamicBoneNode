import matplotlib.pyplot as plt

# 系统参数
m = 1.0     #质量
k = 10.0    #弹簧刚度
mu = 1.0    #阻尼系数

# 初始状态
x = 1.0     # 初始位移
v = 0.0     # 初始速度

dt = 0.05    # 时间步长
T = 10       # 总模拟时间
steps = int(T / dt)

# 保存轨迹
xs, vs, ts = [x], [v], [0.0]

# 定义微分函数
def derivatives(x, v):
    dxdt = v
    dvdt = -(mu/m) * v - (k/m) * x
    return dxdt, dvdt

# RK4 积分主循环
for i in range(steps):
    dx1, dv1 = derivatives(x, v)
    dx2, dv2 = derivatives(x + 0.5*dt*dx1, v + 0.5*dt*dv1)
    dx3, dv3 = derivatives(x + 0.5*dt*dx2, v + 0.5*dt*dv2)
    dx4, dv4 = derivatives(x + dt*dx3, v + dt*dv3)
    
    x += dt / 6.0 * (dx1 + 2*dx2 + 2*dx3 + dx4)
    v += dt / 6.0 * (dv1 + 2*dv2 + 2*dv3 + dv4)
    
    xs.append(x)
    vs.append(v)
    ts.append(ts[-1] + dt)
    
# 绘图
plt.plot(ts, xs)
plt.title("Spring-Damper using RK4")
plt.xlabel("Time")
plt.ylabel("Displacement")
plt.grid(True)
plt.show()