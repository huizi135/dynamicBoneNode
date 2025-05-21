import matplotlib.pyplot as plt
# 绘制一阶欧拉示意
# 模拟简单弹簧阻尼系统： d2x/dt2 = -（k*x + mu*v）
# 系统参数

m = 1.0    # 质量
k = 10.0   # 弹簧刚度
mu = 1.0   # 阻尼系数

# 初始条件
x = 1.0
v = 0.0

dt = 0.2    # 时间步长
T = 10      # 总模拟时间
steps = int(T/dt)

xs = [x]
vs = [v]
ts = [0.0]

for i in range(steps):
    a = -(k/m)*x - (mu/m) * v
    v += a * dt
    x += v * dt
    
    xs.append(x)
    vs.append(v)
    ts.append(ts[-1] + dt)
    
# 绘图
plt.plot(ts, xs)
plt.title("Spring-Damper using Explicit Euler Method")
plt.xlabel("Time")
plt.ylabel("Displacement")
plt.grid(True)
plt.show()