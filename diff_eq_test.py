
import numpy as np
import matplotlib.pyplot as plt

k=-1
f = lambda y: k*y



def euler(all_y, h):
    return all_y[-1] + h*f(all_y[-1])


def runge_kutta(all_y, h):
    k1 = f(all_y[-1])
    k2 = f(all_y[-1] + h/2*k1)
    k3 = f(all_y[-1] + h/2*k2)
    k4 = f(all_y[-1] + h*k3)
    return all_y[-1] + h/6*(k1 + 2*k2 + 2*k3 + k4)


t_max = 10
t_dense = np.linspace(0, t_max, 100)
y_exact = np.exp(k*t_dense)
plt.plot(t_dense, y_exact, label='Exact')

for h in [0.5, 1, 1.5, 2, 2.5]:
    all_t = np.arange(0, t_max, h)
    all_y = [1]
    for t in all_t[:-1]:
        # all_y.append(euler(all_y, h))
        all_y.append(runge_kutta(all_y, h))
    # plt.plot(all_t, all_y, 'o-', label=f'Euler h={h}')
    plt.plot(all_t, all_y, 'o-', label=f'Runge-Kutta h={h}')



plt.legend()

plt.show()
    


















