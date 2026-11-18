import numpy as np
import matplotlib.pyplot as plt
from scipy.special import j0

b = 1.0 #avstånd mellan hjulen
a = 1.0 #parametern a i g(t) 

#Hjulens fart
g = lambda t: a * np.sin(3 * j0(2 * a) * t)
vL = lambda t: np.cos(g(t))**2
vR = lambda t: np.sin(g(t))**2

#ODE-systemets högerled. Samma odefun som på de andra två uppgifterna
f = lambda t, U: odefun(t, U, b, vR=vR, vL=vL) #odefun från 3 eller 4

#Tid o begynnelsevärden
t0, tf = 0.0, 30.0 #Start och sluttid (s)
U0 = np.array([0.0, 0.0, 0.0]) #Begynnelsevärden [x0, y0, theta0]

#Simulera banan till tf = 30 och plotta
N = 2400 #antal steg
t, U = rk4(f, t0, tf, U0, N)
x     = U[0] #x-koordinat vid varje tidpunkt
y     = U[1] #y-koordinat vid varje tidpunkt
theta = U[2] #orientering vid varje tidpunkt

v = (vL(t) + vR(t))/2
print("v min/max:", v.min(), v.max())

# Plotta banan
plt.figure()
plt.plot(x, y)
plt.plot(0, 0, 'go', label='start') #markera startpunkten
plt.axis('equal') #lika skala på axlarna, annars förvrids banan
plt.xlabel('x')
plt.ylabel('y')
plt.title('Robotens bana, a = 1')
plt.legend()
plt.show()

#5a
def find_return_time(N, deg=5):
    '''
    Beräknar returtiden T med rk4 (N steg). Returnerar (T, y(T)).
    '''
    t, U = rk4(f, t0, tf, U0, N)
    x, y = U[0], U[1]

    #Index k där x byter tecken mellan t[k] och t[k+1]
    sc = sc[sc > 0]
    cand = sc[t[sc] > 1.0] #ta bort teckenbytet vid start (t = 0)
    k = cand[np.argmin(np.abs(y[cand]))] #välj det där y ≈ 0 (roboten är hemma)

    # Interpolera x(t) med ett polynom genom deg+1 punkter runt k och hitta nollstället
    m = deg // 2
    sl = slice(k - m, k + m + 2)
    tau = t[sl] - t[k] #förskjuten tid ger bättre numerik
    r = np.roots(np.polyfit(tau, x[sl], deg))
    r = r[np.isreal(r)].real
    r = r[(r >= 0) & (r <= t[k+1] - t[k])] #nollstället i intervallet [t[k], t[k+1]]
    T = t[k] + r[0]

    yT = np.polyval(np.polyfit(tau, y[sl], deg), T - t[k])
    return T, yT

# Halvera steglängden och se att T stabiliserar sig
for N in [150, 300, 600, 1200, 2400]:
    T, yT = find_return_time(N)
    print(f"N={N:5d}  T={T:.12f}  y(T)={yT:.1e}")

T = find_return_time(1200)[0]    # slutligt T (används i 5b)
print("T =", T)

#5b
def simpson(t, F):
    h = t[1] - t[0]
    return h/3*(F[0] + 4*np.sum(F[1:-1:2]) + 2*np.sum(F[2:-1:2]) + F[-1])

def area_from_rk4(N):
    t, U = rk4(f, 0.0, T, U0, N)
    x, y, th = U[0], U[1], U[2]
    v = (vL(t) + vR(t))/2 #robotens fart
    F = 0.5*v*(x*np.sin(th) - y*np.cos(th)) #integranden
    return abs(simpson(t, F))

#Halvera steglängden och se att A stabiliserar sig
for N in [100, 200, 400, 800, 1600, 3200]:
    print(f"N={N:5d}  A={area_from_rk4(N):.10f}")