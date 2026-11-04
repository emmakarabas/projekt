import numpy as np

def robotcircle(b, wL, wR, x0, y0, theta0):
    ''' 
    Cirkelbana med konstant hjulfart
    Returnerar mittpunkt (xbar, ybar), R och varvtid (Tlap)
    '''
    w = (wR + wL)/2 #medelfart
    Dw = (wR - wL)/b #vinkelhastighet
    xbar = x0 - (w/Dw)*np.sin(theta0)
    ybar = y0 + (w/Dw)*np.cos(theta0)
    R = abs(w/Dw)
    Tlap = 2*np.pi/abs(Dw)
    return xbar, ybar, R, Tlap