# https://qiita.com/acela86/items/170dba21021e387dfc89
from sympy import sin, cos, symbols, diff, Matrix
from sympy import solve, simplify
from sympy.physics.mechanics import LagrangesMethod, Lagrangian
from sympy.physics.mechanics import ReferenceFrame, Particle, Point
from sympy.physics.mechanics import dynamicsymbols, kinetic_energy
from sympy.physics.mechanics import mprint, mlatex

t = symbols('t')
m1, m2, l, g = symbols('m1 m2 l g')
p, theta, F = dynamicsymbols('p theta F')

q = Matrix([p, theta])
qd = q.diff(t)

N = ReferenceFrame('N')
P1 = Point('P1')
P2 = Point('P2')

x1 = p
y1 = 0
x2 = p - l * sin(theta)
y2 = l * cos(theta)

Pa1 = Particle('Pa1', P1, m1)
Pa2 = Particle('Pa2', P2, m2)

vx1 = diff(x1, t)
vy1 = diff(y1, t)
vx2 = diff(x2, t)
vy2 = diff(y2, t)

P1.set_vel(N, vx1 * N.x + vy1 * N.y)
P2.set_vel(N, vx2 * N.x + vy2 * N.y)

Pa1.potential_energy = m1 * g * y1
Pa2.potential_energy = m2 * g * y2

fl = [(P1, F*N.x), (P2, 0*N.x)]

LL = Lagrangian(N, Pa1, Pa2)
LM = LagrangesMethod(LL, q, forcelist = fl, frame = N)

eom = LM.form_lagranges_equations().simplify()
f = LM.rhs().simplify()
mprint(eom)
