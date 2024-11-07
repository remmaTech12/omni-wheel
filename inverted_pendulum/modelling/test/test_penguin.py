# http://penguinitis.g1.xrea.com/study/system_control/moe.html
from sympy import symbols, diff, Matrix, lambdify
from sympy.physics.mechanics import dynamicsymbols
from sympy.physics.mechanics import ReferenceFrame, Point, Particle
from sympy.physics.mechanics import LagrangesMethod, Lagrangian
from sympy.physics.mechanics import mprint, mlatex

# preparation of variables and constants
t = symbols("t")
z1, z2, f1, f2 = dynamicsymbols("z1 z2 f1 f2")
m1, m2, c1, c2, k1, k2 = symbols("m1 m2 c1 c2 k1 k2")
q = Matrix([z1, z2])


# setting mass point
N = ReferenceFrame("N")

p1 = Point("p1")
v1 = z1.diff(t)
p1.set_vel(N, v1*N.x)
pa1 = Particle("pa1", p1, m1)
pa1.potential_energy = k1*z1**2/2 + k2*(z2 - z1)**2/2

p2 = Point("p2")
v2 = z2.diff(t)
p2.set_vel(N, v2*N.x)
pa2 = Particle("pa2", p2, m2)


# setting force
F = c1*v1**2/2 + c2*(v2 - v1)**2/2
fc1 = -F.diff(v1)
fc2 = -F.diff(v2)
fl = [(p1, (f1 + fc1)*N.x), (p2, (f2 + fc2)*N.x)]


# Lagrange
L = Lagrangian(N, pa1, pa2)
LM = LagrangesMethod(L, q, forcelist=fl, frame=N)
LM.form_lagranges_equations()


# Linearization
As, Bs, u = LM.linearize(q_ind=q, qd_ind=q.diff(t), A_and_B=True)
mprint(As)
mprint(Bs)
mprint(u)
