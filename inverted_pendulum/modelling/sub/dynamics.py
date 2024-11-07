from sympy import sin, cos, symbols, diff, Matrix
from sympy import solve, simplify
from sympy.physics.mechanics import *
import math

#------------------------------------------------------------------------------------------#
# system setting
## coordinate
Sw = ReferenceFrame('Sw')
Sb = ReferenceFrame('Sb')
Sptmp = ReferenceFrame('Sptmp')
Sp = ReferenceFrame('Sp')
Sm2 = ReferenceFrame('Sm2')
Sm3 = ReferenceFrame('Sm3')

## general
t, g = symbols('t g')

## base
mb, lb, rw = symbols('mb lb rw')
Ibxx, Ibyy, Ibzz = symbols('Ibxx Ibyy Ibzz')
x, y, alpha = dynamicsymbols('x y alpha')
Ib = inertia(Sb, 0, 0, Ibzz)

## pendulum
#ms, mm, mp, lgp, lp, lm = symbols('ms mm mp lgp lp lm') # full description
mp, lgp = symbols('mp lgp')
Ipxx, Ipyy, Ipzz = symbols('Ipxx Ipyy Ipzz')
phi, theta = dynamicsymbols('phi theta')
xi, eta, zeta = dynamicsymbols('xi eta zeta')
Ip = inertia(Sp, Ipxx, Ipxx, 0)
#Ip = inertia(Sp, 1/12*mp*(lgp*2)**2, 1/12*mp*(lgp*2)**2, 0)

## motor
f1, f2, f3 = dynamicsymbols('f1 f2 f3')
#------------------------------------------------------------------------------------------#


#------------------------------------------------------------------------------------------#
# basic calculation
## state
q = Matrix([x, y, alpha, phi, theta])
qd = q.diff(t)

## coordinates transformation
Sb.orient(Sw, 'Axis', [alpha, Sw.z])

Sptmp.orient(Sw, 'Axis', [phi, Sw.x]);
Sp.orient(Sptmp, 'Axis', [theta, Sptmp.y]);
#Sp.set_ang_vel(Sw, phi.diff(t)*cos(theta)*Sw.x + theta.diff(t)*Sw.y + sin(theta)*phi.diff(t)*Sw.z)
print(Sp.ang_vel_in(Sw))

## point
### world
Pwo = Point('Pwo')
Pwo.set_vel(Sw, 0)

### body
Pbo = Point('Pbo')
Pbo.set_pos(Pwo, x*Sw.x + y*Sw.y + 0*Sw.z)
Pbo.set_vel(Sw, Pbo.pos_from(Pwo).dt(Sw))

### pendulum
Ppo = Point('Ppo')
Ppo.set_pos(Pwo, x*Sw.x + y*Sw.y + lgp*Sp.z)
Ppo.set_vel(Sw, Ppo.pos_from(Pwo).dt(Sw))

### motor
a1, a2, a3 = symbols('a1 a2 a3') # a1: 0, a2: 2*math.pi/3, a3: 4*math.pi/3

Sm1 = Sb
Pm1 = Point("Pm1")
Pm1.set_pos(Pbo, lb*Sm1.x + 0*Sm1.y + 0*Sm1.z)
#Pm1.set_vel(Sw, Pm1.pos_from(Pwo).dt(Sw))

Sm2.orient(Sb, 'Axis', [a2, Sb.z])
Pm2 = Point("Pm2")
Pm2.set_pos(Pbo, lb*Sm2.x + 0*Sm2.y + 0*Sm2.z)
#Pm2.set_vel(Sw, Pm2.pos_from(Pwo).dt(Sw))

Sm3.orient(Sb, 'Axis', [a3, Sb.z])
Pm3 = Point("Pm3")
Pm3.set_pos(Pbo, lb*Sm3.x + 0*Sm3.y + 0*Sm3.z)
#Pm3.set_vel(Sw, Pm3.pos_from(Pwo).dt(Sw))

## position and velocity
BodyB = RigidBody('BodyB', Pbo, Sb, mb, (Ib, Pbo))
BodyP = RigidBody('BodyP', Ppo, Sp, mp, (Ip, Ppo))
p_cog_pos = (Ppo.pos_from(Pwo).express(Sw)).simplify()
print(p_cog_pos)

BodyP.potential_energy = mp * lgp * cos(phi) * cos(theta);
print(BodyB.kinetic_energy)

## force from wheels
fl1 = (Pm1, f1*Sm1.y)
fl2 = (Pm2, f2*Sm2.y)
fl3 = (Pm3, f3*Sm3.y)
fl = [fl1, fl2, fl3]
#------------------------------------------------------------------------------------------#


#------------------------------------------------------------------------------------------#
# Lagrangian
LL = Lagrangian(Sw, BodyB, BodyP)
LM = LagrangesMethod(LL, q, forcelist = fl, frame = Sw)

eom = LM.form_lagranges_equations().simplify()
#f = LM.rhs().simplify()

linearizer = LM.to_linearizer(q_ind=q, qd_ind=qd)
op_point = {x: 0, x.diff(t): 0, y: 0, y.diff(t): 0, alpha: 0, alpha.diff(t): 0,
            phi: 0, phi.diff(t): 0, theta: 0, theta.diff(t): 0,
            f1: 0, f2: 0, f3: 0}
A, B = linearizer.linearize(A_and_B = True, op_point = op_point)
mprint(A)
mprint(B)
display(A)
#------------------------------------------------------------------------------------------#
