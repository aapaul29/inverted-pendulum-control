# Physical parameters for the triple inverted pendulum on a cart.
# Each link is modelled as a uniform rod (CoM at l/2, I = m*l^2/3 about joint).
# All angles measured from the upright vertical (theta = 0 => upright equilibrium).

g = 9.81   # m/s^2

# Cart
M = 1.0    # kg
b = 0.1    # cart friction [N·s/m]

# Link 1 (bottom)
m1  = 0.5
l1  = 0.6
lc1 = l1 / 2
I1  = m1 * l1**2 / 3
b1  = 0.01  # joint damping [N·m·s/rad]

# Link 2 (middle)
m2  = 0.4
l2  = 0.5
lc2 = l2 / 2
I2  = m2 * l2**2 / 3
b2  = 0.01

# Link 3 (top)
m3  = 0.3
l3  = 0.4
lc3 = l3 / 2
I3  = m3 * l3**2 / 3
b3  = 0.01
