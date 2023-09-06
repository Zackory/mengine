import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground([0, 0, -0.5])
env.set_gui_camera(look_at_pos=[0, 0, 0])
m.visualize_coordinate_frame()

# line 1
p1 = np.array([0.4, 0.3, 0.2])
q1 = np.array([0.1, 0.2, 0.3])
# p1 = np.array([1, 0, 0]) # Example in class
# q1 = np.array([0, 1, 1])
q1_norm = q1 / np.linalg.norm(q1)
line1 = m.Line(p1 - q1_norm, p1 + q1_norm, radius=0.005, rgba=[1, 0, 0, 1])
point1 = m.Shape(m.Sphere(radius=0.02), static=True, position=p1, rgba=[1, 0, 0, 1])

# line 2
p2 = np.array([-0.2, 0.4, 0.3])
q2 = np.array([-0.2, 0.2, 0.1])
# p2 = np.array([0, 1, 1]) # Example in class
# q2 = np.array([-1, 0, 1])
q2_norm = q2 / np.linalg.norm(q2)
line2 = m.Line(p2 - q2_norm, p2 + q2_norm, radius=0.005, rgba=[0, 1, 0, 1])
point2 = m.Shape(m.Sphere(radius=0.02), static=True, position=p2, rgba=[0, 1, 0, 1])

reciprocal_product = lambda p1, q1, p2, q2: q1.dot(np.cross(p2, q2)) + q2.dot(np.cross(p1, q1))

# Moment of line 2 about line 1 (equal to moments of line 1 about line 2)
moment = reciprocal_product(p1, q1, p2, q2) / (np.linalg.norm(q1)*np.linalg.norm(q2))
print('Moment of line 2 about line 1:', moment)

# Angle between lines
angle = np.arcsin(np.linalg.norm(np.cross(q1, q2)) / (np.linalg.norm(q1)*np.linalg.norm(q2)))
print('Angle between lines 1 and 2 (degrees):', np.degrees(angle))

# Signed distance between lines:
distance = reciprocal_product(p1, q1, p2, q2) / np.linalg.norm(np.cross(q2, q1))
print('Signed distance between lines 1 and 2:', distance)

print('Does line 1 and 2 intersect?', np.isclose(reciprocal_product(p1, q1, p2, q2), 0), 'reciprocal product:', reciprocal_product(p1, q1, p2, q2))

# line 3
p3 = np.array([0.2, -0.2, 0.4])
q3 = np.array([0.2, 0.5, -0.2])
q3_norm = q3 / np.linalg.norm(q3)
line3 = m.Line(p3 - q3_norm, p3 + q3_norm, radius=0.005, rgba=[0, 0, 1, 1])
point3 = m.Shape(m.Sphere(radius=0.02), static=True, position=p3, rgba=[0, 0, 1, 1])

print('Does line 1 and 3 intersect?', np.isclose(reciprocal_product(p1, q1, p3, q3), 0), 'reciprocal product:', reciprocal_product(p1, q1, p3, q3))

moment = reciprocal_product(p1, q1, p3, q3) / (np.linalg.norm(q1)*np.linalg.norm(q3))
print('Moment of line 3 about line 1:', moment)
angle = np.arcsin(np.linalg.norm(np.cross(q1, q3)) / (np.linalg.norm(q1)*np.linalg.norm(q3)))
print('Angle between lines 1 and 3 (degrees):', np.degrees(angle))
distance = reciprocal_product(p1, q1, p3, q3) / np.linalg.norm(np.cross(q3, q1))
print('Signed distance between lines 1 and 3:', distance)

m.step_simulation(steps=10000, realtime=True)


