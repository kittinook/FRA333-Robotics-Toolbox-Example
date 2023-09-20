import numpy as np
from spatialmath import SE3

# การสร้าง transformation จาก translation และ rotation
# ใช้ translation แกน x=1, y=2, z=3 และ rotation รอบแกน z 30 องศา
T = SE3(1, 2, 3) * SE3.Rz(np.radians(30))

print("Transformation T:")
print(T)

# การหา inverse transformation
T_inv = T.inv()

print("\nInverse Transformation T_inv:")
print(T_inv)

# การคูณ transformation กับจุด
point = [4, 5, 6]
transformed_point = T * point

print("\nTransformed Point:")
print(transformed_point)

# การคูณ transformation เข้าด้วยกัน
T2 = SE3(2, 3, 4) * SE3.Rx(np.radians(45))
resultant_transform = T * T2

print("\nResultant Transformation after multiplication:")
print(resultant_transform)
