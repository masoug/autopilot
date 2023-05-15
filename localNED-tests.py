import numpy as np
from navpy import lla2ecef, lla2ned

input_lla = (37.81123473439707, -122.4774845440041, 123.4)
print(f'input={input_lla}')
print(f"output={lla2ecef(input_lla[0], input_lla[1], input_lla[2], model='')}\n")

input_lla = (37.812214083769526, -122.4776198744307, 123.4)
print(f'input={input_lla}')
print(f"output={lla2ecef(input_lla[0], input_lla[1], input_lla[2], model='')}\n")

input_lla = (37.81299241805834, -122.47772272625046, 123.4)
print(f'input={input_lla}')
print(f"output={lla2ecef(input_lla[0], input_lla[1], input_lla[2], model='')}\n")

origin = (37.81123473439707, -122.4774845440041, 123.4)
print(f'origin={origin}')

input_lla = (37.812214083769526, -122.4776198744307, 3.14)
print(f'input={input_lla}')
print(f"output={lla2ned(input_lla[0], input_lla[1], input_lla[2], origin[0], origin[1], origin[2], model='')}\n")

input_lla = (37.81299241805834, -122.47772272625046, 543.21)
print(f'input={input_lla}')
print(f"output={lla2ned(input_lla[0], input_lla[1], input_lla[2], origin[0], origin[1], origin[2], model='')}\n")
