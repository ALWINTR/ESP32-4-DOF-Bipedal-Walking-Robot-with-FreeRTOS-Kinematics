import urllib.request
import time
import sys

ROBOT_IP = "192.168.1.22"

def set_angle(ch, val):
    url = f"http://{ROBOT_IP}/set_servo?ch={ch}&val={val}"
    try:
        req = urllib.request.urlopen(url, timeout=3)
        return req.read().decode()
    except Exception as e:
        print(f"Error communicating with robot: {e}")
        return None

def park():
    url = f"http://{ROBOT_IP}/run_gait?gait=stand"
    try:
        urllib.request.urlopen(url, timeout=3)
    except:
        pass

print("=== 6-DOF BIPED ROBOT: HIP & KNEE DIAGNOSTIC TEST ===")
print("Resetting all joints to neutral stand...")
park()
time.sleep(1.5)

# TEST 1: RIGHT HIP (CH 0)
print("\n>>> TEST 1: Right Hip (CH 0)")
print("Moving Right Hip from 94° to 104° (+10°)...")
set_angle(0, 104)
time.sleep(2.5)
print("Returning Right Hip to 94°...")
set_angle(0, 94)
time.sleep(1.5)

# TEST 2: RIGHT KNEE (CH 1)
print("\n>>> TEST 2: Right Knee (CH 1)")
print("Moving Right Knee from 8° to 22° (+14°)...")
set_angle(1, 22)
time.sleep(2.5)
print("Returning Right Knee to 8°...")
set_angle(1, 8)
time.sleep(1.5)

# TEST 3: LEFT HIP (CH 4)
print("\n>>> TEST 3: Left Hip (CH 4)")
print("Moving Left Hip from 94° to 84° (-10°)...")
set_angle(4, 84)
time.sleep(2.5)
print("Returning Left Hip to 94°...")
set_angle(4, 94)
time.sleep(1.5)

# TEST 4: LEFT KNEE (CH 8)
print("\n>>> TEST 4: Left Knee (CH 8)")
print("Moving Left Knee from 94° to 80° (-14°)...")
set_angle(8, 80)
time.sleep(2.5)
print("Returning Left Knee to 94°...")
set_angle(8, 94)
time.sleep(1.5)

print("\n=== DIAGNOSTIC TEST COMPLETE ===")
park()
